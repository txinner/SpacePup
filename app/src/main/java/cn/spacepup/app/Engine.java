package cn.spacepup.app;

import android.content.Context;
import android.media.Image;
import android.opengl.GLSurfaceView;
import android.os.SystemClock;
import android.os.BatteryManager;
import android.util.Log;
import static android.opengl.GLES20.*;
import com.google.ar.core.*;
import com.google.ar.core.exceptions.NotYetAvailableException;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import java.io.File;
import java.util.*;

final class Engine implements GLSurfaceView.Renderer {
    enum Mode { HOME, ORIGIN, SCAN, SEARCH, VISUAL }
    interface Listener {void status(Mode mode,String title,String detail,boolean saveReady);void message(String text);void saved(String name);void failed(String error);}
    final Context context;final Listener ui;final File dir;
    final Scene scene=new Scene();
    volatile Mode mode=Mode.HOME;
    private Session session;private boolean resumed,suspended;
    private Anchor origin, placedDog;private Vision vision,validator;private Space space;
    private Rigid validationStart;
    private long validationSince;
    private int width=1,height=1,rotation;
    private Rigid mapCamera;
    private final MapRefresh mapRefresh=new MapRefresh();
    private long lastMapMatch;
    private final float[] projection=new float[16];
    private long sampleTime,uiTime;
    private boolean place, setOrigin;
    private double lastCorrection=Double.NaN;
    private long traceTime;
    private long lastChargeRead=-10000,charge;
    final ArrayDeque<String> trace=new ArrayDeque<>();
    Engine(Context ctx,File dir,Listener ui){context=ctx;this.dir=dir;this.ui=ui;}
    static Rigid from(Pose p){return new Rigid(Rigid.doubles(p.getTranslation()),Rigid.doubles(p.getRotationQuaternion()));}
    static Pose toPose(Rigid p){return new Pose(Rigid.floats(p.t),Rigid.floats(p.q));}
    public void onSurfaceCreated(GL10 gl,EGLConfig config){
        scene.init();if(session!=null)session.setCameraTextureName(scene.texture);
    }
    public void onSurfaceChanged(GL10 gl,int w,int h){
        width=w;height=h;glViewport(0,0,w,h);scene.invalidateCameraGeometry();
        if(session!=null)session.setDisplayGeometry(rotation,w,h);
    }
    void start(Space target,boolean creating,int displayRotation){
        reset();suspended=false;trace.clear();traceTime=0;lastCorrection=Double.NaN;space=target;rotation=displayRotation;
        mode=creating?Mode.ORIGIN:Mode.SEARCH;
        openCamera();
    }
    private void openCamera(){
        try{
            vision=new Vision();session=new Session(context);
            CameraConfigFilter filter=new CameraConfigFilter(session)
                .setTargetFps(EnumSet.of(CameraConfig.TargetFps.TARGET_FPS_30))
                .setDepthSensorUsage(EnumSet.of(CameraConfig.DepthSensorUsage.DO_NOT_USE));
            List<CameraConfig> cameras=session.getSupportedCameraConfigs(filter);
            if(cameras.isEmpty()){
                filter.setDepthSensorUsage(EnumSet.allOf(CameraConfig.DepthSensorUsage.class));
                cameras=session.getSupportedCameraConfigs(filter);
            }
            if(!cameras.isEmpty())session.setCameraConfig(cameras.get(0));
            Config config=new Config(session);config.setFocusMode(Config.FocusMode.AUTO);
            config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE);
            config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
            config.setPlaneFindingMode(mode==Mode.ORIGIN || mode==Mode.SCAN?Config.PlaneFindingMode.HORIZONTAL:Config.PlaneFindingMode.DISABLED);
            session.configure(config);session.setCameraTextureName(scene.texture);
            scene.invalidateCameraGeometry();
            session.setDisplayGeometry(rotation,width,height);session.resume();resumed=true;
            sampleTime=0;mapCamera=null;
            lastMapMatch=0;mapRefresh.reset(0,false);
        }catch(Exception | LinkageError e){
            fail(e);
        }
    }
    void actionOrigin(){if(mode==Mode.ORIGIN)setOrigin=true;}
    void actionPlace(){if(mode==Mode.SCAN)place=true;}
    void save(){
        if(mode!=Mode.SCAN || space.dog==null || !space.quality()){ui.message("请完成多视角扫描并放置小狗");return;}
        if(validator!=null)return;
        validator=new Vision();validationStart=mapCamera;validationSince=SystemClock.elapsedRealtime();
        ui.message("地图已冻结。请横移约 20 厘米，验证新视角能找回小狗");
    }
    private void commit(){
        try{space.save(dir);String name=space.name;reset();ui.saved(name);}
        catch(Exception e){endValidation();ui.message("保存失败，可重试 · "+e.getMessage());}
    }
    private void endValidation(){
        if(validator!=null){validator.close();validator=null;}
        validationStart=null;
    }
    private void verifyScan(Frame frame,Camera cam,Rigid arCamera,Rigid mapAR,long ts){
        if(SystemClock.elapsedRealtime()-validationSince>25000){
            endValidation();ui.message("新视角复核超时，已恢复采样。补拍纹理丰富的区域后可重试");return;
        }
        if(ts-sampleTime>220000000L && validationStart!=null && mapCamera.distance(validationStart)>.18){
            sampleTime=ts;
            try(Vision.Shot shot=capture(frame,cam,arCamera)){
                if(shot!=null){
                    Rigid fix=validator.locate(shot,space,ts);
                    if(fix!=null && fix.times(arCamera).distance(mapAR.times(arCamera))<.15 && fix.angle(mapAR)<Math.toRadians(6)){
                        commit();return;
                    }
                }
            }
        }
        report("保存前复核 · 从新位置看向扫描区","横移至少 20 厘米并稳住手机，现实场景须与已建地图对齐\n连续验证 "+validator.confirmations+" / 3");
    }
    private void releaseCamera(){
        endValidation();
        if(placedDog!=null){placedDog.detach();placedDog=null;}
        if(origin!=null){origin.detach();origin=null;}
        if(session!=null){try{if(resumed)session.pause();}finally{session.close();session=null;resumed=false;}}
        if(vision!=null){vision.close();vision=null;}
    }
    void reset(){
        releaseCamera();mode=Mode.HOME;space=null;mapCamera=null;
        setOrigin=false;place=false;uiTime=0;
    }
    void suspend(){
        if(mode==Mode.HOME)return;
        suspended=true;
        if(mode==Mode.SCAN || mode==Mode.ORIGIN){
            endValidation();
            if(session!=null && resumed){session.pause();resumed=false;}
        }else{
            releaseCamera();mode=Mode.SEARCH;mapCamera=null;
        }
    }
    void resumeForeground(){
        if(mode==Mode.HOME)return;
        suspended=false;
        if(session==null){openCamera();return;}
        if(!resumed)try{session.resume();resumed=true;}catch(Exception e){fail(e);}
    }
    private void fail(Throwable e){
        Log.e("SpacePup","Tracking",e);reset();
        ui.failed("无法继续空间跟踪 · "+e.getClass().getSimpleName()+(e.getMessage()==null?"":" · "+e.getMessage()));
    }
    public void onDrawFrame(GL10 gl){
        glClearColor(.047f,.082f,.091f,1);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        try{
            if(mode==Mode.HOME || suspended)return;
            if(mode==Mode.SEARCH)record();
            if(session==null)return;
            Frame frame=session.update();Camera cam=frame.getCamera();long ts=frame.getTimestamp();
            if(ts==0)return;
            scene.background(frame);
            if(cam.getTrackingState()!=TrackingState.TRACKING || mode==Mode.SCAN && origin.getTrackingState()!=TrackingState.TRACKING){
                vision.resetConfirmation();
                report("视觉跟踪尚未就绪","缓慢移动手机，对准有纹理的物体 · "+cam.getTrackingFailureReason());return;
            }
            cam.getProjectionMatrix(projection,0,0.05f,120f);
            Rigid arCamera=from(cam.getPose());
            if(mode==Mode.ORIGIN){
                if(setOrigin){setOrigin=false;Rigid base=arCamera.leveled();
                    if(base==null)ui.message("请将镜头朝前，再设原点");else{origin=session.createAnchor(toPose(base));mode=Mode.SCAN;}
                }
                if(mode==Mode.ORIGIN){report("设定空间原点","选好起点，将手机镜头朝向空间正前方");return;}
            }
            Rigid mapAR=null;
            if(origin!=null && origin.getTrackingState()==TrackingState.TRACKING){
                Rigid arFromAnchor=from(origin.getPose());
                mapAR=mode==Mode.SCAN?arFromAnchor.inverse():space.dog.times(arFromAnchor.inverse());
            }
            if(mode==Mode.SCAN){
                if(place){place=false;placeDog(frame,mapAR,arCamera);}
                if(placedDog!=null && placedDog.getTrackingState()==TrackingState.TRACKING){
                    space.dog=mapAR.times(from(placedDog.getPose()));
                }
                mapCamera=mapAR.times(from(cam.getDisplayOrientedPose()));
                if(validator!=null){
                    scene.renderDog(projection,mapCamera,space.dog);
                    verifyScan(frame,cam,arCamera,mapAR,ts);return;
                }
                if(ts-sampleTime>320000000L){sampleTime=ts;
                    Vision.Shot shot=capture(frame,cam,mapAR.times(arCamera));if(shot!=null)vision.map(shot,space);}
                scene.renderDog(projection,mapCamera,space.dog);
                report(space.quality()?"地图质量已达标":"缓慢横移，换角度拍同一批物体",
                    String.format(Locale.CHINA,"特征 %d / 120　有效视角 %d / 5　移动 %.1f / 0.65 m\n%s",
                    space.points.size(),space.usefulViews,space.extent,space.dog==null?"对准已识别的地面放置小狗":"小狗已放置 · 鼻尖朝向放置时的手机"));
            }else if(mode==Mode.SEARCH){
                if(ts-sampleTime>210000000L){sampleTime=ts;
                    try(Vision.Shot shot=capture(frame,cam,arCamera)){
                        if(shot!=null){Rigid fix=vision.locate(shot,space,ts);
                            if(fix!=null){
                                anchorDog(fix);mapAR=fix;
                                mode=Mode.VISUAL;
                                vision.resetConfirmation();lastMapMatch=ts;
                                mapRefresh.reset(ts,space.nearMappedView(fix.point(arCamera.t)));
                            }
                        }
                    }
                }
                if(mode==Mode.SEARCH){report("正在找回「"+space.name+"」",
                    "对准扫描过的家具或墙角，慢慢移动\n匹配内点 "+vision.inliers+"　画面分区 "+vision.sectors+"　连续验证 "+vision.confirmations+" / 3");return;}
            }
            if(mode==Mode.VISUAL){
                mapAR=correctAgainstMap(frame,cam,arCamera,mapAR,ts);
                if(mapAR==null){
                    report("正在找回小狗的位置","对准放置小狗时看到的地面、墙角或家具。");return;
                }
                Rigid display=from(cam.getDisplayOrientedPose());
                mapCamera=mapAR.times(display);
                scene.renderDog(projection,mapCamera,space.dog);
                String status=mapRefresh.inside?"持续视觉定位 · 扫描区内":"视觉跟踪 · 扫描区外";
                String matchStatus=mapRefresh.inside && ts-lastMapMatch>12000000000L
                    ?"\n等待自动匹配已扫描特征":"";
                report(status,poseText()+matchStatus+(Double.isFinite(lastCorrection)?
                    String.format(Locale.CHINA,"\n上次视觉校正 %.2f m",lastCorrection):""));
                record();
            }
        }catch(Exception | LinkageError e){fail(e);}
    }
    private void anchorDog(Rigid mapFromAr){
        Anchor replacement=session.createAnchor(toPose(mapFromAr.inverse().times(space.dog)));
        if(origin!=null)origin.detach();
        origin=replacement;
    }
    private Rigid correctAgainstMap(Frame frame,Camera cam,Rigid arCamera,Rigid mapAR,long ts){
        boolean wasInside=mapRefresh.inside;
        boolean inside=mapAR!=null && space.nearMappedView(mapAR.point(arCamera.t),wasInside);
        MapRefresh.Work work=mapRefresh.next(ts,mapAR!=null,inside);
        if(wasInside!=mapRefresh.inside)vision.resetConfirmation();
        if(work==MapRefresh.Work.NONE)return mapAR;

        boolean lightweight=work==MapRefresh.Work.LIGHT;
        try(Vision.Shot shot=capture(frame,cam,arCamera,lightweight)){
            if(shot==null)return mapAR;
            if(lightweight){
                Rigid check=vision.lightFix(shot,space);
                if(check!=null){
                    if(mapAR==null || dogShift(mapAR,check)>.035 || mapAR.angle(check)>Math.toRadians(1.3)){
                        mapRefresh.startFull(ts);
                    }else lastMapMatch=ts;
                }
                vision.resetConfirmation();
                return mapAR;
            }
            Rigid fix=vision.locate(shot,space,ts,true);
            if(fix==null)return mapAR;
            vision.resetConfirmation();
            if(mapAR==null){
                anchorDog(fix);lastMapMatch=ts;
                mapRefresh.reset(ts,space.nearMappedView(fix.point(arCamera.t)));
                return fix;
            }
            double cameraShift=mapAR.times(arCamera).distance(fix.times(arCamera));
            double angle=mapAR.angle(fix);
            boolean largeShift=cameraShift>1.5 || angle>Math.toRadians(25);
            if(largeShift && (vision.inliers<40 || vision.sectors<6 || vision.reprojectionError>1.8))return mapAR;
            double shift=dogShift(mapAR,fix);
            lastMapMatch=ts;mapRefresh.finish(ts);
            if(shift>.02 || angle>Math.toRadians(0.8)){
                anchorDog(fix);lastCorrection=shift;mapAR=fix;
            }
            return mapAR;
        }
    }
    private double dogShift(Rigid before,Rigid after){
        double[] previous=before.inverse().point(space.dog.t);
        double[] corrected=after.inverse().point(space.dog.t);
        return Rigid.norm(Rigid.sub(previous,corrected));
    }
    private Vision.Shot capture(Frame frame,Camera cam,Rigid pose){
        return capture(frame,cam,pose,false);
    }
    private Vision.Shot capture(Frame frame,Camera cam,Rigid pose,boolean probe){
        try(Image img=frame.acquireCameraImage()){return vision.capture(img,cam.getImageIntrinsics(),pose,probe);}
        catch(NotYetAvailableException e){return null;}
    }
    private void placeDog(Frame frame,Rigid mapAR,Rigid arCamera){
        for(HitResult hit:frame.hitTest(width*0.5f,height*0.5f)){
            if(!(hit.getTrackable() instanceof Plane))continue;
            Plane plane=(Plane)hit.getTrackable();
            if(plane.getType()!=Plane.Type.HORIZONTAL_UPWARD_FACING || !plane.isPoseInPolygon(hit.getHitPose()))continue;
            double[] foot=mapAR.point(from(hit.getHitPose()).t),head=mapAR.point(arCamera.t);
            space.dog=Rigid.yaw(foot,Math.atan2(head[0]-foot[0],head[2]-foot[2]));
            Anchor replacement=session.createAnchor(toPose(mapAR.inverse().times(space.dog)));
            if(placedDog!=null)placedDog.detach();
            placedDog=replacement;return;
        }
        ui.message("尚未识别准星处的水平面。向侧面移动，再对准地面或桌面");
    }
    private String poseText(){
        if(mapCamera==null)return "";
        double[] t=mapCamera.t;
        double[] to=mapCamera.inverse().point(space.dog.t);
        double elevation=Math.toDegrees(Math.atan2(to[1],Math.hypot(to[0],to[2])));
        return String.format(Locale.US,"X %+.2f　Y %+.2f　Z %+.2f m\n小狗 %.1f m · %s · %s %.0f°",t[0],t[1],t[2],Rigid.norm(to),to[2]>0?"身后":Math.abs(to[0])<0.25?"前方":to[0]<0?"左前方":"右前方",elevation<0?"下方":"上方",Math.abs(elevation));
    }
    private void report(String title,String detail){
        long now=SystemClock.elapsedRealtime();if(now-uiTime<160)return;uiTime=now;
        ui.status(mode,title,detail,mode==Mode.SCAN && validator==null && space.quality() && space.dog!=null);
    }
    private void record(){
        long now=SystemClock.elapsedRealtime();if(now-traceTime<250)return;traceTime=now;
        if(now-lastChargeRead>2000){
            BatteryManager battery=(BatteryManager)context.getSystemService(Context.BATTERY_SERVICE);
            charge=battery.getLongProperty(BatteryManager.BATTERY_PROPERTY_CHARGE_COUNTER);lastChargeRead=now;
        }
        String pose="";
        if(mapCamera!=null){double[] p=mapCamera.t,q=mapCamera.q;
            pose=String.format(Locale.US,"%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f",p[0],p[1],p[2],q[0],q[1],q[2],q[3]);}
        else pose=",,,,,,";
        if(trace.size()>=14400)trace.removeFirst();
        trace.add(String.format(Locale.US,"%d,%s,%s,%s,%.3f,%s",now,mode,pose,session!=null,lastCorrection,charge>0?Long.toString(charge):""));
    }
    String csv(){return "elapsedRealtime_ms,mode,x_m,y_m,z_m,qx,qy,qz,qw,camera_open,last_visual_correction_m,charge_counter_uAh\n"+String.join("\n",trace)+"\n";}
}
