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
    enum Mode { HOME, ORIGIN, SCAN, SEARCH, VISUAL, INERTIAL }
    interface Listener {void status(Mode mode,String title,String detail,boolean saveReady);void message(String text);void saved(String name);void failed(String error);}
    final Context context;final Listener ui;final Motion motion;final File dir;
    final Scene scene=new Scene();
    volatile Mode mode=Mode.HOME;
    private Session session;private boolean resumed,suspended;
    private Anchor origin;private Vision vision,validator;private Space space;
    private Rigid validationStart;
    private long validationSince;
    private int width=1,height=1,rotation;
    private Rigid mapCamera, stationary, latestSensor;
    private final float[] projection=new float[16];
    private long sampleTime,uiTime,previousFrame,stationarySince;
    private boolean place, setOrigin, closeAfterFix=true;
    private double lastCorrection=Double.NaN;
    private Rigid oldEstimate;
    private long searchStarted;
    volatile boolean canReturnToImu;
    private long traceTime;
    private long lastChargeRead=-10000,charge;
    final ArrayDeque<String> trace=new ArrayDeque<>();
    Engine(Context ctx,Motion motion,File dir,Listener ui){context=ctx;this.motion=motion;this.dir=dir;this.ui=ui;}
    static Rigid from(Pose p){return new Rigid(Rigid.doubles(p.getTranslation()),Rigid.doubles(p.getRotationQuaternion()));}
    static Pose toPose(Rigid p){return new Pose(Rigid.floats(p.t),Rigid.floats(p.q));}
    public void onSurfaceCreated(GL10 gl,EGLConfig config){
        scene.init();if(session!=null)session.setCameraTextureName(scene.texture);
    }
    public void onSurfaceChanged(GL10 gl,int w,int h){
        width=w;height=h;glViewport(0,0,w,h);if(session!=null)session.setDisplayGeometry(rotation,w,h);
    }
    void start(Space target,boolean creating,int displayRotation,boolean closeCamera){
        reset();suspended=false;trace.clear();traceTime=0;lastCorrection=Double.NaN;space=target;rotation=displayRotation;closeAfterFix=closeCamera;
        mode=creating?Mode.ORIGIN:Mode.SEARCH;motion.start();openCamera();
    }
    private void openCamera(){
        try{
            vision=new Vision();session=new Session(context);
            Config config=new Config(session);config.setFocusMode(Config.FocusMode.AUTO);
            config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE);
            config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
            config.setPlaneFindingMode(mode==Mode.ORIGIN || mode==Mode.SCAN?Config.PlaneFindingMode.HORIZONTAL:Config.PlaneFindingMode.DISABLED);
            session.configure(config);session.setCameraTextureName(scene.texture);
            session.setDisplayGeometry(rotation,width,height);session.resume();resumed=true;
            sampleTime=0;previousFrame=0;stationarySince=0;stationary=null;mapCamera=null;searchStarted=SystemClock.elapsedRealtime();
        }catch(Exception | LinkageError e){
            if(canReturnToImu){returnToImu();ui.message("相机暂时不可用，已恢复惯性估计");}
            else fail(e);
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
    void recalibrate(boolean thenClose){
        if(mode!=Mode.INERTIAL)return;
        oldEstimate=motion.display();canReturnToImu=oldEstimate!=null;mode=Mode.SEARCH;closeAfterFix=thenClose;openCamera();
    }
    void returnToImu(){
        if(mode!=Mode.SEARCH || !canReturnToImu)return;
        releaseCamera();mode=Mode.INERTIAL;oldEstimate=null;canReturnToImu=false;uiTime=0;
    }
    void requestSleep(){
        if(mode==Mode.VISUAL){closeAfterFix=true;stationary=null;stationarySince=0;}
    }
    private void releaseCamera(){
        endValidation();
        if(origin!=null){origin.detach();origin=null;}
        if(session!=null){try{if(resumed)session.pause();}finally{session.close();session=null;resumed=false;}}
        if(vision!=null){vision.close();vision=null;}
    }
    void reset(){
        releaseCamera();motion.stop();mode=Mode.HOME;space=null;mapCamera=null;oldEstimate=null;
        setOrigin=false;place=false;uiTime=0;canReturnToImu=false;
    }
    void suspend(){
        if(mode==Mode.HOME)return;
        suspended=true;
        motion.stop();
        if(mode==Mode.SCAN || mode==Mode.ORIGIN){
            endValidation();
            if(session!=null && resumed){session.pause();resumed=false;}
        }else{
            releaseCamera();mode=Mode.SEARCH;oldEstimate=null;canReturnToImu=false;
        }
    }
    void resumeForeground(){
        if(mode==Mode.HOME)return;
        suspended=false;
        motion.start();
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
            if(mode==Mode.SEARCH && canReturnToImu && SystemClock.elapsedRealtime()-searchStarted>18000){
                returnToImu();ui.message("尚未认出已扫描的空间，已关相机并继续惯性估计");
            }
            if(mode==Mode.SEARCH)record();
            if(mode==Mode.INERTIAL){
                mapCamera=motion.display();scene.renderDog(projection,mapCamera,space.dog);
                boolean caution=motion.risk()>0.6;
                report(caution?"惯性估计 · 建议校正":"惯性跟踪 · 相机已关闭",
                    poseText()+"\n"+String.format(Locale.US,"关相机 %.0f 秒%s",motion.elapsed(),motion.hasBarometer()?" · 气压辅助高度":""));
                record();return;
            }
            if(session==null)return;
            Frame frame=session.update();Camera cam=frame.getCamera();long ts=frame.getTimestamp();
            if(ts==0)return;
            scene.background(frame);
            if(cam.getTrackingState()!=TrackingState.TRACKING || origin!=null && origin.getTrackingState()!=TrackingState.TRACKING){
                if(vision!=null){vision.confirmations=0;vision.candidate=null;}
                stationary=null;
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
            Rigid mapAR=origin==null?null:from(origin.getPose()).inverse();
            if(mode==Mode.SCAN){
                if(place){place=false;placeDog(frame,mapAR,arCamera);}
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
                                origin=session.createAnchor(toPose(fix.inverse()));mapAR=fix;
                                mode=Mode.VISUAL;stationary=null;canReturnToImu=false;
                                if(oldEstimate!=null){Rigid estimateNow=motion.display();
                                    lastCorrection=fix.times(from(cam.getDisplayOrientedPose())).distance(estimateNow==null?oldEstimate:estimateNow);oldEstimate=null;}
                            }
                        }
                    }
                }
                if(mode==Mode.SEARCH){report("正在找回「"+space.name+"」",
                    "对准扫描过的家具或墙角，慢慢移动\n匹配内点 "+vision.inliers+"　画面分区 "+vision.sectors+"　连续验证 "+vision.confirmations+" / 3");return;}
            }
            if(mode==Mode.VISUAL && mapAR!=null){
                Rigid display=from(cam.getDisplayOrientedPose()),sensor=from(frame.getAndroidSensorPose());
                mapCamera=mapAR.times(display);latestSensor=mapAR.times(sensor);
                scene.renderDog(projection,mapCamera,space.dog);
                if(closeAfterFix && previousFrame!=ts){
                    if(stationary==null || latestSensor.distance(stationary)>0.018 || latestSensor.angle(stationary)>Math.toRadians(2)){
                        stationary=latestSensor;stationarySince=ts;
                    }else if(ts-stationarySince>600000000L){
                        long now=SystemClock.elapsedRealtimeNanos(),cameraNs=frame.getAndroidCameraTimestamp();
                        // SensorEvent uses elapsed realtime. Some camera clocks use another epoch.
                        long seedNs=Math.abs(now-cameraNs)<250000000L?cameraNs:now;
                        if(motion.seed(latestSensor,sensor.inverse().times(display),new double[3],seedNs)){
                            releaseCamera();mode=Mode.INERTIAL;uiTime=0;
                            report("惯性跟踪 · 相机已关闭",poseText());record();return;
                        }
                        if(ts-stationarySince>2500000000L){closeAfterFix=false;ui.message("缺少可同步的惯性数据，已保持视觉定位");}
                    }
                }
                report(closeAfterFix?"已定位 · 请稳住手机以关闭相机":"已定位 · 实景查看",
                    poseText()+(Double.isFinite(lastCorrection)?String.format(Locale.CHINA,"\n上次视觉校正 %.2f m",lastCorrection):""));
                record();
            }
            previousFrame=ts;
        }catch(Exception | LinkageError e){fail(e);}
    }
    private Vision.Shot capture(Frame frame,Camera cam,Rigid pose){
        try(Image img=frame.acquireCameraImage()){return vision.capture(img,cam.getImageIntrinsics(),pose);}
        catch(NotYetAvailableException e){return null;}
    }
    private void placeDog(Frame frame,Rigid mapAR,Rigid arCamera){
        for(HitResult hit:frame.hitTest(width*0.5f,height*0.5f)){
            if(!(hit.getTrackable() instanceof Plane))continue;
            Plane plane=(Plane)hit.getTrackable();
            if(plane.getType()!=Plane.Type.HORIZONTAL_UPWARD_FACING || !plane.isPoseInPolygon(hit.getHitPose()))continue;
            double[] foot=mapAR.point(from(hit.getHitPose()).t),head=mapAR.point(arCamera.t);
            space.dog=Rigid.yaw(foot,Math.atan2(head[0]-foot[0],head[2]-foot[2]));return;
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
