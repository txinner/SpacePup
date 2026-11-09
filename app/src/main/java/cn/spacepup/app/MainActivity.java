package cn.spacepup.app;

import android.Manifest;
import android.app.*;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.*;
import android.graphics.drawable.GradientDrawable;
import android.opengl.GLSurfaceView;
import android.os.*;
import android.text.InputFilter;
import android.view.*;
import android.widget.*;
import com.google.ar.core.ArCoreApk;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public final class MainActivity extends Activity implements Engine.Listener {
    private static final int PAPER=0xffecf2ed,INK=0xff102225,MINT=0xff98e5c3,SUB=0xffa9c2ba,PANEL=0xef1a302f;
    private final Handler handler=new Handler(Looper.getMainLooper());
    private GLSurfaceView surface;private Engine engine;
    private FrameLayout root,overlay;private TextView statusTitle,statusDetail;
    private LinearLayout actions;private Button save;private Engine.Mode drawn;
    private File maps;private Space pending;private boolean creating,installAsked,foreground;
    private Engine.Tracking pendingTracking;
    private boolean inSession;
    private String export="";
    private final Runnable tick=new Runnable(){public void run(){
        if(!foreground || !inSession)return;
        surface.requestRender();
        handler.postDelayed(this,engine.mode==Engine.Mode.INERTIAL?64:32);
    }};
    @Override public void onCreate(Bundle state){
        super.onCreate(state);getWindow().setStatusBarColor(INK);getWindow().setNavigationBarColor(INK);
        maps=new File(getFilesDir(),"spaces");if(!maps.isDirectory() && !maps.mkdirs()){Toast.makeText(this,"空间目录创建失败",Toast.LENGTH_LONG).show();}
        root=new FrameLayout(this);surface=new GLSurfaceView(this);surface.setEGLContextClientVersion(2);
        surface.setEGLConfigChooser(8,8,8,8,16,0);surface.setPreserveEGLContextOnPause(true);
        engine=new Engine(getApplicationContext(),new Motion(this),maps,this);
        surface.setRenderer(engine);surface.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        root.addView(surface);overlay=new FrameLayout(this);root.addView(overlay);setContentView(root);home();
    }
    private int dp(float n){return Math.round(n*getResources().getDisplayMetrics().density);}
    private GradientDrawable bg(int color,int radius){GradientDrawable d=new GradientDrawable();d.setColor(color);d.setCornerRadius(dp(radius));return d;}
    private TextView label(String s,int size,int color,boolean bold){
        TextView v=new TextView(this);v.setText(s);v.setTextSize(size);v.setTextColor(color);
        if(bold)v.setTypeface(Typeface.create("sans-serif-medium",Typeface.NORMAL));v.setIncludeFontPadding(false);return v;
    }
    private LinearLayout column(){LinearLayout l=new LinearLayout(this);l.setOrientation(LinearLayout.VERTICAL);return l;}
    private void add(LinearLayout layout,View v,int gap){
        LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(-1,-2);lp.topMargin=dp(gap);layout.addView(v,lp);
    }
    private Button button(String s,boolean primary,Runnable fn){
        Button b=new Button(this);b.setAllCaps(false);b.setText(s);b.setTextSize(15);b.setTextColor(primary?INK:PAPER);
        b.setMinHeight(dp(46));b.setBackground(bg(primary?MINT:0xff294641,12));b.setOnClickListener(v->fn.run());return b;
    }
    private void home(){
        handler.removeCallbacks(tick);
        inSession=false;drawn=Engine.Mode.HOME;getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        overlay.removeAllViews();overlay.setBackgroundColor(INK);ScrollView scroll=new ScrollView(this);overlay.addView(scroll);
        LinearLayout body=column();body.setPadding(dp(24),dp(34),dp(24),dp(28));scroll.addView(body);
        add(body,label("空间小狗",34,PAPER,true),0);
        add(body,label("扫描房间，记录小狗在现实中的位置。",16,SUB,false),10);
        add(body,button("创建空间",true,this::nameSpace),26);
        add(body,label("我的空间",21,PAPER,true),32);
        File[] list=maps.listFiles((dir,name)->name.endsWith(".pup"));
        if(list==null || list.length==0)add(body,label("还没有地图。创建空间后，从不同角度拍摄同一批物体。",15,SUB,false),15);
        else{
            Arrays.sort(list,Comparator.comparingLong(File::lastModified).reversed());
            for(File f:list){
                String name;try(DataInputStream in=new DataInputStream(new BufferedInputStream(new FileInputStream(f)))){
                    in.readInt();name=in.readUTF();
                }catch(IOException e){name="无法读取的空间";}
                final String title=name;
                Button item=button(title+"　›",false,()->select(f,title));item.setGravity(Gravity.START|Gravity.CENTER_VERTICAL);item.setPadding(dp(16),0,dp(12),0);add(body,item,10);
            }
        }
        if(!export.isEmpty())add(body,button("导出上一段定位记录",false,this::exportCsv),26);
    }
    private void nameSpace(){
        EditText field=new EditText(this);field.setSingleLine();field.setHint("例如：客厅");field.setSelectAllOnFocus(true);
        field.setFilters(new InputFilter[]{new InputFilter.LengthFilter(40)});
        LinearLayout wrap=column();wrap.setPadding(dp(24),dp(8),dp(24),0);wrap.addView(field);
        new AlertDialog.Builder(this).setTitle("为空间命名").setView(wrap).setNegativeButton("取消",null)
            .setPositiveButton("开始扫描",(d,w)->{
                String name=field.getText().toString().trim();if(name.isEmpty())name="我的空间";
                queue(new Space(name,UUID.randomUUID().toString()),true,Engine.Tracking.MANUAL_CORRECT);
            }).show();
    }
    private void select(File file,String name){
        Engine.Tracking[] choices={Engine.Tracking.AUTO_CORRECT,Engine.Tracking.MANUAL_CORRECT,Engine.Tracking.LIVE_CAMERA};
        String[] labels={"定位后关相机，需要时自动校正","定位后关相机，手动校正","实景查看小狗"};
        new AlertDialog.Builder(this).setTitle(name).setItems(labels,(d,index)->{
            try{queue(Space.load(file),false,choices[index]);}
            catch(Exception e){dialog("地图读取失败",e.getMessage());}
        }).setNegativeButton("取消",null).show();
    }
    private void queue(Space space,boolean create,Engine.Tracking tracking){
        pending=space;creating=create;pendingTracking=tracking;installAsked=false;begin();
    }
    private void begin(){
        if(pending==null || !foreground)return;
        if(checkSelfPermission(Manifest.permission.CAMERA)!=PackageManager.PERMISSION_GRANTED){
            requestPermissions(new String[]{Manifest.permission.CAMERA},71);return;
        }
        try{
            if(ArCoreApk.getInstance().requestInstall(this,!installAsked)==ArCoreApk.InstallStatus.INSTALL_REQUESTED){installAsked=true;return;}
            System.loadLibrary("opencv_java4");
        }catch(Exception | LinkageError e){pending=null;
            dialog("无法启动空间扫描","需要支持 ARCore 的 64 位 Android 手机，并安装可用的 Google Play Services for AR。\n"+e.getClass().getSimpleName());return;}
        Space target=pending;pending=null;boolean create=creating;Engine.Tracking tracking=pendingTracking;inSession=true;
        handler.removeCallbacks(tick);handler.post(tick);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        hud(create?Engine.Mode.ORIGIN:Engine.Mode.SEARCH);
        int rotation=getWindowManager().getDefaultDisplay().getRotation();
        event(()->engine.start(target,create,rotation,tracking));
    }
    @Override public void onRequestPermissionsResult(int code,String[] permissions,int[] grants){
        super.onRequestPermissionsResult(code,permissions,grants);
        if(code==71){if(grants.length>0 && grants[0]==PackageManager.PERMISSION_GRANTED)begin();
            else{pending=null;dialog("需要相机权限","创建空间和重新定位需要相机权限。可以在系统设置中授权。");}}
    }
    private void dialog(String title,String detail){
        if(isFinishing() || isDestroyed())return;
        new AlertDialog.Builder(this).setTitle(title).setMessage(detail).setPositiveButton("知道了",null).show();
    }
    private void hud(Engine.Mode m){
        drawn=m;overlay.removeAllViews();overlay.setBackgroundColor(0x00000000);
        LinearLayout top=column();top.setPadding(dp(16),dp(13),dp(16),dp(13));top.setBackground(bg(PANEL,16));
        FrameLayout.LayoutParams head=new FrameLayout.LayoutParams(-1,-2,Gravity.TOP);
        head.setMargins(dp(12),dp(12),dp(12),0);overlay.addView(top,head);
        statusTitle=label("正在启动…",19,MINT,true);statusDetail=label("",14,PAPER,false);
        add(top,statusTitle,0);add(top,statusDetail,6);
        if(m==Engine.Mode.SCAN || m==Engine.Mode.ORIGIN){
            TextView reticle=label("＋",28,PAPER,false);reticle.setGravity(Gravity.CENTER);
            reticle.setShadowLayer(3,0,1,0xff000000);overlay.addView(reticle,new FrameLayout.LayoutParams(dp(40),dp(40),Gravity.CENTER));
        }
        LinearLayout bottom=column();bottom.setPadding(dp(12),dp(12),dp(12),dp(12));bottom.setBackground(bg(PANEL,18));
        FrameLayout.LayoutParams foot=new FrameLayout.LayoutParams(-1,-2,Gravity.BOTTOM);foot.setMargins(dp(12),0,dp(12),dp(12));overlay.addView(bottom,foot);
        actions=new LinearLayout(this);actions.setOrientation(LinearLayout.HORIZONTAL);add(bottom,actions,0);save=null;
        if(m==Engine.Mode.ORIGIN)action("设为原点",true,engine::actionOrigin);
        if(m==Engine.Mode.SCAN){action("放置小狗",false,engine::actionPlace);save=action("保存空间",true,engine::save);}
        if(m==Engine.Mode.INERTIAL){action("视觉校正",true,()->engine.recalibrate(true));action("实景查看",false,()->engine.recalibrate(false));}
        if(m==Engine.Mode.SEARCH && engine.canReturnToImu)action("继续惯性跟踪",false,engine::returnToImu);
        if(m==Engine.Mode.VISUAL)action("关闭相机",true,engine::requestSleep);
        Button exit=button("返回空间列表",false,this::exitSession);add(bottom,exit,actions.getChildCount()>0?8:0);
    }
    private Button action(String title,boolean primary,Runnable run){
        Button b=button(title,primary,()->event(run));LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(0,-2,1);
        if(actions.getChildCount()>0)lp.leftMargin=dp(8);actions.addView(b,lp);return b;
    }
    private void event(Runnable r){surface.queueEvent(r);surface.requestRender();}
    private void exitSession(){
        if(drawn==Engine.Mode.SCAN || drawn==Engine.Mode.ORIGIN){
            new AlertDialog.Builder(this).setTitle("离开创建空间？").setMessage("本次尚未保存的扫描数据将被丢弃。")
                .setPositiveButton("离开",(d,w)->finishSession()).setNegativeButton("继续扫描",null).show();
        }else finishSession();
    }
    private void finishSession(){
        event(()->{String log=engine.trace.isEmpty()?"":engine.csv();engine.reset();
            runOnUiThread(()->{if(!log.isEmpty())export=log;home();});});
    }
    public void status(Engine.Mode mode,String title,String detail,boolean ready){
        runOnUiThread(()->{if(!inSession || engine.mode!=mode || !foreground)return;
            if(drawn!=mode)hud(mode);statusTitle.setText(title);statusDetail.setText(detail);
            if(save!=null){save.setEnabled(ready);save.setAlpha(ready?1f:.48f);}
        });
    }
    public void message(String text){runOnUiThread(()->Toast.makeText(this,text,Toast.LENGTH_LONG).show());}
    public void saved(String name){runOnUiThread(()->{home();message("已保存「"+name+"」");});}
    public void failed(String error){runOnUiThread(()->{home();dialog("空间跟踪已停止",error);});}
    @Override public void onBackPressed(){if(inSession)exitSession();else super.onBackPressed();}
    @Override protected void onResume(){
        super.onResume();foreground=true;surface.onResume();
        if(inSession)event(engine::resumeForeground);
        handler.removeCallbacks(tick);handler.post(tick);
        if(pending!=null && (installAsked || checkSelfPermission(Manifest.permission.CAMERA)==PackageManager.PERMISSION_GRANTED))begin();
    }
    @Override protected void onPause(){
        foreground=false;handler.removeCallbacks(tick);
        if(inSession){CountDownLatch done=new CountDownLatch(1);
            surface.queueEvent(()->{try{engine.suspend();}finally{done.countDown();}});surface.requestRender();
            try{done.await(1200,TimeUnit.MILLISECONDS);}catch(InterruptedException e){Thread.currentThread().interrupt();}
        }
        surface.onPause();super.onPause();
    }
    @Override protected void onDestroy(){
        handler.removeCallbacks(tick);
        if(inSession){
            CountDownLatch done=new CountDownLatch(1);
            surface.queueEvent(()->{try{engine.reset();}finally{done.countDown();}});
            try{done.await(1000,TimeUnit.MILLISECONDS);}catch(InterruptedException e){Thread.currentThread().interrupt();}
        }
        super.onDestroy();
    }
    private void exportCsv(){
        Intent intent=new Intent(Intent.ACTION_CREATE_DOCUMENT);intent.setType("text/csv");intent.putExtra(Intent.EXTRA_TITLE,"spacepup-track.csv");
        startActivityForResult(intent,81);
    }
    @Override protected void onActivityResult(int req,int result,Intent data){
        super.onActivityResult(req,result,data);
        if(req==81 && result==RESULT_OK && data!=null && data.getData()!=null){
            try(OutputStream o=getContentResolver().openOutputStream(data.getData())){
                if(o==null)throw new IOException("无法打开文件");o.write(export.getBytes(StandardCharsets.UTF_8));message("记录已导出");
            }catch(IOException e){message("导出失败 · "+e.getMessage());}
        }
    }
}
