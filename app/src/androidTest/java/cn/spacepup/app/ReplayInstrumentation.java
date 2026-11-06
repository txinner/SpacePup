package cn.spacepup.app;

import android.app.Activity;
import android.app.Instrumentation;
import android.os.Bundle;
import android.util.Log;
import java.nio.file.Files;
import java.nio.file.Path;

/** Exercises the APK's own ARM OpenCV library on a connected Android device. */
public final class ReplayInstrumentation extends Instrumentation {
    @Override public void onCreate(Bundle arguments){super.onCreate(arguments);start();}
    @Override public void onStart(){
        Bundle result=new Bundle();
        try{
            System.loadLibrary("opencv_java4");
            Path fixture=Files.createTempDirectory(getTargetContext().getCacheDir().toPath(),"pup-replay-");
            int checks=Replay.run(fixture);
            result.putString("stream","SpacePup PASS · "+checks+" checks");finish(Activity.RESULT_OK,result);
        }catch(Throwable error){
            result.putString("stream",Log.getStackTraceString(error));finish(Activity.RESULT_CANCELED,result);
        }
    }
}
