package cn.spacepup.app;

import android.content.Context;
import android.hardware.*;
import android.os.Handler;
import android.os.HandlerThread;
import java.util.ArrayDeque;

/** Registered only while a space session is in the foreground. Sensor poses use Android native axes. */
final class Motion implements SensorEventListener {
    private final SensorManager manager;
    private final Sensor rotation, acceleration,gyro,pressure;
    private HandlerThread thread;
    private final ArrayDeque<Sample> orientations=new ArrayDeque<>();
    private final ArrayDeque<double[]> recentAccelerations=new ArrayDeque<>();
    private final DeadReckoner nav=new DeadReckoner();
    private double pressureValue=Double.NaN,gyroRate;
    private long pressureNs,accelNs;
    private Rigid sensorFromDisplay=Rigid.ID;
    private static final class Sample {final long t;final Rigid r;Sample(long t,Rigid r){this.t=t;this.r=r;}}
    Motion(Context context){
        manager=(SensorManager)context.getSystemService(Context.SENSOR_SERVICE);
        Sensor rot=manager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        rotation=rot!=null?rot:manager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        Sensor acc=manager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        acceleration=acc!=null?acc:manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyro=manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);pressure=manager.getDefaultSensor(Sensor.TYPE_PRESSURE);
    }
    synchronized void start(){
        if(thread!=null)return;
        thread=new HandlerThread("SpacePup-IMU");thread.start();Handler h=new Handler(thread.getLooper());
        for(Sensor sensor:new Sensor[]{rotation,acceleration,gyro})if(sensor!=null)manager.registerListener(this,sensor,20000,h);
        if(pressure!=null)manager.registerListener(this,pressure,100000,h);
    }
    synchronized void stop(){
        manager.unregisterListener(this);if(thread!=null)thread.quitSafely();thread=null;nav.live=false;orientations.clear();recentAccelerations.clear();pressureValue=Double.NaN;pressureNs=0;accelNs=0;
    }
    synchronized boolean seed(Rigid mapSensor,Rigid sensorDisplay,double[] velocity,long timestamp){
        if(rotation==null || acceleration==null || gyro==null || orientations.isEmpty() || accelNs==0)return false;
        Sample best=orientations.peekLast();
        for(Sample s:orientations)if(Math.abs(s.t-timestamp)<Math.abs(best.t-timestamp))best=s;
        // Reject a stale sensor sample rather than handing off in an unrelated orientation.
        if(Math.abs(best.t-timestamp)>350000000L || Math.abs(accelNs-timestamp)>350000000L)return false;
        nav.seed(mapSensor,best.r,velocity,timestamp,pressureValue);
        double[] mean=new double[3];int count=0;
        for(double[] sample:recentAccelerations){
            if(sample[3]<timestamp-450000000L || sample[3]>timestamp+30000000L)continue;
            for(int i=0;i<3;i++)mean[i]+=sample[i];count++;
        }
        if(count>=5)nav.calibrateAtRest(Rigid.scale(mean,1.0/count),acceleration.getType()==Sensor.TYPE_ACCELEROMETER);
        nav.rotate(orientations.peekLast().r);sensorFromDisplay=sensorDisplay;return true;
    }
    synchronized Rigid display(){return nav.live?nav.mapFromSensor.times(sensorFromDisplay):null;}
    synchronized double risk(){return nav.risk();}
    synchronized double elapsed(){return nav.elapsed;}
    synchronized boolean hasBarometer(){return Double.isFinite(pressureValue);}
    public synchronized void onSensorChanged(SensorEvent e){
        if(thread==null)return;
        if(e.sensor==rotation){
            float[] wxyz=new float[4];SensorManager.getQuaternionFromVector(wxyz,e.values);
            Rigid r=new Rigid(new double[3],new double[]{wxyz[1],wxyz[2],wxyz[3],wxyz[0]});
            orientations.addLast(new Sample(e.timestamp,r));while(orientations.size()>32)orientations.removeFirst();
            nav.rotate(r);
        }else if(e.sensor==gyro){gyroRate=Math.sqrt(e.values[0]*e.values[0]+e.values[1]*e.values[1]+e.values[2]*e.values[2]);}
        else if(e.sensor==acceleration){
            accelNs=e.timestamp;double[] sample={e.values[0],e.values[1],e.values[2],e.timestamp};
            recentAccelerations.addLast(sample);while(recentAccelerations.size()>40)recentAccelerations.removeFirst();
            nav.advance(e.timestamp,sample,e.sensor.getType()==Sensor.TYPE_ACCELEROMETER,gyroRate);
        }
        else if(e.sensor==pressure){
            pressureValue=e.values[0];if(pressureNs!=0)nav.pressure(pressureValue,(e.timestamp-pressureNs)*1e-9);
            pressureNs=e.timestamp;
        }
    }
    public void onAccuracyChanged(Sensor s,int accuracy){}
}
