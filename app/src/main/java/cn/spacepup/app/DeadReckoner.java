package cn.spacepup.app;

/** Short-horizon inertial propagation in the saved map's gravity-aligned frame (+Y up). */
final class DeadReckoner {
    private static final double REST_CONFIRM_SECONDS = 0.36;
    private static final double REST_ACCELERATION = 0.21;
    private static final double START_ACCELERATION = 0.32;
    Rigid mapFromSensor, mapFromInertial;
    double[] velocity=new double[3],previousA=new double[3],deviceBias=new double[3];
    long firstNs,lastNs; double stillSeconds,elapsed, movingTime,path;
    double basePressure=Double.NaN, filteredHeight;
    boolean interrupted, live;
    void seed(Rigid sensorPose,Rigid inertialFromSensor,double[] v,long ns,double pressure){
        mapFromSensor=sensorPose;mapFromInertial=sensorPose.times(inertialFromSensor.inverse());
        velocity=v.clone();previousA=new double[3];deviceBias=new double[3];
        firstNs=ns;lastNs=0;stillSeconds=Rigid.norm(v)<0.01?REST_CONFIRM_SECONDS:0;elapsed=0;movingTime=0;path=0;interrupted=false;live=true;
        basePressure=pressure;originHeight=sensorPose.t[1];filteredHeight=originHeight;
    }
    void rotate(Rigid inertialFromSensor){
        if(live)mapFromSensor=new Rigid(mapFromSensor.t,Rigid.mul(mapFromInertial.q,inertialFromSensor.q));
    }
    void calibrateAtRest(double[] deviceMean,boolean includesGravity){
        double[] gravity=includesGravity?mapFromSensor.inverse().rotate(new double[]{0,9.80665,0}):new double[3];
        double[] offset=Rigid.sub(deviceMean,gravity);
        if(Rigid.norm(offset)<0.55)deviceBias=offset;
    }
    void advance(long ns,double[] deviceA,boolean includesGravity,double gyroRate){
        if(!live)return;
        elapsed=Math.max(elapsed,(ns-firstNs)*1e-9);
        double dt=lastNs==0?0:(ns-lastNs)*1e-9;
        if(dt<0)return;
        lastNs=ns;
        double[] a=mapFromSensor.rotate(Rigid.sub(deviceA,deviceBias));
        if(includesGravity)a[1]-=9.80665;
        if(dt<=0 || dt>0.22){if(dt>0.22){interrupted=true;velocity=new double[3];}return;}
        double acceleration=Rigid.norm(a);
        boolean wasStill=stillSeconds>=REST_CONFIRM_SECONDS;
        if(wasStill && acceleration<START_ACCELERATION
            || acceleration<REST_ACCELERATION && gyroRate<0.06 && Rigid.norm(velocity)<0.30){
            stillSeconds=Math.min(REST_CONFIRM_SECONDS,stillSeconds+dt);
        }else stillSeconds=0;
        if(stillSeconds>=REST_CONFIRM_SECONDS){
            velocity=new double[3];previousA=new double[3];
            if(gyroRate<0.06 && acceleration<REST_ACCELERATION){
                double k=1-Math.exp(-dt/3);
                deviceBias=Rigid.add(deviceBias,Rigid.scale(mapFromSensor.inverse().rotate(a),k));
            }
            return;
        }
        movingTime+=dt;
        double[] mean=Rigid.scale(Rigid.add(a,previousA),0.5);
        double[] change=Rigid.add(Rigid.scale(velocity,dt),Rigid.scale(mean,0.5*dt*dt));
        velocity=Rigid.add(velocity,Rigid.scale(mean,dt));previousA=a;path+=Rigid.norm(change);
        mapFromSensor=new Rigid(Rigid.add(mapFromSensor.t,change),mapFromSensor.q);
    }
    double originHeight;
    void pressure(double pressure,double dt){
        if(!live || dt<=0 || !Double.isFinite(pressure) || pressure<200 || pressure>1200 || !Double.isFinite(basePressure))return;
        double h=originHeight+44330*(1-Math.pow(pressure/basePressure,0.190295));
        filteredHeight+=(h-filteredHeight)*(1-Math.exp(-Math.min(dt,1.0)/1.4));
        double[] p=mapFromSensor.t.clone();double residual=filteredHeight-p[1];
        p[1]+=residual*(1-Math.exp(-Math.min(dt,1.0)/1.1));
        if(stillSeconds<REST_CONFIRM_SECONDS)velocity[1]+=Math.max(-3,Math.min(3,residual))*Math.min(dt,.5)*0.16;
        mapFromSensor=new Rigid(p,mapFromSensor.q);
    }
    // A caution indicator only, not a statistically calibrated error bound.
    double risk(){return interrupted?Double.POSITIVE_INFINITY:0.05+0.025*movingTime+0.012*movingTime*movingTime+0.018*path;}
    boolean positionUsable(){return live && risk()<0.25 && elapsed<30;}
}
