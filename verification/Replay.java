package cn.spacepup.app;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/** Desktop fixture using the app's actual ORB, triangulation, storage, PnP and inertial code. */
public final class Replay {
    static final int W=640,H=480;
    static final float[] F={475,475},C={320,240};
    static int checks;
    static void require(boolean ok,String why){if(!ok)throw new AssertionError(why);checks++;}
    static int noise(int x,int y,int wall,int seed){
        int h=x*715127+y*23747+wall*739223+seed*127;
        h^=h>>>16;h*=0x45d9f3b;h^=h>>>16;return h&255;
    }
    // Ray cast three perpendicular surfaces with independent, fixed, multi-scale textures.
    static byte[] room(Rigid camera,int seed){
        byte[] out=new byte[W*H];double[] t=camera.t;
        double[] right=camera.rotate(new double[]{1,0,0}),up=camera.rotate(new double[]{0,1,0}),front=camera.rotate(new double[]{0,0,-1});
        for(int y=0;y<H;y++)for(int x=0;x<W;x++){
            double a=(x-C[0])/F[0],b=(C[1]-y)/F[1];
            double dx=front[0]+a*right[0]+b*up[0],dy=front[1]+a*right[1]+b*up[1],dz=front[2]+a*right[2]+b*up[2];
            double depth=(-3.4-t[2])/dz;int wall=0;
            double floor=(-.85-t[1])/dy,side=(1.3-t[0])/dx;
            if(floor>0 && floor<depth){depth=floor;wall=1;}
            if(side>0 && side<depth){depth=side;wall=2;}
            double u=wall==2?t[2]+depth*dz:t[0]+depth*dx;
            double v=wall==1?t[2]+depth*dz:t[1]+depth*dy;
            int hi=noise((int)Math.floor(u*18),(int)Math.floor(v*18),wall,seed);
            int lo=noise((int)Math.floor(u*6),(int)Math.floor(v*6),wall,seed);
            out[y*W+x]=(byte)(20+0.60*hi+0.26*lo);
        }
        return out;
    }
    static void vision(Path dir)throws Exception{
        Space map=new Space("回归测试","fixture");
        try(Vision mapper=new Vision()){
            for(int i=0;i<12;i++){
                Rigid view=Rigid.yaw(new double[]{i*.14,.04*Math.sin(i*.6),i*.008},.12*Math.sin(i*.4));
                mapper.map(mapper.extract(room(view,11),W,H,F,C,view),map);
            }
        }
        System.out.printf(Locale.ROOT,"Map: %d landmarks, %d useful views, extent %.2f m%n",map.points.size(),map.usefulViews,map.extent);
        require(map.quality(),"multi-view quality");map.dog=Rigid.yaw(new double[]{.7,-.85,-1.9},1.1);
        map.save(dir.toFile());Space loaded=Space.load(map.file(dir.toFile()));
        require(!loaded.views.isEmpty() && loaded.nearMappedView(new double[]{.55,0,0}),"scan coverage lost on reload");
        require(!loaded.nearMappedView(new double[]{0,-3,0}),"next floor classified as scan coverage");
        byte[] legacyBytes=Arrays.copyOf(Files.readAllBytes(map.file(dir.toFile()).toPath()),
            (int)map.file(dir.toFile()).length()-4-loaded.views.size()*12);
        java.nio.ByteBuffer legacy=java.nio.ByteBuffer.wrap(legacyBytes);
        legacy.putInt(0,0x50555031);
        java.util.zip.CRC32 crc=new java.util.zip.CRC32();
        crc.update(legacyBytes,0,legacyBytes.length-8);
        legacy.putLong(legacyBytes.length-8,crc.getValue());
        Path legacyFile=Files.createTempFile(dir,"legacy-",".pup");
        try{
            Files.write(legacyFile,legacyBytes);
            Space oldMap=Space.load(legacyFile.toFile());
            require(oldMap.views.isEmpty() && oldMap.nearMappedView(new double[]{.55,0,0})
                && oldMap.points.size()==loaded.points.size(),"legacy map incompatible");
        }finally{Files.deleteIfExists(legacyFile);}
        require(loaded.points.size()==map.points.size() && loaded.dog.angle(map.dog)<1e-6 && loaded.dog.distance(map.dog)<1e-6,"anchor reload");
        Rigid truth=Rigid.yaw(new double[]{1.8,.28,-.7},.64);
        try(Vision finder=new Vision()){
            for(int i=0;i<3;i++){
                Rigid physical=Rigid.yaw(new double[]{.54+i*.015,.015,-.02},.05);
                Rigid inNewSession=truth.inverse().times(physical);
                try(Vision.Shot shot=finder.extract(room(physical,11),W,H,F,C,inNewSession)){
                    Rigid fix=finder.locate(shot,loaded,2000000000L+i*240000000L);
                    System.out.printf(Locale.ROOT,"Frame %d: %d inliers, %d regions%n",i,finder.inliers,finder.sectors);
                    if(i<2)require(fix==null,"premature fix");
                    else{
                        require(fix!=null,"relocalization failed");
                        double err=fix.times(inNewSession).distance(physical),ang=Math.toDegrees(fix.angle(truth));
                        require(err<.08 && ang<3,"wrong recovered pose");
                        System.out.printf(Locale.ROOT,"Re-entry: %.4f m, %.3f degrees (synthetic)%n",err,ang);
                    }
                }
            }
            for(int i=0;i<4;i++){
                try(Vision.Shot shot=finder.extract(room(Rigid.ID,314),W,H,F,C,Rigid.ID)){
                    require(finder.locate(shot,loaded,4000000000L+i*240000000L)==null,"false fix in another room");
                }
            }
            for(int i=0;i<3;i++){
                Rigid oblique=Rigid.yaw(new double[]{-.15+i*.02,.21,-.19},-.09);
                byte[] changed=room(oblique,11);
                for(int j=0;j<changed.length;j++)changed[j]=(byte)(j%W<90?28:16+.78*(changed[j]&255));
                try(Vision.Shot shot=finder.extract(changed,W,H,F,C,truth.inverse().times(oblique))){
                    Rigid fix=finder.locate(shot,loaded,5000000000L+i*240000000L);
                    if(i==2)require(fix!=null && fix.times(truth.inverse().times(oblique)).distance(oblique)<.12,"oblique, dimmer, partly occluded re-entry");
                }
            }
            try(Vision.Shot blank=finder.extract(new byte[W*H],W,H,F,C,Rigid.ID)){
                require(finder.locate(blank,loaded,6000000000L)==null,"blank frame");
            }
            // Keep the same AR session but inject a drift after walking away and returning.
            finder.resetConfirmation();
            Rigid afterWalk=Rigid.yaw(new double[]{2.15,.36,-.87},.75);
            Rigid checkView=Rigid.yaw(new double[]{.54,.015,-.02},.05);
            Rigid arCheckView=afterWalk.inverse().times(checkView);
            try(Vision.Shot shot=finder.extract(room(checkView,11),W,H,F,C,arCheckView,true)){
                Rigid check=finder.lightFix(shot,loaded);
                require(shot.keys.length<=280 && check!=null
                    && check.times(arCheckView).distance(truth.times(arCheckView))>.20,
                    "lightweight match did not detect return drift");
            }
            for(int i=0;i<3;i++){
                Rigid physical=Rigid.yaw(new double[]{.54+i*.015,.015,-.02},.05);
                Rigid arCamera=afterWalk.inverse().times(physical);
                try(Vision.Shot shot=finder.extract(room(physical,11),W,H,F,C,arCamera)){
                    long timestamp=8000000000L+i*300000000L;
                    Rigid correction=finder.locate(shot,loaded,timestamp,true);
                    if(i<2)require(correction==null,"loop correction accepted before three frames");
                    if(i==0)require(finder.locate(shot,loaded,timestamp,true)==null,"duplicate frame counted");
                    if(i==2){
                        require(correction!=null,"return to placement view was not recognized");
                        double[] expected=physical.inverse().point(loaded.dog.t);
                        double before=Rigid.norm(Rigid.sub(truth.times(arCamera).inverse().point(loaded.dog.t),expected));
                        double after=Rigid.norm(Rigid.sub(correction.times(arCamera).inverse().point(loaded.dog.t),expected));
                        require(before>.20 && after<.08,"dog remained offset after loop correction");
                        System.out.printf(Locale.ROOT,"Loop dog alignment: %.3f m before, %.3f m after (synthetic)%n",before,after);
                    }
                }
            }
        }
        Path file=map.file(dir.toFile()).toPath();byte[] bytes=Files.readAllBytes(file);bytes[bytes.length-12]^=4;Files.write(file,bytes);
        try{Space.load(file.toFile());throw new AssertionError("damaged map accepted");}catch(java.io.IOException expected){checks++;}
        Files.delete(file);
    }
    static void motion(){
        DeadReckoner nav=new DeadReckoner();Rigid start=Rigid.yaw(new double[]{.8,1.7,-.4},0.0);
        nav.seed(start,Rigid.ID,new double[3],0,1013.25);nav.originHeight=start.t[1];
        long ns=0;
        for(int i=0;i<50;i++){ns+=20000000;nav.advance(ns,new double[3],false,0);}
        require(nav.mapFromSensor.distance(start)<1e-8,"stationary drift");
        for(int i=0;i<100;i++){ns+=20000000;nav.advance(ns,new double[]{i<50?1:-1,0,0},false,0);}
        for(int i=0;i<30;i++){ns+=20000000;nav.advance(ns,new double[3],false,0);}
        require(Math.abs(nav.mapFromSensor.t[0]-start.t[0]-1)<.04,"1m horizontal translation");
        double[] before=nav.mapFromSensor.t.clone();nav.rotate(Rigid.yaw(new double[3],Math.PI/2));
        require(Rigid.norm(Rigid.sub(nav.mapFromSensor.t,before))<1e-8,"rotation changes position");
        require(Math.abs(nav.mapFromSensor.angle(start)-Math.PI/2)<1e-8,"rotation axes");
        for(int i=0;i<200;i++)nav.pressure(1013.25*Math.pow(1-3.0/44330,1/.190295),.1);
        require(Math.abs(nav.mapFromSensor.t[1]-start.t[1]-3)<.01,"3m barometric rise");
        nav.advance(ns+500000000L,new double[3],false,0);require(nav.interrupted,"IMU gap unmarked");
        require(Rigid.triangulate(Rigid.ID,new double[]{0,0,-1},Rigid.ID,new double[]{0,0,-1})==null,"zero baseline");
        require(start.times(start.inverse()).distance(Rigid.ID)<1e-10,"transform inverse");
    }
    static void stairs(){
        DeadReckoner nav=new DeadReckoner();nav.seed(Rigid.ID,Rigid.ID,new double[3],0,Double.NaN);
        double[] bias={.06,-.035,.09};
        nav.calibrateAtRest(Rigid.add(bias,new double[]{0,9.80665,0}),true);
        long ns=0;
        for(int i=0;i<35;i++){ns+=20000000;nav.advance(ns,Rigid.add(bias,new double[]{0,9.80665,0}),true,0);}
        for(int leg=0;leg<2;leg++){
            for(int i=0;i<200;i++){
                ns+=20000000;
                double phase=(i+200*leg)*.022,tilt=.55*Math.sin(phase);
                Rigid sensor=Rigid.yaw(new double[3],phase).times(new Rigid(new double[3],new double[]{Math.sin(tilt/2),0,0,Math.cos(tilt/2)}));
                nav.rotate(sensor);double sign=(i<100?1:-1)*(leg==0?1:-1);
                double[] world={2*sign,9.80665-.75*sign,0};
                nav.advance(ns,Rigid.add(bias,sensor.inverse().rotate(world)),true,1.1);
            }
            if(leg==0)require(Rigid.norm(Rigid.sub(nav.mapFromSensor.t,new double[]{8,-3,0}))<.10,"midpoint height/translation");
        }
        require(Rigid.norm(nav.mapFromSensor.t)<.10,"return after rotating on 3D path");
        System.out.printf(Locale.ROOT,"Ideal IMU loop endpoint %.5f m (noise-free signals)%n",Rigid.norm(nav.mapFromSensor.t));
    }
    static void driftGuard(){
        DeadReckoner nav=new DeadReckoner();
        nav.seed(Rigid.ID,Rigid.ID,new double[3],0,Double.NaN);
        long ns=0;
        for(int i=0;i<300;i++){
            ns+=20000000;
            nav.rotate(Rigid.yaw(new double[3],i*.016));
            nav.advance(ns,new double[]{.24,0,0},false,.8);
        }
        require(nav.mapFromSensor.distance(Rigid.ID)<1e-8,"rotation with residual acceleration adds translation");
        require(nav.positionUsable(),"rotation prematurely invalidates position");
        for(int i=0;i<250;i++){
            ns+=20000000;
            nav.advance(ns,new double[]{i<30?1:0,0,0},false,0);
        }
        require(nav.mapFromSensor.distance(Rigid.ID)>2,"constant velocity mistaken for rest");
        require(!nav.positionUsable(),"growing displacement uncertainty stays visible");
        nav.seed(Rigid.ID,Rigid.ID,new double[3],ns,Double.NaN);
        require(nav.positionUsable(),"visual reseed does not reset uncertainty");
        for(int i=0;i<1550;i++){ns+=20000000;nav.advance(ns,new double[3],false,0);}
        require(!nav.positionUsable(),"inertial window did not expire");
    }
    static void refreshSchedule(){
        long second=1_000_000_000L;
        MapRefresh refresh=new MapRefresh();
        refresh.reset(second,true);
        require(refresh.next(5*second,true,true)==MapRefresh.Work.NONE,"early in-map check");
        require(refresh.next(6*second,true,true)==MapRefresh.Work.LIGHT,"missing first light check");
        require(refresh.next(11*second,true,true)==MapRefresh.Work.LIGHT,"missing periodic light check");
        require(refresh.next(20*second,true,false)==MapRefresh.Work.NONE,"map matched outside");
        require(refresh.next(35*second,true,false)==MapRefresh.Work.NONE,"repeated map matching outside");
        require(refresh.next(36*second,true,true)==MapRefresh.Work.FULL,"re-entry did not start full match");
        require(refresh.next(36*second,true,true)==MapRefresh.Work.NONE,"same camera frame matched twice");
        require(refresh.next(36*second+300_000_000L,true,true)==MapRefresh.Work.FULL,"re-entry burst missing next sample");
        refresh.finish(37*second);
        require(refresh.next(38*second,true,true)==MapRefresh.Work.NONE,"re-entry repeated full match");
        require(refresh.next(42*second,true,true)==MapRefresh.Work.LIGHT,"periodic check not restored");
        refresh.startFull(42*second);
        require(refresh.next(42*second+100_000_000L,true,true)==MapRefresh.Work.FULL,"drift did not trigger full match");
        require(refresh.next(45*second,true,true)==MapRefresh.Work.NONE,"unsuccessful full match did not stop");
        require(refresh.next(47*second,true,true)==MapRefresh.Work.LIGHT,"unsuccessful correction did not retry lightly");
    }
    public static void main(String[] args)throws Exception{
        System.load(args[0]);run(Files.createTempDirectory("spacepup-replay-"));
    }
    static int run(Path dir)throws Exception{
        checks=0;
        try{vision(dir);motion();stairs();driftGuard();refreshSchedule();System.out.println("PASS · "+checks+" checks");return checks;}
        finally{Files.deleteIfExists(dir.resolve("fixture.pup"));Files.deleteIfExists(dir);}
    }
}
