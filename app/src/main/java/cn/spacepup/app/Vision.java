package cn.spacepup.app;

import android.media.Image;
import com.google.ar.core.CameraIntrinsics;
import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.calib3d.Calib3d;
import java.nio.ByteBuffer;
import java.util.*;

/** CPU image coordinates always use the physical, unrotated camera and its CPU intrinsics. */
final class Vision implements AutoCloseable {
    final ORB orb=ORB.create(1000);
    final ORB probeOrb=ORB.create(280);
    final BFMatcher matcher=BFMatcher.create(Core.NORM_HAMMING,false);
    final ArrayDeque<Shot> history=new ArrayDeque<>();
    private final Mat gray=new Mat(), noMask=new Mat();
    private final MatOfKeyPoint detectedKeys=new MatOfKeyPoint();
    private byte[] luminance=new byte[0];
    Mat mapDescriptors;
    int inliers, sectors;
    double reprojectionError;
    long lastGoodTime;
    Rigid candidate;
    int confirmations;

    static final class Shot implements AutoCloseable {
        final Rigid pose; final KeyPoint[] keys; final Mat desc;
        final double fx,fy,cx,cy;
        final int width,height;
        Shot(Rigid pose,KeyPoint[] keys,Mat descriptors,float[] f,float[] c,int w,int h) {
            this.pose=pose;this.keys=keys;desc=descriptors;width=w;height=h;
            fx=f[0];fy=f[1];cx=c[0];cy=c[1];
        }
        double[] ray(int i){Point p=keys[i].pt;return new double[]{(p.x-cx)/fx,-(p.y-cy)/fy,-1};}
        public void close(){desc.release();}
    }
    Shot capture(Image image,CameraIntrinsics intr,Rigid pose){
        return capture(image,intr,pose,false);
    }
    Shot capture(Image image,CameraIntrinsics intr,Rigid pose,boolean probe){
        int w=image.getWidth(),h=image.getHeight(); Image.Plane y=image.getPlanes()[0];
        int stride=y.getRowStride(),step=y.getPixelStride(); ByteBuffer data=y.getBuffer().duplicate();
        if(luminance.length!=w*h)luminance=new byte[w*h];
        if(step==1 && stride==w){
            data.get(luminance);
        }else{
            int offset=data.position();
            for(int row=0;row<h;row++){
                if(step==1){data.position(offset+row*stride);data.get(luminance,row*w,w);}
                else for(int col=0;col<w;col++)luminance[row*w+col]=data.get(offset+row*stride+col*step);
            }
        }
        return extract(luminance,w,h,intr.getFocalLength(),intr.getPrincipalPoint(),pose,probe);
    }
    Shot extract(byte[] pixels,int w,int h,float[] focal,float[] center,Rigid pose){
        return extract(pixels,w,h,focal,center,pose,false);
    }
    Shot extract(byte[] pixels,int w,int h,float[] focal,float[] center,Rigid pose,boolean lightweight){
        ORB detector=lightweight?probeOrb:orb;
        gray.create(h,w,CvType.CV_8U);
        Mat desc=new Mat();
        try{
            gray.put(0,0,pixels);
            detector.detectAndCompute(gray,noMask,detectedKeys,desc);
            return new Shot(pose,detectedKeys.toArray(),desc,focal,center,w,h);
        }
        catch(RuntimeException e){desc.release();throw e;}
    }
    private List<DMatch> matches(Mat query,Mat train){
        ArrayList<DMatch> result=new ArrayList<>();
        if(query.rows()==0 || train.rows()<2)return result;
        List<MatOfDMatch> pairs=new ArrayList<>();
        try {
            matcher.knnMatch(query,train,pairs,2);
            for(MatOfDMatch pair:pairs){DMatch[] m=pair.toArray();
                if(m.length==2 && m[0].distance<62 && m[0].distance<0.77*m[1].distance)result.add(m[0]);}
        }finally{for(MatOfDMatch p:pairs)p.release();}
        result.sort(Comparator.comparingDouble(x->x.distance));
        HashSet<Integer> used=new HashSet<>();
        result.removeIf(x->!used.add(x.trainIdx));
        return result;
    }
    void map(Shot s,Space space){
        Shot last=history.peekLast();
        if(s.keys.length<65 || last!=null && s.pose.distance(last.pose)<0.12 && s.pose.angle(last.pose)<0.16){s.close();return;}
        int before=space.points.size();
        int paired=0;
        Iterator<Shot> prior=history.descendingIterator();
        while(prior.hasNext() && paired<4){
            Shot other=prior.next(); double baseline=s.pose.distance(other.pose);
            if(baseline<0.15 || baseline>2.4 || s.pose.angle(other.pose)>1.25) continue;
            paired++;
            for(DMatch m:matches(s.desc,other.desc)){
                double[] p=Rigid.triangulate(s.pose,s.ray(m.queryIdx),other.pose,other.ray(m.trainIdx));
                if(p!=null){byte[] d=new byte[32];s.desc.get(m.queryIdx,0,d);space.add(p,d);}
            }
        }
        if(space.points.size()-before>=8)space.usefulViews++;
        space.extent=Math.max(space.extent,Rigid.norm(s.pose.t));
        space.recordView(s.pose);
        space.frames++;history.addLast(s);if(history.size()>14)history.removeFirst().close();
    }
    /** Stable transform from this AR session to the saved map, verified across distinct frames. */
    Rigid locate(Shot s,Space space,long time){
        return locate(s,space,time,false);
    }
    Rigid locate(Shot s,Space space,long time,boolean correction){
        Rigid pose=solve(s,space,matches(s.desc,descriptors(space)),240);
        if(pose==null || correction && (inliers<24 || sectors<5 || reprojectionError>2.2)){
            resetConfirmation();return null;
        }
        Rigid mapFromAR=pose.times(s.pose.inverse());
        // Both frames have gravity-aligned Y axes. Reject upright/ceiling aliases.
        if(mapFromAR.rotate(new double[]{0,1,0})[1]<0.97){resetConfirmation();return null;}
        if(candidate!=null && time<=lastGoodTime)return null;
        boolean agrees=candidate!=null && time-lastGoodTime<1500000000L
            && Rigid.norm(Rigid.sub(candidate.point(s.pose.t),pose.t))<(correction?0.09:0.13)
            && candidate.angle(mapFromAR)<Math.toRadians(correction?4:6);
        confirmations=agrees?Math.min(confirmations+1,3):1;
        candidate=agrees?candidate.blend(mapFromAR,1.0/confirmations):mapFromAR;
        lastGoodTime=time;
        return confirmations>=3?candidate:null;
    }
    Rigid lightFix(Shot shot,Space space){
        Rigid pose=solve(shot,space,matches(shot.desc,descriptors(space)),80);
        if(pose==null || inliers<18 || sectors<5 || reprojectionError>2.2)return null;
        Rigid mapFromAr=pose.times(shot.pose.inverse());
        return mapFromAr.rotate(new double[]{0,1,0})[1]>=.97?mapFromAr:null;
    }
    private Mat descriptors(Space space){
        if(mapDescriptors==null){
            mapDescriptors=new Mat(space.points.size(),32,CvType.CV_8U);
            byte[] bytes=new byte[32*space.points.size()];
            for(int i=0;i<space.points.size();i++)System.arraycopy(space.points.get(i).descriptor,0,bytes,32*i,32);
            mapDescriptors.put(0,0,bytes);
        }
        return mapDescriptors;
    }
    void resetConfirmation(){confirmations=0;candidate=null;}
    private Rigid solve(Shot s,Space space,List<DMatch> matches,int iterations){
        inliers=0;sectors=0;reprojectionError=Double.POSITIVE_INFINITY;
        if(matches.size()<14)return null;
        Point3[] world=new Point3[matches.size()];Point[] pixels=new Point[matches.size()];
        for(int i=0;i<matches.size();i++){
            DMatch m=matches.get(i);double[] p=space.points.get(m.trainIdx).p;
            world[i]=new Point3(p[0],p[1],p[2]);pixels[i]=s.keys[m.queryIdx].pt;
        }
        MatOfPoint3f obj=new MatOfPoint3f(world);MatOfPoint2f uv=new MatOfPoint2f(pixels);
        Mat k=Mat.eye(3,3,CvType.CV_64F),r=new Mat(),t=new Mat(),indices=new Mat();
        MatOfDouble distortion=new MatOfDouble();
        try{
            k.put(0,0,s.fx,0,s.cx,0,s.fy,s.cy,0,0,1);
            if(!Calib3d.solvePnPRansac(obj,uv,k,distortion,r,t,false,iterations,3.5f,0.995,indices,Calib3d.SOLVEPNP_EPNP))return null;
            inliers=(int)indices.total();
            if(inliers<13 || inliers<matches.size()*0.35)return null;
            int[] ids=new int[inliers];indices.get(0,0,ids);
            boolean[] grid=new boolean[12];Point3[] goodWorld=new Point3[inliers];Point[] goodUv=new Point[inliers];
            for(int j=0;j<ids.length;j++){int i=ids[j];Point px=pixels[i];
                int col=Math.min(3,(int)(px.x*4/s.width)),row=Math.min(2,(int)(px.y*3/s.height));
                grid[4*row+col]=true;goodWorld[j]=world[i];goodUv[j]=px;}
            for(boolean hit:grid)if(hit)sectors++;
            if(sectors<4)return null;
            obj.fromArray(goodWorld);uv.fromArray(goodUv);
            Calib3d.solvePnPRefineLM(obj,uv,k,distortion,r,t);
            double[] axis=new double[3],trans=new double[3];r.get(0,0,axis);t.get(0,0,trans);
            double angle=Rigid.norm(axis),a=Math.sin(angle/2)/Math.max(angle,1e-15);
            Rigid cvFromMap=new Rigid(trans,new double[]{axis[0]*a,axis[1]*a,axis[2]*a,Math.cos(angle/2)});
            double error=0;
            for(int j=0;j<inliers;j++){
                Point3 p=goodWorld[j];double[] v=cvFromMap.point(new double[]{p.x,p.y,p.z});
                if(v[2]<0.1 || v[2]>16)return null;
                error+=Math.pow(s.fx*v[0]/v[2]+s.cx-goodUv[j].x,2)+Math.pow(s.fy*v[1]/v[2]+s.cy-goodUv[j].y,2);
            }
            reprojectionError=Math.sqrt(error/inliers);
            if(error/inliers>7.0)return null;
            return cvFromMap.inverse().times(new Rigid(new double[3],new double[]{1,0,0,0}));
        }catch(CvException | IllegalArgumentException degenerate){
            return null;
        }finally{obj.release();uv.release();k.release();r.release();t.release();indices.release();distortion.release();}
    }
    public void close(){
        for(Shot shot:history)shot.close();
        history.clear();
        if(mapDescriptors!=null){mapDescriptors.release();mapDescriptors=null;}
        gray.release();noMask.release();detectedKeys.release();luminance=new byte[0];
    }
}
