package cn.spacepup.app;

import java.util.Arrays;

/** A transform from local coordinates to parent coordinates. Quaternions are x,y,z,w. */
final class Rigid {
    final double[] t, q;
    static final Rigid ID = new Rigid(new double[3], new double[]{0,0,0,1});
    Rigid(double[] t, double[] q) {
        this.t = t.clone();
        double length = Math.sqrt(dot(q,q));
        if (!(length > 1e-8) || !Double.isFinite(length)) throw new IllegalArgumentException("无效旋转");
        this.q = scale(q, 1/length);
        for (double x : t) if (!Double.isFinite(x)) throw new IllegalArgumentException("无效位置");
    }
    static double[] add(double[] a,double[] b) { return new double[]{a[0]+b[0],a[1]+b[1],a[2]+b[2]}; }
    static double[] sub(double[] a,double[] b) { return add(a,scale(b,-1)); }
    static double[] scale(double[] a,double k) { double[] b=a.clone(); for(int i=0;i<b.length;i++) b[i]*=k; return b; }
    static double dot(double[] a,double[] b) { double s=0; for(int i=0;i<a.length;i++) s+=a[i]*b[i]; return s; }
    static double norm(double[] a) { return Math.sqrt(dot(a,a)); }
    static double[] unit(double[] a) { return scale(a,1/Math.max(1e-12,norm(a))); }
    static double[] cross(double[] a,double[] b) {return new double[]{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
    static double[] mul(double[] a,double[] b) {
        return new double[]{a[3]*b[0]+b[3]*a[0]+a[1]*b[2]-a[2]*b[1],
            a[3]*b[1]+b[3]*a[1]+a[2]*b[0]-a[0]*b[2],
            a[3]*b[2]+b[3]*a[2]+a[0]*b[1]-a[1]*b[0],
            a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]};
    }
    double[] rotate(double[] p) {
        double[] v=cross(q,p);
        return add(p,scale(add(scale(v,q[3]),cross(q,v)),2));
    }
    double[] point(double[] p) {return add(t,rotate(p));}
    Rigid times(Rigid b) { return new Rigid(point(b.t),mul(q,b.q)); }
    Rigid inverse() {
        Rigid r=new Rigid(new double[3],new double[]{-q[0],-q[1],-q[2],q[3]});
        return new Rigid(r.rotate(scale(t,-1)),r.q);
    }
    double distance(Rigid b) {return norm(sub(t,b.t));}
    double angle(Rigid b) {return 2*Math.acos(Math.min(1,Math.abs(dot(q,b.q))));}
    static Rigid yaw(double[] position,double angle) { return new Rigid(position,new double[]{0,Math.sin(angle/2),0,Math.cos(angle/2)}); }
    Rigid leveled() {
        double[] forward=rotate(new double[]{0,0,-1});
        if(Math.hypot(forward[0],forward[2])<0.27) return null;
        return yaw(t, Math.atan2(-forward[0],-forward[2]));
    }
    static double[] doubles(float[] a) {double[] b=new double[a.length];for(int i=0;i<a.length;i++) b[i]=a[i];return b;}
    static float[] floats(double[] a) {float[] b=new float[a.length];for(int i=0;i<a.length;i++) b[i]=(float)a[i];return b;}
    float[] matrix() {
        float[] m=new float[16];
        for(int col=0;col<3;col++){double[] v=new double[3];v[col]=1;v=rotate(v);for(int i=0;i<3;i++)m[4*col+i]=(float)v[i];}
        for(int i=0;i<3;i++)m[12+i]=(float)t[i];
        m[15]=1;
        return m;
    }
    static double[] triangulate(Rigid a,double[] localA,Rigid b,double[] localB){
        double[] u=unit(a.rotate(localA)), v=unit(b.rotate(localB)), delta=sub(b.t,a.t);
        double c=dot(u,v), det=1-c*c;
        if(det<0.0019 || c<0.45) return null;
        double d=(dot(delta,u)-c*dot(delta,v))/det, e=c*d-dot(delta,v);
        if(d<0.25 || e<0.25 || d>12 || e>12) return null;
        double[] p=add(a.t,scale(u,d)), other=add(b.t,scale(v,e));
        if(norm(sub(p,other))>Math.min(d,e)*0.005) return null;
        return scale(add(p,other),0.5);
    }
    @Override public String toString(){return Arrays.toString(t);}
}
