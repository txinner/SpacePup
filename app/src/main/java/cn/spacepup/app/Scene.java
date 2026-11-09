package cn.spacepup.app;

import android.opengl.GLES11Ext;
import android.opengl.Matrix;
import static android.opengl.GLES20.*;
import com.google.ar.core.Coordinates2d;
import com.google.ar.core.Frame;
import java.nio.*;
import java.util.ArrayList;

final class Scene {
    int texture,bg,mesh,vertices;
    private int quadPosition, textureCoordinates, cameraSampler, dogPosition, dogColor, dogMvp;
    private boolean cameraGeometryDirty=true;
    private final float[] mvp=new float[16];
    FloatBuffer dog;
    final FloatBuffer quad=buffer(new float[]{-1,-1,1,-1,-1,1,1,1});
    final FloatBuffer uv=buffer(new float[8]);
    static FloatBuffer buffer(float[] data){
        FloatBuffer b=ByteBuffer.allocateDirect(data.length*4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        b.put(data).position(0);return b;
    }
    static int shader(int type,String source){
        int s=glCreateShader(type);glShaderSource(s,source);glCompileShader(s);int[] ok=new int[1];glGetShaderiv(s,GL_COMPILE_STATUS,ok,0);
        if(ok[0]==0)throw new IllegalStateException(glGetShaderInfoLog(s));return s;
    }
    static int program(String vert,String frag){
        int v=shader(GL_VERTEX_SHADER,vert),f=shader(GL_FRAGMENT_SHADER,frag);
        int p=glCreateProgram();glAttachShader(p,v);glAttachShader(p,f);glLinkProgram(p);
        int[] ok=new int[1];glGetProgramiv(p,GL_LINK_STATUS,ok,0);
        if(ok[0]==0)throw new IllegalStateException(glGetProgramInfoLog(p));
        glDeleteShader(v);glDeleteShader(f);return p;
    }
    void init(){
        bg=program("attribute vec2 pos;attribute vec2 uv;varying vec2 tc;void main(){tc=uv;gl_Position=vec4(pos,0.,1.);}",
            "#extension GL_OES_EGL_image_external : require\nprecision mediump float;uniform samplerExternalOES camera;varying vec2 tc;void main(){gl_FragColor=texture2D(camera,tc);}");
        mesh=program("uniform mat4 mvp;attribute vec3 pos;attribute vec3 color;varying vec3 rgb;void main(){rgb=color;gl_Position=mvp*vec4(pos,1.);}",
            "precision mediump float;varying vec3 rgb;void main(){gl_FragColor=vec4(rgb,1.);}");
        quadPosition=glGetAttribLocation(bg,"pos");
        textureCoordinates=glGetAttribLocation(bg,"uv");
        cameraSampler=glGetUniformLocation(bg,"camera");
        dogPosition=glGetAttribLocation(mesh,"pos");
        dogColor=glGetAttribLocation(mesh,"color");
        dogMvp=glGetUniformLocation(mesh,"mvp");
        invalidateCameraGeometry();
        int[] tex=new int[1];glGenTextures(1,tex,0);texture=tex[0];glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,texture);
        glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
        glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
        Model model=new Model();dog=buffer(model.build());vertices=dog.capacity()/6;
    }
    void invalidateCameraGeometry(){cameraGeometryDirty=true;}
    void background(Frame frame){
        if(cameraGeometryDirty || frame.hasDisplayGeometryChanged()){
            quad.position(0);uv.position(0);
            frame.transformCoordinates2d(Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES,quad,Coordinates2d.TEXTURE_NORMALIZED,uv);
            cameraGeometryDirty=false;
        }
        glDisable(GL_DEPTH_TEST);glDepthMask(false);glUseProgram(bg);glActiveTexture(GL_TEXTURE0);glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,texture);
        glUniform1i(cameraSampler,0);
        attr(quadPosition,quad,2,0,0);attr(textureCoordinates,uv,2,0,0);glDrawArrays(GL_TRIANGLE_STRIP,0,4);
        glDisableVertexAttribArray(quadPosition);glDisableVertexAttribArray(textureCoordinates);glDepthMask(true);
    }
    void renderDog(float[] projection,Rigid mapCamera,Rigid dogAnchor){
        if(mapCamera==null || dogAnchor==null)return;
        float[] viewModel=mapCamera.inverse().times(dogAnchor).matrix();
        Matrix.multiplyMM(mvp,0,projection,0,viewModel,0);
        glEnable(GL_DEPTH_TEST);glUseProgram(mesh);
        glUniformMatrix4fv(dogMvp,1,false,mvp,0);
        attr(dogPosition,dog,3,24,0);attr(dogColor,dog,3,24,3);
        glDrawArrays(GL_TRIANGLES,0,vertices);
        glDisableVertexAttribArray(dogPosition);glDisableVertexAttribArray(dogColor);
    }
    static void attr(int location,FloatBuffer b,int size,int stride,int offset){
        b.position(offset);glEnableVertexAttribArray(location);glVertexAttribPointer(location,size,GL_FLOAT,false,stride,b);
    }

    /** Hand-built low-poly terrier, +Z is the nose, feet at y=0. Size is measured in metres. */
    static final class Model{
        final ArrayList<Float> data=new ArrayList<>();
        final int ginger=0xCF7837,cream=0xFAE7C6,dark=0x352925,collar=0x358B85;
        void tri(double[] a,double[] b,double[] c,int rgb){
            double[] n=Rigid.unit(Rigid.cross(Rigid.sub(b,a),Rigid.sub(c,a)));
            double shade=0.77+0.22*Math.abs(Rigid.dot(n,new double[]{-0.35,0.82,0.44}));
            for(double[] v:new double[][]{a,b,c}){for(double x:v)data.add((float)x);
                for(int shift:new int[]{16,8,0})data.add((float)(((rgb>>shift)&255)/255.0*shade));}
        }
        double[] ellPoint(double[] p,double[] r,double lat,double lon){
            return new double[]{p[0]+r[0]*Math.cos(lat)*Math.sin(lon),p[1]+r[1]*Math.sin(lat),p[2]+r[2]*Math.cos(lat)*Math.cos(lon)};
        }
        void ell(double x,double y,double z,double rx,double ry,double rz,int color){
            double[] p={x,y,z},r={rx,ry,rz};int rings=8,sides=12;
            for(int i=0;i<rings;i++)for(int j=0;j<sides;j++){
                double low=Math.PI*i/rings-Math.PI/2,high=low+Math.PI/rings,start=2*Math.PI*j/sides,end=start+2*Math.PI/sides;
                double[] a=ellPoint(p,r,low,start),b=ellPoint(p,r,low,end),c=ellPoint(p,r,high,end),d=ellPoint(p,r,high,start);
                tri(a,b,c,color);tri(a,c,d,color);
            }
        }
        float[] build(){
            ell(0,.183,-.025,.115,.115,.194,ginger);
            for(int side:new int[]{-1,1}){
                ell(side*.074,.085,.072,.036,.083,.044,ginger);
                ell(side*.081,.088,-.115,.044,.086,.055,ginger);
                for(double z:new double[]{.083,-.108})ell(side*.078,.028,z,.040,.028,.054,cream);
            }
            ell(0,.27,.101,.101,.033,.098,collar);
            ell(0,.346,.122,.123,.113,.112,ginger);
            for(int side:new int[]{-1,1}){
                double[] a={side*.035,.416,.14},b={side*.111,.402,.131},c={side*.094,.521,.112},d={side*.081,.421,.068};
                tri(a,b,c,ginger);tri(b,d,c,ginger);tri(d,a,c,ginger);
                tri(new double[]{side*.057,.424,.141},new double[]{side*.099,.419,.137},new double[]{side*.091,.489,.120},0xA65E4B);
                ell(side*.073,.370,.208,.012,.016,.014,dark);
                ell(side*.073,.375,.220,.004,.005,.003,0xffffff);
            }
            ell(0,.323,.209,.077,.052,.067,cream);ell(0,.342,.269,.031,.022,.016,dark);
            ell(0,.256,-.190,.045,.073,.053,ginger);
            ell(0,.304,-.166,.043,.047,.065,cream);
            float[] result=new float[data.size()];for(int i=0;i<result.length;i++)result[i]=data.get(i);return result;
        }
    }
}
