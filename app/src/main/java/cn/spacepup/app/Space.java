package cn.spacepup.app;

import java.io.*;
import java.util.*;
import java.util.zip.CRC32;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

final class Space {
    static final int MAX_POINTS=12000;
    final ArrayList<Landmark> points=new ArrayList<>();
    private final HashMap<String,ArrayList<Landmark>> cells=new HashMap<>();
    Rigid dog;
    int frames, usefulViews;
    double extent;
    final String name, id;
    Space(String name,String id){this.name=name;this.id=id;}
    static final class Landmark {
        final double[] p; final byte[] descriptor;
        Landmark(double[] p,byte[] d){this.p=p;descriptor=d;}
    }
    boolean add(double[] p,byte[] desc){
        if(points.size()>=MAX_POINTS) return false;
        String key=(int)Math.floor(p[0]*12)+"/"+(int)Math.floor(p[1]*12)+"/"+(int)Math.floor(p[2]*12);
        ArrayList<Landmark> bucket=cells.computeIfAbsent(key,k->new ArrayList<>());
        for(Landmark other:bucket) if(Rigid.norm(Rigid.sub(other.p,p))<0.045) return false;
        Landmark item=new Landmark(p,desc);bucket.add(item);points.add(item);return true;
    }
    boolean quality(){return usefulViews>=5 && points.size()>=120 && extent>=0.65;}
    File file(File dir){return new File(dir,id+".pup");}
    void save(File dir) throws IOException {
        if(!quality() || dog==null)throw new IOException("空间尚未通过质量检查");
        ByteArrayOutputStream raw=new ByteArrayOutputStream();
        try(DataOutputStream out=new DataOutputStream(raw)){
            out.writeInt(0x50555031);out.writeUTF(name);out.writeInt(frames);out.writeInt(usefulViews);out.writeDouble(extent);
            for(double n:dog.t)out.writeDouble(n);for(double n:dog.q)out.writeDouble(n);
            out.writeInt(points.size());
            for(Landmark item:points){for(double x:item.p)out.writeFloat((float)x);out.write(item.descriptor);}
        }
        byte[] data=raw.toByteArray(); CRC32 checksum=new CRC32();checksum.update(data);
        File pending=File.createTempFile("space-", ".pending",dir);
        try{
            try(FileOutputStream stream=new FileOutputStream(pending)){
                stream.write(data);new DataOutputStream(stream).writeLong(checksum.getValue());stream.getFD().sync();
            }
            Files.move(pending.toPath(),file(dir).toPath(),StandardCopyOption.ATOMIC_MOVE,StandardCopyOption.REPLACE_EXISTING);
        }finally{Files.deleteIfExists(pending.toPath());}
    }
    static Space load(File file)throws IOException {
        if(file.length()>1500000)throw new IOException("地图文件过大");
        byte[] data=Files.readAllBytes(file.toPath());
        if(data.length<80)throw new IOException("地图不完整");
        CRC32 crc=new CRC32();crc.update(data,0,data.length-8);
        try(DataInputStream in=new DataInputStream(new ByteArrayInputStream(data))){
            if(in.readInt()!=0x50555031)throw new IOException("不支持的地图版本");
            Space s=new Space(in.readUTF(),file.getName().replace(".pup",""));
            s.frames=in.readInt();s.usefulViews=in.readInt();s.extent=in.readDouble();
            double[] t=new double[3],q=new double[4];for(int i=0;i<3;i++)t[i]=in.readDouble();for(int i=0;i<4;i++)q[i]=in.readDouble();
            s.dog=new Rigid(t,q);
            int n=in.readInt();if(n<120 || n>MAX_POINTS)throw new IOException("地图特征数无效");
            for(int i=0;i<n;i++){double[] p={in.readFloat(),in.readFloat(),in.readFloat()};byte[] d=new byte[32];in.readFully(d);s.points.add(new Landmark(p,d));}
            if(in.readLong()!=crc.getValue() || in.available()!=0 || !s.quality())throw new IOException("地图校验失败");
            return s;
        }catch(IllegalArgumentException e){throw new IOException("地图坐标损坏",e);}
    }
}
