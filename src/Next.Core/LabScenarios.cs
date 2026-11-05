using System;
namespace BetterPenetrationNext.Core
{
    // Neutral technical meshes used solely for reproducible laboratory scenarios.
    public static class LabScenarios
    {
        public static MeshData Tube(int segments,int rings,double radius,double length)
        {
            if(segments<3||rings<1||radius<=0||length<=0) throw new ArgumentException("Invalid tube.");
            var v=new Vec3[segments*(rings+1)]; var t=new int[segments*rings*6]; int index=0;
            for(int y=0;y<=rings;y++) for(int x=0;x<segments;x++) { double a=2*Math.PI*x/segments; v[y*segments+x]=new Vec3(radius*Math.Cos(a),length*((double)y/rings-.5),radius*Math.Sin(a)); }
            for(int y=0;y<rings;y++) for(int x=0;x<segments;x++) { int a=y*segments+x,b=y*segments+(x+1)%segments,c=a+segments,d=b+segments; t[index++]=a; t[index++]=b; t[index++]=c; t[index++]=b; t[index++]=d; t[index++]=c; }
            return new MeshData { Vertices=v,Triangles=t };
        }
    }
}
