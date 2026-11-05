using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    public sealed class MeshData
    {
        public Vec3[] Vertices;
        public int[] Triangles;
    }
    public static class PrimitiveMeshes
    {
        public static MeshData Sphere(int segments,int rings,double radius)
        {
            if(segments<3||rings<2||radius<=0) throw new ArgumentException("Invalid sphere dimensions.");
            var v=new List<Vec3>(); var t=new List<int>(); v.Add(new Vec3(0,radius,0));
            for(int y=1;y<rings;y++) for(int x=0;x<segments;x++)
            { double p=Math.PI*y/rings,a=2*Math.PI*x/segments; v.Add(new Vec3(Math.Sin(p)*Math.Cos(a),Math.Cos(p),Math.Sin(p)*Math.Sin(a))*radius); }
            int bottom=v.Count; v.Add(new Vec3(0,-radius,0));
            for(int x=0;x<segments;x++)
            {
                int next=(x+1)%segments; Add(t,0,1+x,1+next);
                for(int y=0;y<rings-2;y++)
                { int a=1+y*segments+x,b=1+y*segments+next,c=a+segments,d=b+segments; Add(t,a,c,b); Add(t,b,c,d); }
                Add(t,bottom,1+(rings-2)*segments+next,1+(rings-2)*segments+x);
            }
            return Outward(v,t);
        }
        public static MeshData Cylinder(int segments,int rings,double radius,double length)
        {
            if(segments<3||rings<1||radius<=0||length<=0) throw new ArgumentException("Invalid cylinder dimensions.");
            var v=new List<Vec3>(); var t=new List<int>();
            for(int y=0;y<=rings;y++) for(int x=0;x<segments;x++)
            { double a=2*Math.PI*x/segments; v.Add(new Vec3(radius*Math.Cos(a),length*((double)y/rings-0.5),radius*Math.Sin(a))); }
            int bottom=v.Count; v.Add(new Vec3(0,-length/2,0)); int top=v.Count; v.Add(new Vec3(0,length/2,0));
            for(int x=0;x<segments;x++)
            {
                int next=(x+1)%segments;
                for(int y=0;y<rings;y++) { int a=y*segments+x,b=y*segments+next; Add(t,a,b,a+segments); Add(t,b,b+segments,a+segments); }
                Add(t,bottom,next,x); Add(t,top,rings*segments+x,rings*segments+next);
            }
            return Outward(v,t);
        }
        public static MeshData Grid(int columns,int rows,double width,double height)
        {
            if(columns<1||rows<1||width<=0||height<=0) throw new ArgumentException("Invalid grid dimensions.");
            var v=new Vec3[(columns+1)*(rows+1)]; var t=new List<int>();
            for(int y=0;y<=rows;y++) for(int x=0;x<=columns;x++)
            {
                int a=y*(columns+1)+x; v[a]=new Vec3(width*((double)x/columns-0.5),height*((double)y/rows-0.5),0);
                if(x<columns&&y<rows) { Add(t,a,a+1,a+columns+1); Add(t,a+1,a+columns+2,a+columns+1); }
            }
            return new MeshData { Vertices=v,Triangles=t.ToArray() };
        }
        static void Add(List<int> t,int a,int b,int c) { t.Add(a); t.Add(b); t.Add(c); }
        static MeshData Outward(List<Vec3> v,List<int> t)
        {
            for(int i=0;i<t.Count;i+=3)
            {
                Vec3 a=v[t[i]],b=v[t[i+1]],c=v[t[i+2]];
                if(Vec3.Dot(Vec3.Cross(b-a,c-a),(a+b+c)/3)<0) { int n=t[i+1]; t[i+1]=t[i+2]; t[i+2]=n; }
            }
            return new MeshData { Vertices=v.ToArray(),Triangles=t.ToArray() };
        }
    }
}
