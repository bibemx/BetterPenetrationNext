using System;
namespace BetterPenetrationNext.Core
{
    // Exact finite triangle/plane cuts, including genuinely open shells. The
    // caller owns the distance buffer. No extrapolated rings or caps are used.
    public static class SourceSurfaceSection
    {
        public static double Radius(Vec3[] vertices,int[] faces,Vec3 centre,Vec3 normal,double[] distances,out int cutTriangles)
        {
            if(vertices.Length!=distances.Length)throw new ArgumentException("Section buffer dimensions differ");
            double min=double.PositiveInfinity,max=double.NegativeInfinity;cutTriangles=0;
            for(int i=0;i<vertices.Length;i++){double d=Vec3.Dot(vertices[i]-centre,normal);distances[i]=d;min=Math.Min(min,d);max=Math.Max(max,d);}
            if(min>0||max<0)return 0;
            double radiusSquared=0;
            for(int f=0;f<faces.Length;f+=3)
            {
                int a=faces[f],b=faces[f+1],c=faces[f+2];double da=distances[a],db=distances[b],dc=distances[c];
                if((da>0&&db>0&&dc>0)||(da<0&&db<0&&dc<0))continue;
                cutTriangles++;
                Edge(vertices[a],vertices[b],da,db,centre,ref radiusSquared);
                Edge(vertices[b],vertices[c],db,dc,centre,ref radiusSquared);
                Edge(vertices[c],vertices[a],dc,da,centre,ref radiusSquared);
            }
            return Math.Sqrt(radiusSquared);
        }
        static void Edge(Vec3 a,Vec3 b,double da,double db,Vec3 centre,ref double radiusSquared)
        {
            if(da==0)radiusSquared=Math.Max(radiusSquared,(a-centre).LengthSquared);
            if((da<0&&db>0)||(db<0&&da>0))radiusSquared=Math.Max(radiusSquared,(a+(b-a)*(da/(da-db))-centre).LengthSquared);
        }
    }
}
