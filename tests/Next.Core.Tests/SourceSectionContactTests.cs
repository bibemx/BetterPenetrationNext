using System;
using BetterPenetrationNext.Core;
static class SourceSectionContactTests
{
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    static readonly Vec3 X=new Vec3(1,0,0),Y=new Vec3(0,1,0),Z=new Vec3(0,0,1);
    static MeshData Tube(int rings,int sides,double radius,double length,bool seam)
    {
        int stride=sides+1,f=0;var p=new Vec3[(rings+1)*stride];var faces=new int[rings*sides*6];
        for(int s=0;s<=rings;s++)for(int k=0;k<=sides;k++){double a=k*Math.PI*2/sides;p[s*stride+k]=new Vec3(radius*Math.Cos(a),radius*Math.Sin(a),length*s/rings);}
        for(int s=0;s<rings;s++)for(int k=0;k<sides;k++){int a=s*stride+k,b=a+1,c=a+stride,d=c+1;faces[f++]=a;faces[f++]=b;faces[f++]=c;faces[f++]=b;faces[f++]=d;faces[f++]=c;}return new MeshData{Vertices=p,Triangles=faces};
    }
    static void Brute(Vec3[] p,int[] faces,ReceiverRadialField field,int s,Vec3 c,Vec3 n,Vec3 x,Vec3 y)
    {
        for(int i=0;i<faces.Length;i+=3)for(int j=0;j<3;j++)
        {
            Vec3 a=p[faces[i+j]],b=p[faces[i+(j+1)%3]];double da=Vec3.Dot(a-c,n),db=Vec3.Dot(b-c,n);
            if(Math.Abs(da)<1e-12)field.Include(s,a-c,x,y,0);
            if(da*db<0)field.Include(s,a+(b-a)*(da/(da-db))-c,x,y,0);
        }
    }
    public static void Run(Action<string,Action> test)
    {
        test("Active sector release follows current contact independently of slow damping",()=>{
            var field=new ReceiverRadialField(2);var c=new[]{new Vec3(),Z};var baseline=new[]{.01,.01};field.Begin(baseline);field.Include(0,X*.04,X,Y,0);field.Include(1,X*.04,X,Y,0);field.Update(c,1,1,true);
            Require(field.Radius(0,.5,0)>.04,"Initial surface load missing");field.Begin(baseline);field.Include(0,-X*.03,X,Y,0);field.Include(1,-X*.03,X,Y,0);field.Update(c,.016,1,true);
            Require(Math.Abs(field.Radius(0,.5,0)-.01)<1e-12&&field.Radius(0,.5,Math.PI)>.03,"Previous opposite-side contact still dilates an unloaded wall");
        });
        test("Radial angular seam tolerates negative sub-ulp surface directions",()=>{
            var field=new ReceiverRadialField(2);field.Begin(new[]{.01,.01});field.Include(0,new Vec3(.02,-1e-20,0),X,Y,0);field.Update(new[]{new Vec3(),Z},.016,12,true);Require(field.Radius(0,0,0)>.02,"Angular wrap lost surface contact");
        });
        test("Finite source section BVH matches independent cuts of a bent open shell",()=>{
            var m=Tube(32,64,.02,.2,true);var posed=(Vec3[])m.Vertices.Clone();
            for(int i=0;i<posed.Length;i++){Vec3 p=posed[i];posed[i]+=new Vec3(.012*Math.Sin(p.Z*12),.004*Math.Sin(p.Z*9),0);}
            var search=new SourceSectionContacts(m.Vertices,m.Triangles);search.Refit(posed);var cached=new ReceiverRadialField(3);var brute=new ReceiverRadialField(3);var baseline=new[]{.0005,.0005,.0005};var centres=new[]{new Vec3(0,0,.03),new Vec3(0,0,.1),new Vec3(0,0,.18)};cached.Begin(baseline);brute.Begin(baseline);
            for(int s=0;s<3;s++){Vec3 n=new Vec3(.3*s,0,1).Normalized,x=Vec3.Cross(Y,n).Normalized;search.Include(cached,s,centres[s],n,x,Y,0);Brute(posed,m.Triangles,brute,s,centres[s],n,x,Y);}
            cached.Update(centres,1,12);brute.Update(centres,1,12);
            for(int s=0;s<2;s++)for(int k=0;k<128;k++)Require(Math.Abs(cached.Radius(s,.37,k*Math.PI/64)-brute.Radius(s,.37,k*Math.PI/64))<1e-12,"Spatial pruning changed the real surface cut");
            Require(search.TriangleTests<m.Triangles.Length/3,"BVH did not reduce total narrow checks across three sections");
        });
        test("Finite cuts never extrapolate an open shell beyond its tip",()=>{
            var m=Tube(16,32,.02,.2,true);var search=new SourceSectionContacts(m.Vertices,m.Triangles);var field=new ReceiverRadialField(2);var centres=new[]{new Vec3(0,0,.25),new Vec3(0,0,.3)};field.Begin(new[]{.001,.001});search.Refit(m.Vertices);
            for(int s=0;s<2;s++)search.Include(field,s,centres[s],Z,X,Y,0);field.Update(centres,1,12);
            Require(field.Radius(0,.5,0)==.001&&search.TriangleTests==0,"Open end invented a cap or extrapolated contact");
        });
        test("Finite plane refits retain distinct animated seam vertices",()=>{
            var p=new[]{new Vec3(.01,0,0),new Vec3(.01,.01,0),new Vec3(.01,0,.2),new Vec3(.01,0,0),new Vec3(.01,.01,0),new Vec3(.01,0,.2)};
            var faces=new[]{0,1,2,3,4,5};var search=new SourceSectionContacts(p,faces);var posed=(Vec3[])p.Clone();for(int i=3;i<6;i++)posed[i]+=new Vec3(.03,0,0);
            var field=new ReceiverRadialField(2);field.Begin(new[]{.001,.001});search.Refit(posed);var c=new[]{new Vec3(0,0,.05),new Vec3(0,0,.15)};for(int s=0;s<2;s++)search.Include(field,s,c[s],Z,X,Y,0);field.Update(c,1,12);
            Require(field.Radius(0,.5,0)>.04,"Coincident material identities were merged during refit");
        });
        test("Finite source contact refit and all section queries allocate no warm buffers",()=>{
            var m=Tube(16,32,.02,.2,true);var search=new SourceSectionContacts(m.Vertices,m.Triangles);var field=new ReceiverRadialField(2);var c=new[]{new Vec3(0,0,.05),new Vec3(0,0,.15)};var baseline=new[]{.001,.001};
            for(int pass=0;pass<30;pass++){search.Refit(m.Vertices);field.Begin(baseline);for(int s=0;s<2;s++)search.Include(field,s,c[s],Z,X,Y,0);field.Update(c,.016,12);}
            long before=GC.GetAllocatedBytesForCurrentThread();for(int pass=0;pass<30;pass++){search.Refit(m.Vertices);field.Begin(baseline);for(int s=0;s<2;s++)search.Include(field,s,c[s],Z,X,Y,0);field.Update(c,.016,12);}Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Section refit/query allocated");
        });
    }
}
