using System;
using BetterPenetrationNext.Core;

static class InextensibleRigTests
{
    static void Require(bool condition,string message){if(!condition)throw new Exception(message);}
    static MeshData Tube()
    {
        const int rings=26,sides=32;var p=new Vec3[rings*(sides+1)];var t=new int[(rings-1)*sides*6];int f=0;
        for(int r=0;r<rings;r++)for(int k=0;k<=sides;k++){double angle=k*Math.PI*2/sides;p[r*33+k]=new Vec3(.1*Math.Cos(angle),.08*Math.Sin(angle),-.08+r*.08);}
        for(int r=0;r<rings-1;r++)for(int k=0;k<sides;k++){int a=r*33+k,b=a+1,c=a+33,d=c+1;t[f++]=a;t[f++]=b;t[f++]=c;t[f++]=b;t[f++]=d;t[f++]=c;}
        return new MeshData{Vertices=p,Triangles=t};
    }
    static InextensibleSourceRig Create(MeshData m){return new InextensibleSourceRig(new SourceSectionRig(m.Vertices,m.Triangles,new Vec3(),new Vec3(0,0,1)));}
    static AnatomicalPathConstraintSystem Path(){return new AnatomicalPathConstraintSystem(new[]{new Vec3(0,0,.4),new Vec3(.1,0,.9),new Vec3(.4,0,1.4)},true,8);}
    static void Links(InextensibleSourceRig r)
    {for(int i=1;i<r.Count;i++)Require(Math.Abs((r.Solved[i]-r.Solved[i-1]).Length-(r.Animated[i]-r.Animated[i-1]).Length)<1e-12,"Link length changed");}
    public static void Run(Action<string,Action> test)
    {
        test("Finite open-shell cuts never extrapolate a missing section",()=>{
            var m=Tube();var buffer=new double[m.Vertices.Length];int cuts;
            Require(Math.Abs(SourceSurfaceSection.Radius(m.Vertices,m.Triangles,new Vec3(0,0,1),new Vec3(0,0,1),buffer,out cuts)-.1)<1e-12&&cuts>0,"Real open shell section radius differs");
            Require(SourceSurfaceSection.Radius(m.Vertices,m.Triangles,new Vec3(0,0,3),new Vec3(0,0,1),buffer,out cuts)==0&&cuts==0,"An extrapolated tip opened a section beyond the mesh");
            double radius=SourceSurfaceSection.Radius(m.Vertices,m.Triangles,new Vec3(0,0,.8),new Vec3(1,0,.00001).Normalized,buffer,out cuts);
            Require(radius<1.3&&cuts>0,"Grazing orientation produced an unbounded projected radius");
            for(int i=0;i<10;i++)SourceSurfaceSection.Radius(m.Vertices,m.Triangles,new Vec3(0,0,1),new Vec3(0,0,1),buffer,out cuts);
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<30;i++)SourceSurfaceSection.Radius(m.Vertices,m.Triangles,new Vec3(0,0,1),new Vec3(0,0,1),buffer,out cuts);
            Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Section search allocated per frame");
        });
        test("Arc look-ahead stays continuous while animation crosses finite guide samples",()=>{
            var m=Tube();var r=Create(m);Vec3 previous=new Vec3();double maxStep=0;
            for(int f=0;f<=180;f++)
            {
                Vec3 root=new Vec3(0,0,f*.002);var posed=(Vec3[])m.Vertices.Clone();for(int i=0;i<posed.Length;i++)posed[i]+=root;
                r.Guide(Path(),posed,root,.1,.35,1.0/120,12,.7);Links(r);Vec3 tip=r.Solved[r.Count-1]-root;
                if(f>0)maxStep=Math.Max(maxStep,(tip-previous).Length);previous=tip;
                Require(r.MaximumTurn<=.300001&&r.MaximumDisplacement<=.700001,"Guide violated curvature or displacement constraints");
            }
            Require(maxStep<.01,"Positional path crossing produced a tip snap: "+maxStep);
        });
        test("Oblique cervical approach detects a finite surface missed by terminal-parallel rays",()=>{
            var p=new[]{new Vec3(-.2,-.2,1),new Vec3(.2,-.2,1),new Vec3(0,.2,1)};var c=new CervicalSurfaceContact(p,new[]{0,1,2},new[]{3});c.Refit(p);Vec3 n;
            Vec3 tip=new Vec3(.5,0,1.2),entry=new Vec3(-2,0,0);
            Require(c.PenetrationAlongRay(tip,new Vec3(),entry,new Vec3(0,0,1),0,.1,out n)==0,"Parallel control ray unexpectedly hit");
            Require(Math.Abs(c.PenetrationFromEntry(tip,new Vec3(),entry,0,out n)-.2)<1e-12&&n.Z<0,"Actual oblique approach crossed without reaction");
            Require(c.PenetrationFromEntry(new Vec3(.5,0,.8),new Vec3(),entry,0,out n)==0,"Withdrawn tip was retained");
        });
        test("Cervical approach transforms both sweep endpoints with permitted organ travel",()=>{
            var p=new[]{new Vec3(-.2,-.2,1),new Vec3(.2,-.2,1),new Vec3(0,.2,1)};var c=new CervicalSurfaceContact(p,new[]{0,1,2},new[]{3});c.Refit(p);Vec3 n;
            var shift=new Vec3(.7,.1,.3);
            Require(Math.Abs(c.PenetrationFromEntry(new Vec3(0,0,1.2)+shift,shift,new Vec3(0,0,0)+shift,0,out n)-.2)<1e-12,"Travel changed finite-surface contact depth");
        });
        test("Rigid section transport preserves open-shell cross sections and anchored base",()=>{
            var m=Tube();var r=Create(m);var p=new Vec3[m.Vertices.Length];r.Guide(Path(),m.Vertices,new Vec3(),.1,1,1.0/60,12,.7);r.MapVertices(m.Vertices,p);Links(r);
            Require(r.MaximumTurn<=.300001,"Curvature exceeded");Require((p[0]-m.Vertices[0]).Length==0,"Base detached");
            for(int ring=2;ring<26;ring++){int a=ring*33;Require(Math.Abs((p[a]-p[a+16]).Length-(m.Vertices[a]-m.Vertices[a+16]).Length)<1e-10,"Cross section squashed");}
            Require((p[p.Length-1]-m.Vertices[p.Length-1]).Length>.01,"Guide inactive");
        });
        test("Guide extrapolates and leaves axial animation free beyond its endpoint",()=>{
            var m=Tube();var r=Create(m);var straight=new AnatomicalPathConstraintSystem(new[]{new Vec3(0,0,.2),new Vec3(0,0,.7)},false);var p=new Vec3[m.Vertices.Length];
            r.Guide(straight,m.Vertices,new Vec3(),.1,1,.016,12,.5);r.MapVertices(m.Vertices,p);Require((p[p.Length-1]-m.Vertices[p.Length-1]).Length<1e-10,"Tip pinned to guide end");
            var animated=(Vec3[])m.Vertices.Clone();for(int i=0;i<animated.Length;i++)animated[i]+=new Vec3(0,0,.12);
            r.Guide(straight,animated,new Vec3(0,0,.12),.1,1,.016,12,.5);r.MapVertices(animated,p);Require((p[p.Length-1]-animated[p.Length-1]).Length<1e-10,"Axial motion cancelled");
        });
        test("Displacement saturation reduces angular guidance without per-vertex crushing",()=>{
            var m=Tube();var r=Create(m);var p=new Vec3[m.Vertices.Length];r.Guide(Path(),m.Vertices,new Vec3(),.1,1,.016,12,.05);r.MapVertices(m.Vertices,p);Links(r);
            Require(r.BudgetBlend<1,"Limit did not activate");for(int i=0;i<p.Length;i++)Require((p[i]-m.Vertices[i]).Length<=.05000001,"Displacement budget exceeded");
            Require(Math.Abs((p[330]-p[346]).Length-.2)<1e-10,"Saturated section changed width");
        });
        test("Worker rebase retains current axial movement and exact current link lengths",()=>{
            var m=Tube();var old=Create(m);old.Guide(Path(),m.Vertices,new Vec3(),.1,1,.016,12,.5);var expected=new Vec3[m.Vertices.Length];old.MapVertices(m.Vertices,expected);
            var current=Create(m);var v=(Vec3[])m.Vertices.Clone();var shift=new Vec3(.2,.1,.12);for(int i=0;i<v.Length;i++)v[i]+=shift;
            var identity=new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1));current.Rebase(v,shift,old.Animated,old.Solved,identity,identity,.5);var output=new Vec3[v.Length];current.MapVertices(v,output);Links(current);
            for(int i=0;i<output.Length;i++)Require((output[i]-expected[i]-shift).Length<1e-9,"Rebase erased current animation");
        });
        test("Different seam animation remains independent under rigid section transport",()=>{
            var m=Tube();var r=Create(m);var animated=(Vec3[])m.Vertices.Clone();animated[330+32]+=new Vec3(0,.025,0);var p=new Vec3[animated.Length];r.Guide(Path(),animated,new Vec3(),.1,1,.016,12,.5);r.MapVertices(animated,p);
            Require(Math.Abs((p[330]-p[362]).Length-(animated[330]-animated[362]).Length)<1e-10,"Seam identities were merged");
        });
        test("Cervical surface contact releases on withdrawal and does not cap a guide",()=>{
            var p=new[]{new Vec3(-1,-1,1),new Vec3(1,-1,1),new Vec3(0,1,1)};var c=new CervicalSurfaceContact(p,new[]{0,1,2},new[]{3});c.Refit(p);Vec3 n;
            Require(Math.Abs(c.Penetration(new Vec3(0,0,1.1),new Vec3(),new Vec3(0,0,1),0,.5,out n)-.1)<1e-12&&n.Z<0,"Missing unilateral reaction");
            Require(c.Penetration(new Vec3(0,0,.8),new Vec3(),new Vec3(0,0,1),0,.5,out n)==0,"Cervix retained a tensile attachment");
        });
        test("Warm rod guidance, saturation and mapping allocate no per-frame buffers",()=>{
            var m=Tube();var r=Create(m);var p=new Vec3[m.Vertices.Length];var path=Path();for(int i=0;i<20;i++){r.Guide(path,m.Vertices,new Vec3(),.1,1,.016,12,.05);r.MapVertices(m.Vertices,p);}
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<30;i++){r.Guide(path,m.Vertices,new Vec3(),.1,1,.016,12,.05);r.MapVertices(m.Vertices,p);}Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Rod update allocated");
        });
        test("Finite BVH rays stop at real triangles and respect segment length",()=>{
            var p=new[]{new Vec3(-1,-1,1),new Vec3(1,-1,1),new Vec3(0,1,1)};var s=new TriangleSurface(p,new[]{0,1,2},SurfaceKind.OrientedShell);SurfaceSample hit;
            Require(!s.TryRaycast(new Vec3(),new Vec3(0,0,.9),out hit),"Ray extended past its segment");
            Require(s.TryRaycast(new Vec3(),new Vec3(0,0,2),out hit)&&(hit.Point-new Vec3(0,0,1)).Length<1e-12,"Ray missed actual triangle");
            Require(!s.TryRaycast(new Vec3(3,0,0),new Vec3(3,0,2),out hit),"Ray fabricated an infinite plane");
        });
        test("Cervical contact catches a crossed front surface beyond nearest-query range",()=>{
            var p=new[]{new Vec3(-1,-1,1),new Vec3(1,-1,1),new Vec3(0,1,1)};var c=new CervicalSurfaceContact(p,new[]{0,1,2},new[]{3});c.Refit(p);Vec3 n;
            double depth=c.PenetrationAlongRay(new Vec3(0,0,1.6),new Vec3(),new Vec3(),new Vec3(0,0,1),0,.1,out n);
            Require(Math.Abs(depth-.6)<1e-12&&n.Z<0,"Crossed contact disappeared beyond local search range");
            Require(c.PenetrationAlongRay(new Vec3(0,0,.8),new Vec3(),new Vec3(),new Vec3(0,0,1),0,.1,out n)==0,"Withdrawing tip acquired contact");
        });
        test("Cervical rays preserve a real opening while detecting its surrounding rim",()=>{
            var p=new[]{new Vec3(-1,-1,1),new Vec3(1,-1,1),new Vec3(1,1,1),new Vec3(-1,1,1),new Vec3(-.2,-.2,1),new Vec3(.2,-.2,1),new Vec3(.2,.2,1),new Vec3(-.2,.2,1)};
            var faces=new[]{0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7};var regions=new int[8];for(int i=0;i<regions.Length;i++)regions[i]=3;
            var c=new CervicalSurfaceContact(p,faces,regions);c.Refit(p);Vec3 n;
            Require(c.PenetrationAlongRay(new Vec3(0,0,1.2),new Vec3(),new Vec3(),new Vec3(0,0,1),0,.1,out n)==0,"Real opening was capped");
            Require(Math.Abs(c.PenetrationAlongRay(new Vec3(.5,0,1.2),new Vec3(),new Vec3(),new Vec3(0,0,1),0,.1,out n)-.2)<1e-12,"Surrounding real rim was missed");
        });
    }
}
