using System;
using BetterPenetrationNext.Core;
internal static class SectionRigTests
{
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    static MeshData Tube()
    {
        const int rings=25,sides=16;var vertices=new Vec3[rings*sides];var triangles=new int[(rings-1)*sides*6];int t=0;
        for(int r=0;r<rings;r++)for(int s=0;s<sides;s++)vertices[r*sides+s]=new Vec3(.01*Math.Cos(s*2*Math.PI/sides),.01*Math.Sin(s*2*Math.PI/sides),r*.1/(rings-1));
        for(int r=0;r<rings-1;r++)for(int s=0;s<sides;s++){int a=r*sides+s,b=r*sides+(s+1)%sides,c=a+sides,d=b+sides;triangles[t++]=a;triangles[t++]=b;triangles[t++]=c;triangles[t++]=b;triangles[t++]=d;triangles[t++]=c;}
        return new MeshData {Vertices=vertices,Triangles=triangles};
    }
    static Vec3[] Bent(Vec3[] rest){var result=(Vec3[])rest.Clone();for(int i=0;i<result.Length;i++)result[i]+=new Vec3(.02*Math.Sin(Math.PI*rest[i].Z/.1),0,0);return result;}
    static double CentreError(Vec3[] values){double maximum=0;for(int ring=0;ring<25;ring++){Vec3 centre=new Vec3();for(int side=0;side<16;side++)centre+=values[ring*16+side]/16;maximum=Math.Max(maximum,new Vec3(centre.X,centre.Y,0).Length);}return maximum;}
    public static void Run(Action<string,Action> test)
    {
        test("1.0.7 area limiter finds the first boundary, including invalid intermediate corrections",()=>{
            var p=new[]{new Vec3(),new Vec3(1,0,0),new Vec3(0,1,0)};var faces=new[]{0,1,2};
            double linear=SourceDeformationLimiter.Limit(p,new[]{new Vec3(),new Vec3(-2,0,0),new Vec3()},faces);
            Require(Math.Abs(linear-.495)<1e-9,"Linear area constraint was not solved exactly");
            double quadratic=SourceDeformationLimiter.Limit(p,new[]{new Vec3(),new Vec3(-3,0,0),new Vec3(0,-3,0)},faces);
            Require(Math.Abs(quadratic-.3)<1e-9,"Limiter jumped over a folded intermediate configuration");
            Require(SourceDeformationLimiter.Limit(p,new[]{new Vec3(1,2,3),new Vec3(1,2,3),new Vec3(1,2,3)},faces)==1,"Rigid translation was limited");
        });
        test("1.0.7 analytical area limiter preserves a valid prefix for random spatial corrections",()=>{
            var random=new Random(7251);var p=new[]{new Vec3(.2,-.1,.5),new Vec3(1.3,.2,.4),new Vec3(.1,1.1,.8)};var d=new Vec3[3];var faces=new[]{0,1,2};
            var normal=Vec3.Cross(p[1]-p[0],p[2]-p[0]);double area=normal.LengthSquared;
            for(int sample=0;sample<400;sample++){
                for(int i=0;i<3;i++)d[i]=new Vec3(random.NextDouble()*6-3,random.NextDouble()*6-3,random.NextDouble()*6-3);
                double limit=SourceDeformationLimiter.Limit(p,d,faces);Require(limit>=0&&limit<=1&&Vec3.Finite(limit),"Nonfinite or unbounded area limit");
                for(int j=0;j<=40;j++){double s=limit*j/40;var n=Vec3.Cross(p[1]-p[0]+(d[1]-d[0])*s,p[2]-p[0]+(d[2]-d[0])*s);Require(Vec3.Dot(n,normal)>=area*(.01-1e-12),"Invalid intermediate source face");}
            }
        });
        test("1.0.7 local section projection repairs a folded face without moving endpoints or allocating",()=>{
            var p=new[]{new Vec3(),new Vec3(1,0,.5),new Vec3(0,1,1)};var field=new Vec3[5];var projector=new SourceSectionAreaConstraints(new[]{0.0,.5,1.0},new[]{0,1,2},5);field[2]=new Vec3(-2,0,0);
            projector.Project(p,field,3);var normal=Vec3.Cross(p[1]-p[0],p[2]-p[0]);var corrected=Vec3.Cross(p[1]+field[2]-p[0],p[2]-p[0]);
            Require(Vec3.Dot(normal,corrected)>=normal.LengthSquared*.01999999,"Local section repair left the face folded");Require(field[0].Length==0&&field[4].Length==0,"Section endpoints lost their guide target");
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++){field[2]=new Vec3(-2,0,0);projector.Project(p,field,3);}Require(before==GC.GetAllocatedBytesForCurrentThread(),"Active local constraint projection allocated");
        });
        test("1.0.7 section rig removes an existing animated bend along the entire open source",()=>{
            var mesh=Tube();var animated=Bent(mesh.Vertices);var output=new Vec3[animated.Length];var oldOutput=new Vec3[animated.Length];var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(),new Vec3(0,0,.1)},false);var rig=new SourceSectionRig(mesh.Vertices,mesh.Triangles,new Vec3(),new Vec3(0,0,1));var old=new SourceRodGuide();
            for(int frame=0;frame<90;frame++){rig.Capture(path,animated,new Vec3(),1.0/60,.04);rig.MapVertices(animated,output);old.Capture(path,new Vec3(),new Vec3(0,0,.1),1.0/60,.04);old.MapVertices(animated,oldOutput);}
            double before=CentreError(oldOutput),after=CentreError(output);Require(before>.0199&&after<1e-5,"Existing bone bend survived the new section correction");Require(rig.GeometryLimitScale>.999,"Valid neutral bend was unnecessarily limited");
            for(int i=0;i<output.Length;i++)Require((output[i]-animated[i]).Length<=.040000001&&output[i].IsFinite,"Source movement cap or finiteness failed");
            Require(new SimulationTopology(mesh.Vertices,mesh.Triangles).BoundaryEdges==32,"A true open rim was capped");
            Console.WriteLine("SECTION_RIG legacyWholeCentreError="+before+" indexedWholeCentreError="+after);
        });
        test("1.0.7 indexed source correction retains independently skinned coincident material vertices",()=>{
            var mesh=Tube();var rest=new Vec3[mesh.Vertices.Length+1];Array.Copy(mesh.Vertices,rest,mesh.Vertices.Length);rest[rest.Length-1]=rest[12*16];var pose=Bent(rest);pose[pose.Length-1]+=new Vec3(0,.002,0);var output=new Vec3[pose.Length];var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(),new Vec3(0,0,.1)},false);var rig=new SourceSectionRig(rest,mesh.Triangles,new Vec3(),new Vec3(0,0,1));
            for(int frame=0;frame<90;frame++){rig.Capture(path,pose,new Vec3(),1.0/60,.04);rig.MapVertices(pose,output);}
            Require(((output[output.Length-1]-output[12*16])-(pose[pose.Length-1]-pose[12*16])).Length<1e-12,"Different skin paths were merged");
            var body=new SourceContactBody();body.BeginFrame(new BidirectionalContactSettings(),1.0/60);body.ApplyDeformation(rig);for(int i=0;i<output.Length;i++)Require((body.ToCorrectedVertex(i,pose[i])-output[i]).Length<1e-12,"Render/contact indexed mapping differs");
        });
        test("1.0.7 section sampling is cached, allocation-free and resets without retaining shape",()=>{
            var mesh=Tube();var pose=Bent(mesh.Vertices);var output=new Vec3[pose.Length];var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(),new Vec3(0,0,.1)},false);var rig=new SourceSectionRig(mesh.Vertices,mesh.Triangles,new Vec3(),new Vec3(0,0,1));
            for(int i=0;i<10;i++){rig.Capture(path,pose,new Vec3(),1.0/60,.04);rig.MapVertices(pose,output);}long before=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<100;i++){rig.Capture(path,pose,new Vec3(),1.0/60,.04);rig.MapVertices(pose,output);}Require(GC.GetAllocatedBytesForCurrentThread()==before,"Section rig rebuilt/allocated in its hot path");
            rig.Capture(path,pose,new Vec3(),1.0/60,.001);rig.MapVertices(pose,output);for(int i=0;i<pose.Length;i++)Require((output[i]-pose[i]).Length<=.001000001,"Lowered cap retained an old deformation");
            rig.Reset();rig.MapVertices(pose,output);for(int i=0;i<pose.Length;i++)Require((output[i]-pose[i]).Length==0,"Reset changed source baseline");
        });
    }
}
