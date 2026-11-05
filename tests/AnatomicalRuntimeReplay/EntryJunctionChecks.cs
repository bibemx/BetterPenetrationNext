using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
static partial class Program
{
    static int EntryJunctionChecks(List<Frame> frames,SimulationDefinition physical,SimulationDefinition support,ReceiverSupportTransfer transfer,ReceiverGuideDefinition guide,Source source,ReceiverSupportTransfer mucosal,ReactiveMaterialChart chart,string directory)
    {
        Func<ReactiveReceiverRig> create=()=>new ReactiveReceiverRig(physical,support,transfer,guide,source.ReferenceVertices,source.Triangles,source.MaterialAxis,source.CreateSectionRig(),mucosal,chart);
        var rig=create();var input=new ReactiveRigInput(physical.BoneNames.Length,physical.Morphs.Length,source.ReferenceVertices.Length);var result=new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,input.Source.Length,rig.SectionCount);
        int[] exterior=Enumerable.Range(0,chart.TriangleRegions.Length).Where(i=>chart.TriangleRegions[i]==0).SelectMany(i=>support.Triangles.Skip(i*3).Take(3)).ToArray();var surface=new TriangleSurface(support.Rest,exterior,SurfaceKind.OrientedShell);
        var dimensions=new SourceDimensions(source.CreateSectionRig(),frames[0].SourceLengthScale,frames[0].SourceWidthScale);int root=Array.IndexOf(physical.BoneNames,"MED_Pelvis");int totalRays=0,blocked=0;double symmetric=0;int count=0;
        for(int n=-30;n<frames.Count;n++)
        {
            var f=frames[Math.Max(0,n)];Array.Copy(f.Linear,input.Bones,f.Linear.Length);Array.Copy(f.Translation,input.Translation,f.Translation.Length);Array.Copy(f.Morph,input.Morph,f.Morph.Length);input.SourceRoot=f.Root;input.SourceFrame=f.SourceFrame;input.Dt=n<0?1.0/60:f.Dt;input.Frame=f.Id;input.PosedSize=f.Size;input.HasSource=true;input.SourceFollowsPath=true;
            input.CanalLengthScale=FrameSetting(f,"canalLengthScale",1);input.CervixMaxDisplacement=FrameSetting(f,"cervixMaxDisplacement",.025);input.MaxSource=FrameSetting(f,"maxSource",.12);input.MaxStrain=FrameSetting(f,"maxStrain",4);input.SourceCompliance=FrameSetting(f,"sourceStrength",.35);input.Response=FrameSetting(f,"contactDamping",12);input.Recovery=4.5;
            dimensions.Apply(f.Source,f.Root,f.SourceLengthScale,f.SourceWidthScale,input.Dt,input.Response,input.Source);rig.Step(input,result);if(n<0)continue;
            foreach(Vec3 p in result.FullWorld)if(!p.IsFinite)throw new Exception("Nonfinite entrance surface");
            if(result.ContactCanalExtension>1e-12)throw new Exception("Junction correction extended the canal");
            symmetric=Math.Max(symmetric,result.EntranceSymmetryResidual);surface.Refit(result.FullWorld);
            Vec3 centre=new Vec3();foreach(int v in guide.Boundaries[0])centre+=result.Physical[v]/guide.Boundaries[0].Length;
            // The oriented R0->R1 plane, followed throughout actual animation.
            Vec3 normal=(result.Path[1]-result.Path[0]).Normalized,side=input.Bones[root].X;side=(side-normal*Vec3.Dot(side,normal)).Normalized;Vec3 up=Vec3.Cross(normal,side);double radius=double.PositiveInfinity;
            foreach(int v in guide.Boundaries[0]){Vec3 q=result.Physical[v]-centre;q-=normal*Vec3.Dot(q,normal);radius=Math.Min(radius,q.Length);}radius*=.6;
            for(int x=-7;x<=7;x++)for(int y=-7;y<=7;y++)if(x*x+y*y<=49){Vec3 p=centre+side*(x*radius/7)+up*(y*radius/7);SurfaceSample hit;totalRays++;if(surface.TryRaycast(p-normal*.02,p+normal*.02,out hit))blocked++;}
            count++;
        }
        if(blocked>0)throw new Exception("Exterior body still covers the central R0 aperture: "+blocked+" / "+totalRays);
        if(symmetric>1e-8)throw new Exception("Entrance symmetry regressed");
        File.WriteAllText(Path.Combine(directory,"native-input.json"),JsonSerializer.Serialize(new{SourceReference=source.ReferenceVertices,SourceFaces=source.Triangles,SourceReferenceRoot=source.ReferenceRoot,SourceAxis=source.MaterialAxis,input.Source,input.SourceRoot,input.SourceFrame,input.Bones,input.Translation,input.Morph,input.CanalLengthScale,input.CervixMaxDisplacement,input.MaxSource,input.MaxStrain,input.SourceCompliance,input.Response,input.PosedSize},ExportOptions));
        input.HasSource=false;input.Dt=0;var previous=(Vec3[])result.FullWorld.Clone();rig.Step(input,result);double transition=0;for(int i=0;i<previous.Length;i++)transition=Math.Max(transition,(result.FullWorld[i]-previous[i]).Length);if(transition>1e-8)throw new Exception("Junction release snapped");
        for(int i=0;i<271;i++){input.Dt=1.0/60;rig.Step(input,result);}
        var baseline=create();var empty=new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,input.Source.Length,rig.SectionCount);baseline.Step(input,empty);double recovery=0;for(int i=0;i<result.FullWorld.Length;i++)recovery=Math.Max(recovery,(result.FullWorld[i]-empty.FullWorld[i]).Length);if(recovery>1e-8)throw new Exception("Junction failed to recover after 4.5 seconds");
        File.WriteAllText(Path.Combine(directory,"entry-junction-checks.json"),JsonSerializer.Serialize(new{Scope="Actual recorded Source/Receiver skin/morph/animation data on .NET6, full production Step; not Unity controller execution or FPS",Passed=true,AnimatedStates=count,ProbeRays=totalRays,ExteriorOcclusions=blocked,MaximumBilateralResidual=symmetric,InactiveTransitionJump=transition,FinalRecoveryError=recovery},new JsonSerializerOptions{WriteIndented=true}));
        Console.WriteLine("ENTRY_JUNCTION_CHECKS_PASSED states="+count+" rays="+totalRays);return 0;
    }
}
