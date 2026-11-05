using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using BetterPenetrationNext.Core;

static partial class Program
{
    static int ReactiveControlChecks(Frame f,SimulationDefinition physical,SimulationDefinition support,ReceiverSupportTransfer transfer,ReceiverGuideDefinition contours,Source source,ReceiverSupportTransfer mucosal,ReactiveMaterialChart chart,string output)
    {
        Func<ReactiveReceiverRig> create=()=>new ReactiveReceiverRig(physical,support,transfer,contours,source.ReferenceVertices,source.Triangles,source.MaterialAxis,source.CreateSectionRig(),mucosal,chart);
        Func<ReactiveRigInput> input=()=>{var p=new ReactiveRigInput(f.Linear.Length,f.Morph.Length,f.Source.Length);Array.Copy(f.Linear,p.Bones,f.Linear.Length);Array.Copy(f.Translation,p.Translation,f.Translation.Length);Array.Copy(f.Morph,p.Morph,f.Morph.Length);Array.Copy(f.Source,p.Source,f.Source.Length);p.SourceRoot=f.Root;p.Dt=1.0/60;p.PosedSize=f.Size;p.HasSource=true;p.SourceFollowsPath=true;return p;};
        Func<ReactiveRigOutput> result=()=>new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,f.Source.Length,contours.Contours.Length);
        var a=create();var b=create();var ia=input();var ib=input();var oa=result();var ob=result();ia.Recovery=.1;ib.Recovery=9;
        for(int i=0;i<80;i++){a.Step(ia,oa);b.Step(ib,ob);}
        if(!oa.ReceiverActive||!ob.ReceiverActive)throw new Exception("Recorded pose did not establish active receiver");double activeDifference=0;for(int i=0;i<oa.FullWorld.Length;i++)activeDifference=Math.Max(activeDifference,(oa.FullWorld[i]-ob.FullWorld[i]).Length);if(activeDifference>1e-12)throw new Exception("RecoveryTime changed active geometry");
        var identity=new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1));ia.SourceFrame=identity;
        a.Step(ia,oa);var currentGuide=create();var og=result();currentGuide.GuideSourceOnly(ia,og,oa);double guideError=0;
        for(int i=0;i<og.SourceWorld.Length;i++)guideError=Math.Max(guideError,(og.SourceWorld[i]-oa.SourceWorld[i]).Length);
        if(guideError>1e-8)throw new Exception("Current-frame source guide disagrees with the same-frame full solver: "+guideError);
        var shifted=input();shifted.SourceFrame=identity;shifted.Recovery=ia.Recovery;Vec3 shiftGuide=new Vec3(.02,.01,-.03);
        for(int i=0;i<shifted.Source.Length;i++)shifted.Source[i]+=shiftGuide;shifted.SourceRoot+=shiftGuide;
        for(int i=0;i<shifted.Translation.Length;i++)shifted.Translation[i]+=shiftGuide;
        currentGuide.GuideSourceOnly(shifted,og,oa);double guideTransportError=0;
        for(int i=0;i<og.SourceWorld.Length;i++)guideTransportError=Math.Max(guideTransportError,(og.SourceWorld[i]-oa.SourceWorld[i]-shiftGuide).Length);
        if(guideTransportError>1e-8)throw new Exception("Current source guide used an old absolute pose: "+guideTransportError);
        for(int i=0;i<20;i++)currentGuide.GuideSourceOnly(ia,og,oa);long beforeGuide=GC.GetAllocatedBytesForCurrentThread();
        for(int i=0;i<30;i++)currentGuide.GuideSourceOnly(ia,og,oa);long guideAllocated=GC.GetAllocatedBytesForCurrentThread()-beforeGuide;
        if(guideAllocated!=0)throw new Exception("Current source guide allocated per-frame buffers");
        var accepted=(Vec3[])ob.FullWorld.Clone();ib.Recovery=4.5;ib.HasSource=false;b.Step(ib,ob);double releaseJump=0;for(int i=0;i<accepted.Length;i++)if(chart.Regions[i]>=0&&chart.Regions[i]<=5)releaseJump=Math.Max(releaseJump,(accepted[i]-ob.FullWorld[i]).Length);if(releaseJump>1e-8)throw new Exception("Inactive transition snapped receiver geometry");
        var restRig=create();var restInput=input();restInput.HasSource=false;var restResult=result();restRig.Step(restInput,restResult);double halfError=0;
        for(int i=0;i<271;i++)
        {
            b.Step(ib,ob);if(i==134)for(int k=0;k<ob.Offsets.Length;k++)if(chart.Regions[k]>=0&&chart.Regions[k]<=5)halfError=Math.Max(halfError,(ob.FullWorld[k]-(accepted[k]+restResult.FullWorld[k])*.5).Length);
        }
        if(halfError>1e-9)throw new Exception("Outer entry, wall and cervix did not recover together at 2.25 seconds: "+halfError);
        double remaining=0;for(int i=0;i<ob.Offsets.Length;i++)if(chart.Regions[i]>=0&&chart.Regions[i]<=5)remaining=Math.Max(remaining,(ob.FullWorld[i]-restResult.FullWorld[i]).Length);if(remaining>1e-9)throw new Exception("Actual mesh did not recover after 4.5 seconds");
        ib.HasSource=true;b.Step(ib,ob);if(!ob.ReceiverActive)throw new Exception("Repeat contact did not interrupt recovery");
        var remote=create();var ir=input();var or=result();Vec3 remoteShift=new Vec3(10,0,0);for(int i=0;i<ir.Source.Length;i++)ir.Source[i]+=remoteShift;ir.SourceRoot+=remoteShift;
        for(int i=0;i<10;i++)remote.Step(ir,or);if(or.ReceiverActive||or.EntryContact||or.SourceAdmitted)throw new Exception("Remote source activated the real receiver");
        foreach(var offset in or.SourceOffsets)if(offset.Length>1e-10)throw new Exception("Local source bending began before entry");
        currentGuide.GuideSourceOnly(ir,og,oa);foreach(var offset in og.SourceOffsets)if(offset.Length>1e-10)throw new Exception("A stale contact snapshot bent a source outside the current entrance");
        for(int i=0;i<or.Offsets.Length;i++)if((or.Offsets[i]-restResult.Offsets[i]).Length>1e-10)throw new Exception("Receiver deformed beyond configured rest before actual contact");
        // Deliberate compression of the actual Source against an actual cervical
        // triangle, with Source motion priority locked for isolated organ testing.
        var probe=create();var ip=input();var op=result();ip.SourceFollowsPath=false;ip.SourceCompliance=0;ip.HasSource=false;probe.Step(ip,op);ip.HasSource=true;
        Vec3 incoming=(op.Path[op.Path.Length-1]-op.Path[0]).Normalized,point=new Vec3(),normal=new Vec3();double best=-1;
        for(int t=0;t<chart.TriangleRegions.Length;t++)if(chart.TriangleRegions[t]==3)
        {
            int k=t*3;Vec3 p=op.FullWorld[support.Triangles[k]],q=op.FullWorld[support.Triangles[k+1]],r=op.FullWorld[support.Triangles[k+2]],n=Vec3.Cross(q-p,r-p).Normalized;double facing=Math.Abs(Vec3.Dot(n,incoming));if(facing>best){best=facing;point=(p+q+r)/3;normal=Vec3.Dot(n,incoming)>0?-n:n;}
        }
        probe.Landmarks.Update(ip.Source,new Vec3(1,0,0));
        // A translated tilted source can miss the entrance entirely in this
        // recorded pose. Rigidly orient this deliberately placed test probe
        // from the actual entry towards the selected real cervical triangle.
        Vec3 oldTip=probe.Landmarks.Tip,from=(oldTip-probe.Landmarks.Base).Normalized,to=(point-op.Path[0]).Normalized;
        Vec3 axis=Vec3.Cross(from,to);double cosine=Vec3.Dot(from,to);
        if(cosine<-.999999)throw new Exception("Probe fixture has an antiparallel approach");
        Func<Vec3,Vec3> rotate=v=>v+Vec3.Cross(axis,v)+Vec3.Cross(axis,Vec3.Cross(axis,v))/Math.Max(1e-12,1+cosine);
        Vec3 placedTip=point-normal*.003;for(int i=0;i<ip.Source.Length;i++)ip.Source[i]=placedTip+rotate(ip.Source[i]-oldTip);ip.SourceRoot=placedTip+rotate(ip.SourceRoot-oldTip);ip.CervixMaxDisplacement=.02;
        for(int i=0;i<100;i++)probe.Step(ip,op);double push=op.CervicalDisplacement.Length;if(push<.0005)throw new Exception("Actual cervical geometry did not react: push="+push+" depth="+op.CervicalPenetration+" residual="+op.CervicalResidual+" admitted="+op.SourceAdmitted+" entry="+op.EntryContact+" Rn="+op.TerminalCentre+" RnNormal="+op.TerminalNormal+" face="+point+" normal="+normal+" target="+(point-normal*.003)+" tip="+probe.Landmarks.Tip);
        Vec3 group=new Vec3();bool first=true;double shapeError=0;for(int i=0;i<op.Offsets.Length;i++)if(chart.Regions[i]==3||chart.Regions[i]==4||chart.Regions[i]==5){if(first){group=op.Offsets[i];first=false;}else shapeError=Math.Max(shapeError,(op.Offsets[i]-group).Length);}if(shapeError>1e-10)throw new Exception("Cervical reaction distorted organ shape");
        ip.CervixMaxDisplacement=.001;for(int i=0;i<60;i++)probe.Step(ip,op);if(op.CervicalDisplacement.Length>.001000001)throw new Exception("Configured cervical limit exceeded");double cappedPush=op.CervicalDisplacement.Length;
        ip.CervixMaxDisplacement=0;for(int i=0;i<80;i++)probe.Step(ip,op);if(op.CervicalDisplacement.Length>1e-8)throw new Exception("Zero cervical travel did not lock the group");
        ip.SourceCompliance=.35;for(int i=0;i<20;i++)probe.Step(ip,op);double cappedSourceMovement=0,cappedAttachmentError=0;
        for(int i=0;i<op.SourceOffsets.Length;i++){cappedSourceMovement=Math.Max(cappedSourceMovement,op.SourceOffsets[i].Length);if(probe.IsSourceAttachment(i))cappedAttachmentError=Math.Max(cappedAttachmentError,op.SourceOffsets[i].Length);}
        if(cappedSourceMovement<1e-5||cappedAttachmentError>1e-10)throw new Exception("Capped cervical joint lost actual source reaction or its anchored base");
        var length=create();var il=input();var ol=result();il.CanalLengthScale=1.15;length.Step(il,ol);double extension=ol.CanalExtension;if(extension<=0)throw new Exception("Canal length setting had no effect");
        var report=new{Scope="Actual recorded Source/Receiver ZIPMOD meshes, bones and pose on .NET6. Deliberate tip-placement test uses real cervical triangles but is not a live animation or clinical validation.",Passed=true,CurrentGuideFullSolverError=guideError,CurrentGuideTransportError=guideTransportError,CurrentGuideWarmAllocated=guideAllocated,CurrentGuideMs=og.TotalMs,ActiveGeometryDifferenceBetweenRecoveryPoint1And9=activeDifference,InactiveTransitionJump=releaseJump,HalfRecoveryErrorAt2Point25=halfError,RemainingTissueOffsetAfter4Point5=remaining,CervicalPush=push,CervicalRigidShapeError=shapeError,CappedCervicalPush=cappedPush,CappedCervicalSourceMovement=cappedSourceMovement,CappedCervicalAttachmentError=cappedAttachmentError,LengthExtension=extension,ReceiverLandmarks=ol.ReceiverLandmarkCount,SourceSizedStep=ol.ReceiverSpacing};
        string text=JsonSerializer.Serialize(report,new JsonSerializerOptions{WriteIndented=true});File.WriteAllText(Path.Combine(output,"control-checks.json"),text);Console.WriteLine(text);return 0;
    }
}
