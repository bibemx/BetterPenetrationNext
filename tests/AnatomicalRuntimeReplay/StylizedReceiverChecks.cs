using System;
using System.IO;
using System.Text.Json;
using BetterPenetrationNext.Core;

static partial class Program
{
    static int StylizedReceiverChecks(Frame f,SimulationDefinition physical,SimulationDefinition support,ReceiverSupportTransfer transfer,ReceiverGuideDefinition contours,Source source,ReceiverSupportTransfer mucosal,ReactiveMaterialChart chart,string directory)
    {
        Func<ReactiveReceiverRig> create=()=>new ReactiveReceiverRig(physical,support,transfer,contours,source.ReferenceVertices,source.Triangles,source.MaterialAxis,source.CreateSectionRig(),mucosal,chart);
        Func<ReactiveRigInput> input=()=>{var p=new ReactiveRigInput(f.Linear.Length,f.Morph.Length,f.Source.Length);Array.Copy(f.Linear,p.Bones,f.Linear.Length);Array.Copy(f.Translation,p.Translation,f.Translation.Length);Array.Copy(f.Morph,p.Morph,f.Morph.Length);Array.Copy(f.Source,p.Source,f.Source.Length);p.SourceRoot=f.Root;p.Dt=1.0/60;p.PosedSize=f.Size;p.HasSource=true;p.SourceFollowsPath=true;p.CervixMaxDisplacement=.035;p.CanalLengthScale=1.15;return p;};
        Func<ReactiveRigOutput> output=()=>new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,f.Source.Length,contours.Contours.Length);
        var rig=create();var native=input();var solved=output();native.HasSource=false;native.CanalLengthScale=1;rig.Step(native,solved);
        if(Math.Abs(solved.ConfiguredCanalArc-solved.NativeCanalArc)>1e-12||solved.CanalLengthDisplacement.Length>1e-12)throw new Exception("Length 1 changed the native centreline");
        native.CanalLengthScale=1.15;rig.Step(native,solved);double lengthError=Math.Abs(solved.ConfiguredCanalArc/solved.NativeCanalArc-1.15);if(lengthError>1e-6)throw new Exception("Configured arc length is not 115 percent of the native arc");
        if(!rig.TerminalLandmark.IsCanalFacingThroat)throw new Exception("Actual Receiver did not bind its canal-facing throat");
        native.HasSource=true;for(int i=0;i<80;i++)rig.Step(native,solved);
        Vec3 local=solved.UterusTranslationPelvis,rotation=solved.UterusRotationPelvis;
        if(local.Y<=0||Math.Abs(local.Z)>local.Y*.1200001)throw new Exception("Actual contact response escaped anatomical pelvis-up cone");
        double rigidError=RigidError(rig,solved);if(rigidError>1e-8)throw new Exception("Actual organ did not follow one rigid pose");
        double sourceRootError=0;for(int i=0;i<solved.SourceOffsets.Length;i++)if(rig.IsSourceAttachment(i))sourceRootError=Math.Max(sourceRootError,solved.SourceOffsets[i].Length);if(sourceRootError>1e-10)throw new Exception("Source attachment moved");
        double tipGap=solved.TerminalTipDistance;if(tipGap>.001)throw new Exception("Actual material tip did not meet the selected throat: "+tipGap);
        var guide=create();var guided=output();guide.GuideSourceOnly(native,guided,solved);double guidePathError=0;
        if(guide.SourcePath.Count!=rig.SourcePath.Count)throw new Exception("Current guide and worker use different control counts");
        for(int i=0;i<guide.SourcePath.Count;i++)guidePathError=Math.Max(guidePathError,(guide.SourcePath[i]-rig.SourcePath[i]).Length);
        if(guidePathError>1e-8)throw new Exception("Current Source and Receiver use different cervical path deformation: "+guidePathError);
        // Preserve the accepted local state while the pelvis actually changes
        // pose. No world-up or fixed-at-load frame may enter this transport.
        double movingPelvisError=0;
        for(int frame=0;frame<16;frame++)
        {
            var p=input();p.Dt=0;Vec3 turn=new Vec3(frame*.03,frame*.04,-frame*.02),shift=new Vec3(.2*frame,-.1*frame,.03*frame);
            for(int b=0;b<p.Bones.Length;b++){var m=p.Bones[b];p.Bones[b]=new Linear3(RigidOrganPose.Rotate(m.X,turn),RigidOrganPose.Rotate(m.Y,turn),RigidOrganPose.Rotate(m.Z,turn));p.Translation[b]=RigidOrganPose.Rotate(p.Translation[b],turn)+shift;}
            for(int i=0;i<p.Source.Length;i++)p.Source[i]=RigidOrganPose.Rotate(p.Source[i],turn)+shift;p.SourceRoot=RigidOrganPose.Rotate(p.SourceRoot,turn)+shift;
            rig.Step(p,solved);movingPelvisError=Math.Max(movingPelvisError,Math.Max((local-solved.UterusTranslationPelvis).Length,(rotation-solved.UterusRotationPelvis).Length));rigidError=Math.Max(rigidError,RigidError(rig,solved));
        }
        if(movingPelvisError>1e-10||rigidError>1e-8)throw new Exception("Motion retained an old world-space pelvis frame");
        native.Dt=0;rig.Step(native,solved);native.Dt=1.0/60;
        var active=(Vec3[])solved.FullWorld.Clone();native.HasSource=false;rig.Step(native,solved);double releaseJump=0;for(int i=0;i<active.Length;i++)releaseJump=Math.Max(releaseJump,(active[i]-solved.FullWorld[i]).Length);if(releaseJump>1e-8)throw new Exception("Release snapped actual geometry: "+releaseJump);
        for(int i=0;i<271;i++){rig.Step(native,solved);rigidError=Math.Max(rigidError,RigidError(rig,solved));}
        if(solved.CervicalDisplacement.Length>1e-10||solved.UterusRotation.Length>1e-10)throw new Exception("Organ did not recover in 4.5 seconds");
        var rest=create();var expected=output();rest.Step(native,expected);double recoveryError=0;for(int i=0;i<solved.FullWorld.Length;i++)recoveryError=Math.Max(recoveryError,(solved.FullWorld[i]-expected.FullWorld[i]).Length);if(recoveryError>1e-8)throw new Exception("Actual receiver did not recover its configured animated baseline");
        native.HasSource=true;rig.Step(native,solved);if(!solved.ReceiverActive)throw new Exception("Repeat contact did not interrupt release");
        var report=new{Scope="Recorded actual Source/Receiver geometry and bone/morph matrices on .NET6; moving-pelvis transport is an imposed numerical pose test. Excludes Unity controller/plugin execution, frame time and clinical validation.",Passed=true,CurrentGuideWorkerPathError=guidePathError,CanalFacingThroatPoints=rig.TerminalLandmark.SurfacePointCount,ConfiguredLengthRelativeError=lengthError,LocalOrganTranslation=new[]{local.X,local.Y,local.Z},LocalOrganRotation=new[]{rotation.X,rotation.Y,rotation.Z},RigidOrganError=rigidError,MovingPelvisLocalStateError=movingPelvisError,SourceAttachmentError=sourceRootError,MaterialTipGap=tipGap,InactiveTransitionJump=releaseJump,RecoveredGeometryError=recoveryError};
        string text=JsonSerializer.Serialize(report,new JsonSerializerOptions{WriteIndented=true});File.WriteAllText(Path.Combine(directory,"stylized-controls.json"),text);Console.WriteLine(text);return 0;
    }
    static double RigidError(ReactiveReceiverRig rig,ReactiveRigOutput result)
    {
        var pose=new RigidOrganPose{Pivot=result.TerminalCentre-result.CervicalDisplacement,Translation=result.CervicalDisplacement,Rotation=result.UterusRotation};double error=0;
        for(int i=0;i<result.FullWorld.Length;i++)if(rig.IsRigidOrganControl(i))error=Math.Max(error,(result.FullWorld[i]-pose.Point(rig.NativeSupportPoint(i)+result.CanalLengthDisplacement)).Length);return error;
    }
}
