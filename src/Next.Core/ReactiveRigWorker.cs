using System;
using System.Threading;

namespace BetterPenetrationNext.Core
{
    // One bounded outstanding snapshot. The worker owns only portable arrays;
    // no Unity object, BepInEx entry or renderer crosses this boundary.
    public sealed class ReactiveRigWorker : IDisposable
    {
        readonly object gate=new object();readonly AutoResetEvent wake=new AutoResetEvent(false);readonly Thread thread;
        readonly Action<ReactiveRigInput,ReactiveRigOutput> step;readonly ReactiveRigInput input;readonly ReactiveRigOutput calculated,published;
        bool busy,stopping,hasResult;string failure;
        public int Submitted,Dropped,Completed;
        public string Failure {get{lock(gate)return failure;}}
        public bool Busy {get{lock(gate)return busy;}}
        public ReactiveRigWorker(ReactiveReceiverRig rig,int bones,int morphs,int support,int physical,int source)
            :this(RequireRig(rig).Step,rig.SectionCount,bones,morphs,support,physical,source) { }
        static ReactiveReceiverRig RequireRig(ReactiveReceiverRig rig){if(rig==null)throw new ArgumentNullException("rig");return rig;}
        // A portable processor also lets the frame-transfer protocol be tested
        // independently of a particular material mesh or Unity scene.
        public ReactiveRigWorker(Action<ReactiveRigInput,ReactiveRigOutput> processor,int sections,int bones,int morphs,int support,int physical,int source)
        {if(processor==null)throw new ArgumentNullException("processor");step=processor;input=new ReactiveRigInput(bones,morphs,source);calculated=new ReactiveRigOutput(support,physical,source,sections);published=new ReactiveRigOutput(support,physical,source,sections);thread=new Thread(Run){IsBackground=true,Name="BPN reactive material rig"};thread.Start();}
        public bool Submit(ReactiveRigInput frame)
        {
            if(frame==null)throw new ArgumentNullException("frame");
            lock(gate){if(stopping)return false;if(busy){Dropped++;return false;}if(frame.Bones.Length!=input.Bones.Length||frame.Translation.Length!=input.Translation.Length||frame.Morph.Length!=input.Morph.Length||frame.Source.Length!=input.Source.Length)throw new ArgumentException("Worker input dimensions changed");Array.Copy(frame.Bones,input.Bones,input.Bones.Length);Array.Copy(frame.Translation,input.Translation,input.Translation.Length);Array.Copy(frame.Morph,input.Morph,input.Morph.Length);Array.Copy(frame.Source,input.Source,input.Source.Length);input.CervixMaxDisplacement=frame.CervixMaxDisplacement;input.CervixFollowStrength=frame.CervixFollowStrength;input.CanalLengthScale=frame.CanalLengthScale;input.SourceRoot=frame.SourceRoot;input.SourceFrame=frame.SourceFrame;input.Frame=frame.Frame;input.Dt=frame.Dt;input.PosedSize=frame.PosedSize;input.SourceFollowsPath=frame.SourceFollowsPath;input.WholeSource=frame.WholeSource;input.HasSource=frame.HasSource;input.MaxSource=frame.MaxSource;input.MaxStrain=frame.MaxStrain;input.SourceCompliance=frame.SourceCompliance;input.TissueStrength=frame.TissueStrength;input.Compliance=frame.Compliance;input.Response=frame.Response;input.Recovery=frame.Recovery;input.MaximumOffsetRatio=frame.MaximumOffsetRatio;busy=true;Submitted++;wake.Set();return true;}
        }
        public bool TryRead(ReactiveRigOutput destination)
        {lock(gate){if(!hasResult)return false;Copy(published,destination);hasResult=false;return true;}}
        static void Copy(ReactiveRigOutput a,ReactiveRigOutput b)
        {
            // Validate the complete publication before copying anything. In
            // particular, diagnostic counts are producer data: multiplying an
            // unchecked count by five can overflow and corrupt an unread result.
            int receiverCount=a.ReceiverLandmarkCount;
            if(b==null||a.Offsets.Length!=b.Offsets.Length||a.FullWorld.Length!=b.FullWorld.Length||a.Physical.Length!=b.Physical.Length||a.SourceOffsets.Length!=b.SourceOffsets.Length||a.SourceWorld.Length!=b.SourceWorld.Length||a.Path.Length!=b.Path.Length||a.LandmarkCount<0||a.LandmarkCount>a.LandmarkPoints.Length||a.LandmarkCount>b.LandmarkPoints.Length||
               receiverCount<0||receiverCount>a.ReceiverSourcePairs.Length||receiverCount>b.ReceiverSourcePairs.Length||receiverCount>a.ReceiverLandmarkPoints.Length/5||receiverCount>b.ReceiverLandmarkPoints.Length/5||
               a.SourceAnimatedCentres.Length!=b.SourceAnimatedCentres.Length||a.SourceGuidedCentres.Length!=b.SourceGuidedCentres.Length||a.SourceSolvedCentres.Length!=b.SourceSolvedCentres.Length||a.TerminalPoints.Length!=b.TerminalPoints.Length||a.ExternalLandmarkPoints.Length!=b.ExternalLandmarkPoints.Length)
                throw new ArgumentException("Worker output dimensions changed");
            Array.Copy(a.Offsets,b.Offsets,a.Offsets.Length);Array.Copy(a.FullWorld,b.FullWorld,a.FullWorld.Length);Array.Copy(a.Physical,b.Physical,a.Physical.Length);Array.Copy(a.SourceOffsets,b.SourceOffsets,a.SourceOffsets.Length);Array.Copy(a.SourceWorld,b.SourceWorld,a.SourceWorld.Length);Array.Copy(a.Path,b.Path,a.Path.Length);b.LandmarkCount=a.LandmarkCount;Array.Copy(a.LandmarkPoints,b.LandmarkPoints,a.LandmarkCount);
            Array.Copy(a.SourceAnimatedCentres,b.SourceAnimatedCentres,a.SourceAnimatedCentres.Length);Array.Copy(a.SourceSolvedCentres,b.SourceSolvedCentres,a.SourceSolvedCentres.Length);b.SourceFrame=a.SourceFrame;b.SourceBudgetBlend=a.SourceBudgetBlend;b.SourceMaximumTurn=a.SourceMaximumTurn;b.CervicalPenetration=a.CervicalPenetration;b.CervicalResidual=a.CervicalResidual;b.CervicalDisplacement=a.CervicalDisplacement;b.CervicalQueries=a.CervicalQueries;
            Array.Copy(a.SourceGuidedCentres,b.SourceGuidedCentres,a.SourceGuidedCentres.Length);
            b.CervicalMotionAxis=a.CervicalMotionAxis;b.GuidanceCervicalDisplacement=a.GuidanceCervicalDisplacement;b.CervicalReactionAttempts=a.CervicalReactionAttempts;b.CervicalReactionRejected=a.CervicalReactionRejected;
            b.SourceArcLength=a.SourceArcLength;b.TerminalRootDistance=a.TerminalRootDistance;b.TerminalWithinSourceLength=a.TerminalWithinSourceLength;b.MaterialTipVertex=a.MaterialTipVertex;b.SourceGuideWeight=a.SourceGuideWeight;
            b.EntranceRegularizedVertices=a.EntranceRegularizedVertices;b.EntranceSymmetryPairs=a.EntranceSymmetryPairs;b.EntranceSymmetryResidual=a.EntranceSymmetryResidual;b.EntranceMidlineVertices=a.EntranceMidlineVertices;b.EntranceMidlineResidual=a.EntranceMidlineResidual;
            b.MaterialEdgeCount=a.MaterialEdgeCount;b.MaterialFaceCount=a.MaterialFaceCount;
            b.CervicalTipPressure=a.CervicalTipPressure;b.EffectiveCervicalTravel=a.EffectiveCervicalTravel;
            b.NativeCanalArc=a.NativeCanalArc;b.ConfiguredCanalArc=a.ConfiguredCanalArc;b.CanalLengthDisplacement=a.CanalLengthDisplacement;
            b.UterusRotation=a.UterusRotation;b.UterusRotationPelvis=a.UterusRotationPelvis;b.UterusTranslationPelvis=a.UterusTranslationPelvis;b.UterusPelvisUp=a.UterusPelvisUp;b.GuidanceCervicalRotationPelvis=a.GuidanceCervicalRotationPelvis;
            b.CervicalMaterialBlend=a.CervicalMaterialBlend;b.CervicalContactTolerance=a.CervicalContactTolerance;b.TerminalCanalFacing=a.TerminalCanalFacing;b.TerminalSurfacePoints=a.TerminalSurfacePoints;
            b.TerminalAvailable=a.TerminalAvailable;b.TerminalCentre=a.TerminalCentre;b.TerminalNormal=a.TerminalNormal;b.TerminalRadius=a.TerminalRadius;b.TerminalTipAxial=a.TerminalTipAxial;b.TerminalTipLateral=a.TerminalTipLateral;b.TerminalTipDistance=a.TerminalTipDistance;b.ContactCanalExtension=a.ContactCanalExtension;Array.Copy(a.TerminalPoints,b.TerminalPoints,5);
            b.ReceiverActive=a.ReceiverActive;b.EntryContact=a.EntryContact;b.SourceAdmitted=a.SourceAdmitted;b.ReceiverSpacing=a.ReceiverSpacing;b.CanalExtension=a.CanalExtension;b.ReceiverLandmarkCount=a.ReceiverLandmarkCount;Array.Copy(a.ReceiverLandmarkPoints,b.ReceiverLandmarkPoints,a.ReceiverLandmarkCount*5);Array.Copy(a.ReceiverSourcePairs,b.ReceiverSourcePairs,a.ReceiverLandmarkCount);Array.Copy(a.ExternalLandmarkPoints,b.ExternalLandmarkPoints,5);
            b.Frame=a.Frame;b.Contacts=a.Contacts;b.SectionTests=a.SectionTests;b.TriangleTests=a.TriangleTests;b.ActiveSections=a.ActiveSections;b.ContainedVertices=a.ContainedVertices;b.EngagedVertices=a.EngagedVertices;b.SourceVertices=a.SourceVertices;b.WorkerThread=a.WorkerThread;b.LimitingEdge=a.LimitingEdge;b.CaptureMs=a.CaptureMs;b.PathMs=a.PathMs;b.ContactMs=a.ContactMs;b.TransferMs=a.TransferMs;b.TotalMs=a.TotalMs;b.MaximumStrain=a.MaximumStrain;b.MaximumPrimaryStrain=a.MaximumPrimaryStrain;b.MaterialBlend=a.MaterialBlend;b.MaxSourceMovement=a.MaxSourceMovement;b.MaximumResidual=a.MaximumResidual;b.EntranceRadius=a.EntranceRadius;b.RequiredEntranceRadius=a.RequiredEntranceRadius;b.Carrier=a.Carrier;b.CarrierTranslation=a.CarrierTranslation;b.RecoveryFraction=a.RecoveryFraction;
        }
        void Run()
        {
            try{for(;;){wake.WaitOne();lock(gate){if(stopping)return;}try{step(input,calculated);lock(gate){if(!stopping){Copy(calculated,published);hasResult=true;Completed++;}busy=false;}}catch(Exception e){lock(gate){failure=e.ToString();busy=false;}}}}
            finally{wake.Dispose();}
        }
        public void Dispose(){lock(gate){if(!stopping){stopping=true;wake.Set();}}if(Thread.CurrentThread!=thread)thread.Join(2000);}
    }
    public sealed class SnapshotSourceDeformation : IIndexedSourceDeformation
    {
        readonly Vec3[] offsets;public double MaximumDisplacement {get;private set;}public Bounds3 DisplacementBounds {get;private set;}
        public SnapshotSourceDeformation(int count){offsets=new Vec3[count];}
        public void Capture(Vec3[] values,double limit){double max=0;Bounds3 bounds=Bounds3.Empty;for(int i=0;i<offsets.Length;i++){offsets[i]=Vec3.ClampLength(values[i],limit);max=Math.Max(max,offsets[i].Length);bounds.Include(offsets[i]);}MaximumDisplacement=max;DisplacementBounds=bounds;}
        public Vec3 MapVertex(int index,Vec3 animated){return animated+offsets[index];}
        public void MapVertices(Vec3[] animated,Vec3[] output){for(int i=0;i<offsets.Length;i++)output[i]=MapVertex(i,animated[i]);}
        // This snapshot is indexed by the original skin identity. Generic point
        // queries do not invent a nearest material identity across a seam.
        public Vec3 Map(Vec3 point){return point;}
    }
}
