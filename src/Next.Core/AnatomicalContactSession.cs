using System;

namespace BetterPenetrationNext.Core
{
    public interface IAnatomicalSourceBinding
    {
        Vec3[] AnimatedVertices {get;}
        Vec3[] ReferenceVertices {get;}
        int[] Triangles {get;}
        Vec3 Root {get;}
        Vec3 MaterialAxis {get;}
        SourceContactBody Body {get;}
        SourceSectionRig CreateSectionRig();
        TriangleSurface Refit(SourceSectionRig rig,Vec3[] initializationOverride);
    }
    // Shared by the native player and recorded-pose acceptance runner. Unity
    // capture/upload stays in its adapter; this class contains no Unity API.
    public sealed class AnatomicalContactSession
    {
        readonly SimulationDefinition physical,support;
        readonly ReceiverSupportTransfer transfer;
        readonly AnimatedPathProfile profile;
        readonly ReceiverGuideDefinition contourDefinition;
        readonly int root;
        readonly Vec3[] anchors,material,fullMaterial,actual,stable,field,localField,auxiliary,pathPoints,guidedSource;
        readonly MaterialMetric[] metrics;
        readonly CoupledChannelGuide channel;
        readonly SoftBodySettings tissue;
        PhysicsPolicy policy;
        AnatomicalPathConstraintSystem reference;
        SourceSectionRig rig,sampler;
        ReceiverPreparation preparation;
        IAnatomicalSourceBinding owner;
        TriangleSurface lastSurface;
        Linear3 previousRoot;
        bool initialized,active;
        double recoveryElapsed=4.5;
        public readonly SoftBodySolver Solver;
        public readonly Vec3[] WorldOffsets;
        public Vec3[] StableAnchors {get{return anchors;}}
        public AnatomicalPathConstraintSystem Path {get{return channel.SharedPath??reference;}}
        public bool Ready {get{return preparation!=null&&preparation.Ready;}}
        public double RecoverySeconds=4.5;
        public int ContactSamples=1024;
        public int RefinementPasses=6;
        public double MaximumSourceMovement {get;private set;}

        public AnatomicalContactSession(SimulationDefinition physical,SimulationDefinition support,ReceiverSupportTransfer transfer,AnimatedPathProfile profile,ReceiverGuideDefinition contours,int sourceVertexCount)
        {
            this.physical=physical;this.support=support;this.transfer=transfer;this.profile=profile;contourDefinition=contours;
            root=Array.IndexOf(physical.BoneNames,"MED_Pelvis");if(root<0||support.BoneNames.Length!=physical.BoneNames.Length)throw new ArgumentException("Stable material carrier is missing");
            for(int i=0;i<physical.BoneNames.Length;i++)if(physical.BoneNames[i]!=support.BoneNames[i])throw new ArgumentException("Receiver palettes differ");
            anchors=new Vec3[physical.Rest.Length];material=new Vec3[anchors.Length];metrics=new MaterialMetric[anchors.Length];auxiliary=new Vec3[anchors.Length];
            fullMaterial=new Vec3[support.Rest.Length];actual=new Vec3[fullMaterial.Length];stable=new Vec3[fullMaterial.Length];field=new Vec3[fullMaterial.Length];localField=new Vec3[fullMaterial.Length];WorldOffsets=new Vec3[fullMaterial.Length];
            pathPoints=new Vec3[profile.ControlCount];guidedSource=new Vec3[sourceVertexCount];
            Solver=new SoftBodySolver(new SimulationTopology(physical.Rest,physical.Triangles,physical.AllowJunctions),physical.PinWeights){MaterialPositions=material,MaterialMetrics=metrics,MaximumCavityResidualPasses=RefinementPasses};
            Solver.GuideTargetEnabled=new bool[anchors.Length];for(int i=0;i<anchors.Length;i++)Solver.GuideTargetEnabled[i]=true;
            // The confinement mesh explicitly defines primary controls. Auxiliary
            // guides never overwrite the primary contour/contact coordinates.
            foreach(int i in physical.ConfinementTriangles)Solver.GuideTargetEnabled[i]=false;
            channel=new CoupledChannelGuide(physical.GuideCoordinates,physical.GuideWeights,25);
            Solver.ContourGuides=new ReceiverContourGuides(contours);
            tissue=new SoftBodySettings{StretchCompliance=.02,BendCompliance=.02,ShapeCompliance=.2,Damping=8,MaximumOffsetRatio=.6,RecoverySeconds=4.5,PreserveVolume=false};
            policy=PhysicsPolicy.Resolve(PhysicsQuality.Performance);policy.MaximumSubsteps=1;
            foreach(int i in transfer.PhysicalIndices)support.GuideWeights[i]=1;
        }
        void CaptureMaterial(Linear3[] bones,Vec3[] translations,double[] morphs)
        {
            if(bones.Length!=physical.BoneNames.Length||translations.Length!=bones.Length)throw new ArgumentException("Captured receiver palette changed");
            for(int i=0;i<anchors.Length;i++)
            {
                Vec3 p=physical.Rest[i];if(physical.Morphs!=null)for(int m=0;m<physical.Morphs.Length;m++)p+=physical.Morphs[m].Deltas[i]*morphs[m];material[i]=p;anchors[i]=bones[root].Multiply(p)+translations[root];
                metrics[i]=new MaterialMetric();for(int j=physical.InfluenceOffsets[i];j<physical.InfluenceOffsets[i+1];j++)metrics[i].Add(bones[root],physical.BoneWeights[j]);
            }
            for(int i=0;i<fullMaterial.Length;i++)
            {
                Vec3 p=support.Rest[i];if(support.Morphs!=null)for(int m=0;m<support.Morphs.Length;m++)p+=support.Morphs[m].Deltas[i]*morphs[m];fullMaterial[i]=p;stable[i]=bones[root].Multiply(p)+translations[root];Vec3 q=new Vec3();
                for(int j=support.InfluenceOffsets[i];j<support.InfluenceOffsets[i+1];j++){int b=support.BoneIndices[j];q+=(bones[b].Multiply(p)+translations[b])*support.BoneWeights[j];}actual[i]=q;
            }
            for(int i=0;i<pathPoints.Length;i++)
            {
                Vec3 p=new Vec3();for(int j=profile.PointOffsets[i];j<profile.PointOffsets[i+1];j++){Vec3 v=profile.Rest[j];if(profile.MorphDeltas!=null)for(int m=0;m<profile.MorphDeltas.Length;m++)v+=profile.MorphDeltas[m][j]*morphs[m];p+=(bones[root].Multiply(v)+translations[root])*profile.Weights[j];}pathPoints[i]=p;
            }
            if(reference==null)reference=new AnatomicalPathConstraintSystem(pathPoints,true);else reference.UpdateControls(pathPoints);
        }
        public void Release(Linear3[] bones,Vec3[] translations,double[] morphs,double elapsed)
        {
            CaptureMaterial(bones,translations,morphs);active=false;recoveryElapsed=Math.Min(RecoverySeconds,recoveryElapsed+Math.Max(0,elapsed));double t=RecoverySeconds>0?recoveryElapsed/RecoverySeconds:1,w=1-t*t*(3-2*t);
            for(int i=0;i<WorldOffsets.Length;i++)WorldOffsets[i]=(bones[root].Multiply(localField[i])+(stable[i]-actual[i])*support.GuideWeights[i])*w;
        }
        public void Step(Linear3[] bones,Vec3[] translations,double[] morphs,IAnatomicalSourceBinding source,BidirectionalContactSettings contact,double elapsed,double posedSize,bool sourceFollowsPath)
        {
            if(source==null||source.AnimatedVertices.Length!=guidedSource.Length)throw new ArgumentException("Source topology changed");
            CaptureMaterial(bones,translations,morphs);double dt=Math.Max(1e-5,Math.Min(.1,elapsed));
            if(!initialized){Solver.Reset(anchors);initialized=true;previousRoot=bones[root];}
            else for(int i=0;i<anchors.Length;i++)Solver.Transport(i,previousRoot,bones[root]);previousRoot=bones[root];
            if(!active&&recoveryElapsed>0){double t=Math.Min(1,recoveryElapsed/Math.Max(1e-9,RecoverySeconds)),w=1-t*t*(3-2*t);Solver.Reset(anchors);for(int i=0;i<anchors.Length;i++)Solver.Positions[i]=anchors[i]+bones[root].Multiply(localField[transfer.PhysicalIndices[i]])*w;}
            active=true;recoveryElapsed=0;
            if(!ReferenceEquals(owner,source))
            {
                owner=source;rig=source.CreateSectionRig();sampler=source.CreateSectionRig();preparation=new ReceiverPreparation(source.ReferenceVertices,source.MaterialAxis);lastSurface=null;
                Solver.Cavity=new CavityContactConstraints(anchors,physical.ConfinementTriangles,source.ReferenceVertices,ContactSamples,source.Triangles){Source=source.Body,Path=reference,UseConservativeProxy=true,UseCachedContacts=true,WholeSourceCoverage=true,UseExactSurfaceCoverage=true};Solver.Cavity.Orient(anchors,reference);
            }
            var body=source.Body;var vertices=source.AnimatedVertices;AnatomicalPathConstraintSystem sourcePath=reference;
            if(sourceFollowsPath){rig.Capture(reference,vertices,source.Root,dt,contact.MaxSourceDisplacement*.8,12);rig.MapVertices(vertices,guidedSource);channel.UseCorrectedSource=true;channel.Capture(reference,rig,guidedSource,anchors,1,tissue.MaximumOffsetRatio*posedSize,dt);}
            else{sampler.Capture(reference,vertices,source.Root,dt,0);channel.UseCorrectedSource=false;channel.Capture(reference,sampler,vertices,anchors,.9,tissue.MaximumOffsetRatio*posedSize,dt);sourcePath=channel.SourcePath;rig.Capture(sourcePath,vertices,source.Root,dt,contact.MaxSourceDisplacement*.8,12);rig.MapVertices(vertices,guidedSource);}
            body.PreserveContactAcrossDeformation=true;body.InverseMass=contact.Mode==ContactMode.Bidirectional?contact.SourceCorrectionStrength:0;body.ApplyDeformation(rig);body.TranslationAxis=sourcePath.Closest(rig.Map(rig.AnimatedCentre(rig.SectionCount-1))).Tangent;body.MaximumContactTravel=contact.MaxSourceDisplacement;
            var receiverPath=channel.SharedPath;preparation.Capture(reference,receiverPath,guidedSource,dt,channel.EndClearance);if(!preparation.Ready)receiverPath=preparation.Path;
            Solver.PreparingReceiver=!preparation.Ready;Solver.PreparationHasMatchingContactSurface=true;Solver.Cavity.WorldSourceOverride=preparation.SourceOverride;Solver.Cavity.Path=receiverPath;Solver.Cavity.AnimatedSource=vertices;
            Solver.ContourGuides.EntryTranslationSource=body;Solver.ContourGuides.Capture(receiverPath);
            for(int i=0;i<auxiliary.Length;i++){double u=physical.GuideCoordinates[i];auxiliary[i]=anchors[i]+Vec3.ClampLength((receiverPath.AtDistance(receiverPath.Length*u).Point-reference.AtDistance(reference.Length*u).Point)*physical.GuideWeights[i],tissue.MaximumOffsetRatio*posedSize);}Solver.GuideTargets=auxiliary;
            TriangleSurface current=source.Refit(rig,preparation.SourceOverride);
            if(!ReferenceEquals(current,lastSurface))
            {
                int n=physical.Triangles.Length-physical.ConfinementTriangles.Length;var obstacle=new int[n];Array.Copy(physical.Triangles,physical.ConfinementTriangles.Length,obstacle,0,n);
                Solver.DirectionalLimit=new DirectionalContactLimit(current,source.Triangles.Length/3,obstacle){Source=body};Solver.MeshContacts=new TriangleMeshContactRefiner(physical.ConfinementTriangles,source.Triangles,current.TriangleCount){Source=body,OutwardSign=Solver.Cavity.ReceiverOutwardSign};lastSurface=current;
            }
            Solver.DirectionalLimit.Enabled=preparation.Ready||preparation.Inflation>=.01;Solver.MaximumCavityResidualPasses=RefinementPasses;
            SoftBodySolver.LocalGuardEnabled=true;SoftBodySolver.SequentialLocalGuard=true;SoftBodySolver.NormAreaProjection=true;SoftBodySolver.IncrementalMaterialGuard=true;SoftBodySolver.ContinuousMaterialSelfContact=true;SoftBodySolver.TransferTissueSelfReaction=true;
            policy.FixedStep=dt;Solver.Advance(anchors,body,tissue,policy,dt,posedSize,contact);
            transfer.Apply(Solver,field);Linear3 inverse;if(!bones[root].TryInverse(out inverse))throw new InvalidOperationException("Singular receiver carrier");
            for(int i=0;i<WorldOffsets.Length;i++){localField[i]=inverse.Multiply(field[i]);WorldOffsets[i]=field[i]+(stable[i]-actual[i])*support.GuideWeights[i];}
            MaximumSourceMovement=0;for(int i=0;i<vertices.Length;i++)MaximumSourceMovement=Math.Max(MaximumSourceMovement,(body.ToCorrectedVertex(i,vertices[i])-vertices[i]).Length);
            if(MaximumSourceMovement>contact.MaxSourceDisplacement+1e-7)throw new InvalidOperationException("Source correction exceeded its animation-relative bound");
        }
    }
}
