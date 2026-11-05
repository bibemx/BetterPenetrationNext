using System;
using System.Diagnostics;

namespace BetterPenetrationNext.Core
{
    public sealed class SoftBodySettings
    {
        // Dimensionless compliance is scaled by current posed size and h^2.
        public double StretchCompliance=1e-6,BendCompliance=2e-4,ShapeCompliance=2e-3,PinCompliance=1e-6,VolumeCompliance=1e-5;
        public double Damping=8,MaximumOffsetRatio=0.15,MaximumSpeedRatio=3,ContactMarginRatio=0.0005,ContactQueryRatio=0.5;
        // Nominal time to restore the last contact displacement. Zero preserves
        // purely elastic recovery. This rate limiter is not a calibrated material law.
        public double RecoverySeconds;
        public double MaximumAdaptiveStep;
        public Vec3 Gravity;
        public bool PreserveVolume=true;
        public void Validate()
        {
            Nonnegative(StretchCompliance); Nonnegative(BendCompliance); Nonnegative(ShapeCompliance); Nonnegative(PinCompliance); Nonnegative(VolumeCompliance); Nonnegative(Damping); Nonnegative(ContactMarginRatio); Nonnegative(RecoverySeconds);
            if(!Vec3.Finite(MaximumAdaptiveStep)||MaximumAdaptiveStep<0||MaximumAdaptiveStep>.1)throw new ArgumentException("Invalid adaptive step");
            if(!Gravity.IsFinite||!Vec3.Finite(MaximumOffsetRatio)||MaximumOffsetRatio<=0||!Vec3.Finite(MaximumSpeedRatio)||MaximumSpeedRatio<=0||!Vec3.Finite(ContactQueryRatio)||ContactQueryRatio<=0) throw new ArgumentException("Invalid motion limit/query range.");
        }
        static void Nonnegative(double value) { if(!Vec3.Finite(value)||value<0) throw new ArgumentException("Invalid nonnegative physical parameter."); }
    }
    public struct PhysicsContact { public int Vertex; public Vec3 Point,Normal; public double Penetration; }
    public sealed class SoftBodySolver
    {
        public CavityContactConstraints Cavity;public Vec3[] GuideTargets,MaterialPositions;public MaterialMetric[] MaterialMetrics;
        public bool[] GuideTargetEnabled;
        // Optional generic material-space symmetry. Contact and motion limits
        // remain authoritative; the final residual reports any disagreement.
        public BilateralDisplacementConstraint Symmetry;
        public Linear3[] SymmetryFrames;
        public double SymmetryCompliance;
        public int MaterialBaselineResets {get{return materialProjector.BaselineResets;}}
        public ReceiverContourGuides ContourGuides;
        public SourcePatchContactConstraints PatchContacts;
        public TriangleSelfCollisionGuard SelfCollision;
        public bool PreparingReceiver;
        public bool PreparationHasMatchingContactSurface;
        public TriangleMeshContactRefiner MeshContacts;
        public DirectionalContactLimit DirectionalLimit;
        public int MaximumCavityResidualPasses=4;
        public static bool SequentialLocalGuard;
        public static bool NormAreaProjection;
        public static bool IncrementalMaterialGuard;
        public static bool ContinuousMaterialSelfContact;
        public static bool TransferTissueSelfReaction;
        public TriangleMotionGuard ContinuousSelfContact {get{return materialProjector.SelfContact;}}
        readonly MaterialFeasibilityProjector materialProjector;
        readonly Vec3[] guardTargets;readonly int[] neighborEdges,faceOffsets,vertexFaces;
        public int GuardBaselineViolations,GuardLimitedVertices;
        public double MaterialGuardMilliseconds { get; private set; }
        public int MaterialGuardSweeps { get; private set; }
        public readonly SimulationTopology Topology;
        public readonly Vec3[] Positions,Velocities;
        public readonly PhysicsContact[] Contacts;
        public static bool LocalGuardEnabled; public int LocalGuardPasses { get; private set; } public int LocalGuardFallbacks { get; private set; } public int ContactCount { get; private set; }
        public int SaturatedVertices { get; private set; }
        public int LastSubsteps { get; private set; }
        public double LastStepDuration {get;private set;}
        public double LastFrameSimulationTime {get;private set;}
        public double DroppedTime { get; private set; }
        public double LastFrameDroppedTime { get; private set; }
        public double Interpolation { get; private set; }
        public int ContactQueries { get; private set; }
        public int ActiveContactCandidates { get; private set; }
        public double MaximumResidualPenetration { get; private set; }
        public double MaximumEdgeStrain { get; private set; }
        public int StrainLimitedSteps { get; private set; }
        public int ActiveContactChecks { get; private set; }
        public int ResidualQueries { get; private set; }
        public double BroadphaseMilliseconds { get; private set; }
        public double SearchMilliseconds { get; private set; }
        public double CorrectionMilliseconds { get; private set; }
        public double ResidualMilliseconds { get; private set; }
        public double AdvanceMilliseconds { get; private set; }
        readonly int[] activeContacts,triangleHints;
        int activeCount;
        bool profile;
        static double Milliseconds(long start) { return (Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency; }
        readonly int[] neighborOffsets,neighbors;
        readonly SourceContactBody[] contactSources;
        BidirectionalContactSettings coupled;
        double contactStep;
        readonly int[] contactCandidates;
        readonly Vec3[] previous,anchors,framePrevious,stepAnchors,oldStepAnchors,anchorLambda,volumeGradient,renderDisplacements,lastStepDisplacements;
        readonly Vec3[] recoveryOffsets;
        readonly double[] pin,inverseMass,stretchLambda,bendLambda,contactLambda,stretchRest,bendRest,recoveryElapsed;
        double accumulator,volumeLambda;
        bool initialized;
        bool previousStepHadContact;
        public SoftBodySolver(SimulationTopology topology,double[] pinWeights)
        {
            Topology=topology??throw new ArgumentNullException("topology"); int n=topology.VertexCount;
            materialProjector=new MaterialFeasibilityProjector(topology);
            contactCandidates=new int[n];
            activeContacts=new int[n]; triangleHints=new int[n]; for(int i=0;i<n;i++) triangleHints[i]=-1;
            if(pinWeights==null||pinWeights.Length!=n) throw new ArgumentException("Missing pin weights.");
            pin=(double[])pinWeights.Clone(); inverseMass=new double[n];
            for(int i=0;i<n;i++) { if(!Vec3.Finite(pin[i])||pin[i]<0||pin[i]>1) throw new ArgumentException("Invalid pin weight."); inverseMass[i]=pin[i]>=1-1e-8?0:1; }
            Positions=new Vec3[n]; Velocities=new Vec3[n]; previous=new Vec3[n]; anchors=new Vec3[n]; framePrevious=new Vec3[n]; stepAnchors=new Vec3[n]; oldStepAnchors=new Vec3[n];
            anchorLambda=new Vec3[n]; volumeGradient=new Vec3[n]; renderDisplacements=new Vec3[n]; lastStepDisplacements=new Vec3[n]; Contacts=new PhysicsContact[n];
            recoveryOffsets=new Vec3[n];
            recoveryElapsed=new double[n];
            contactSources=new SourceContactBody[n]; neighborOffsets=new int[n+1];
            foreach(var edge in topology.Stretch) { neighborOffsets[edge.A+1]++; neighborOffsets[edge.B+1]++; }
            for(int i=1;i<=n;i++) neighborOffsets[i]+=neighborOffsets[i-1];
            neighbors=new int[neighborOffsets[n]]; var cursor=(int[])neighborOffsets.Clone();
            neighborEdges=new int[neighbors.Length];guardTargets=new Vec3[n];int ei=0;
            foreach(var edge in topology.Stretch) { int a=cursor[edge.A]++,b=cursor[edge.B]++;neighbors[a]=edge.B;neighbors[b]=edge.A;neighborEdges[a]=neighborEdges[b]=ei++; }
            faceOffsets=new int[n+1];foreach(int v in topology.Triangles)faceOffsets[v+1]++;for(int i=1;i<=n;i++)faceOffsets[i]+=faceOffsets[i-1];vertexFaces=new int[topology.Triangles.Length];cursor=(int[])faceOffsets.Clone();for(int t=0;t<topology.Triangles.Length;t++)vertexFaces[cursor[topology.Triangles[t]]++]=t/3;
            stretchLambda=new double[topology.Stretch.Length]; bendLambda=new double[topology.Bend.Length]; contactLambda=new double[n];
            stretchRest=new double[topology.Stretch.Length]; bendRest=new double[topology.Bend.Length];
        }
        public void Reset(Vec3[] animated)
        {
            ValidateAnchors(animated);
            if(Symmetry!=null)Symmetry.Reset();
            Array.Copy(animated,Positions,animated.Length); Array.Copy(animated,anchors,animated.Length); Array.Copy(animated,stepAnchors,animated.Length); Array.Copy(animated,oldStepAnchors,animated.Length);
            Array.Copy(animated,framePrevious,animated.Length); Array.Clear(Velocities,0,Velocities.Length); Array.Clear(renderDisplacements,0,renderDisplacements.Length); Array.Clear(lastStepDisplacements,0,lastStepDisplacements.Length);
            Array.Clear(recoveryOffsets,0,recoveryOffsets.Length); previousStepHadContact=false;
            Array.Clear(recoveryElapsed,0,recoveryElapsed.Length);
            MaximumResidualPenetration=MaximumEdgeStrain=0; StrainLimitedSteps=0;
            accumulator=0; ContactCount=SaturatedVertices=LastSubsteps=0; DroppedTime=LastFrameDroppedTime=Interpolation=0; initialized=true;
            LastStepDuration=LastFrameSimulationTime=0;
        }
        void ValidateAnchors(Vec3[] value) { if(value==null||value.Length!=Positions.Length) throw new ArgumentException("Animated topology changed."); foreach(var p in value) if(!p.IsFinite) throw new ArgumentException("Non-finite animated pose."); }
        public void Advance(Vec3[] animated,IContactSurface contact,SoftBodySettings settings,PhysicsPolicy policy,double frameDt,double posedSize,BidirectionalContactSettings coupling=null)
        {
            settings.Validate(); ValidateAnchors(animated);
            if(GuideTargetEnabled!=null&&GuideTargetEnabled.Length!=Positions.Length)throw new ArgumentException("Guide target mask differs from the physical topology");
            if(Symmetry!=null&&(Symmetry.VertexCount!=Positions.Length||(SymmetryFrames!=null&&SymmetryFrames.Length!=Positions.Length)||!Vec3.Finite(SymmetryCompliance)||SymmetryCompliance<0))throw new ArgumentException("Invalid symmetry binding or compliance");
            coupled=coupling; if(coupled!=null) coupled.Validate(Positions.Length);
            profile=coupled!=null&&coupled.ProfileStages; long frameStart=profile?Stopwatch.GetTimestamp():0;
            BroadphaseMilliseconds=SearchMilliseconds=CorrectionMilliseconds=ResidualMilliseconds=AdvanceMilliseconds=0; ActiveContactChecks=ResidualQueries=0; LocalGuardPasses=LocalGuardFallbacks=0;
            MaterialGuardMilliseconds=0;MaterialGuardSweeps=0;
            if(SelfCollision!=null){SelfCollision.Milliseconds=0;SelfCollision.Calls=SelfCollision.PairTests=SelfCollision.TotalCorrections=0;}
            if(MeshContacts!=null)MeshContacts.Milliseconds=0;
            if(!Vec3.Finite(frameDt)||frameDt<0||!Vec3.Finite(posedSize)||posedSize<1e-8||policy.FixedStep<=0||policy.Iterations<1||policy.MaximumSubsteps<1) throw new ArgumentException("Invalid frame policy.");
            if(!initialized) Reset(animated);
            if(coupled!=null&&coupled.Mode==ContactMode.Original) { Reset(animated); return; }
            Array.Copy(anchors,framePrevious,anchors.Length); Array.Copy(animated,anchors,anchors.Length);
            double h=policy.FixedStep;
            accumulator+=frameDt;
            if(settings.MaximumAdaptiveStep>h&&accumulator>h*policy.MaximumSubsteps)h=Math.Min(settings.MaximumAdaptiveStep,accumulator/policy.MaximumSubsteps);
            double maxTime=h*policy.MaximumSubsteps;
            LastFrameDroppedTime=0;
            if(accumulator>maxTime) { LastFrameDroppedTime=accumulator-maxTime; DroppedTime+=LastFrameDroppedTime; accumulator=maxTime; }
            int steps=(int)Math.Floor((accumulator+1e-12)/h); LastSubsteps=steps; SaturatedVertices=0; ContactQueries=0; StrainLimitedSteps=0;
            LastStepDuration=steps>0?h:0;LastFrameSimulationTime=steps*h;
            contactStep=h; currentOffset=settings.MaximumOffsetRatio*posedSize;
            for(int substep=0;substep<steps;substep++)
            {
                double t=(double)(substep+1)/steps;
                for(int i=0;i<Positions.Length;i++)
                {
                    stepAnchors[i]=framePrevious[i]*(1-t)+anchors[i]*t;
                    // State follows the animated attachment frame. Dynamic velocity is
                    // relative to that frame; source constraints never write bones.
                    Positions[i]+=stepAnchors[i]-oldStepAnchors[i]; previous[i]=Positions[i];
                    lastStepDisplacements[i]=Positions[i]-stepAnchors[i];
                    if(inverseMass[i]==0) { Positions[i]=stepAnchors[i]; Velocities[i]=new Vec3(); }
                    else { Velocities[i]=(Velocities[i]+settings.Gravity*h)*Math.Exp(-settings.Damping*h); Positions[i]+=Velocities[i]*h; }
                    anchorLambda[i]=new Vec3(); contactLambda[i]=0; contactSources[i]=null;
                }
                if(Cavity!=null)Cavity.BeginStep();
                if(Symmetry!=null)Symmetry.BeginStep(stepAnchors,SymmetryFrames,inverseMass,h,SymmetryCompliance);
                if(PatchContacts!=null)PatchContacts.BeginStep();
                if(DirectionalLimit!=null)DirectionalLimit.BeginStep();
                Array.Clear(stretchLambda,0,stretchLambda.Length); Array.Clear(bendLambda,0,bendLambda.Length); volumeLambda=0;
                RestLengths(Topology.Stretch,stretchRest); RestLengths(Topology.Bend,bendRest);
                if(coupled!=null&&IncrementalMaterialGuard){materialProjector.EnforceContinuousSelfContact=ContinuousMaterialSelfContact;materialProjector.TransferSelfContactReaction=TransferTissueSelfReaction;materialProjector.BeginStep(previous,stepAnchors,inverseMass,stretchRest,coupled.MaxTissueStrain,currentOffset);}
                long broadStart=profile?Stopwatch.GetTimestamp():0;
                if(PatchContacts==null)BuildContactCandidates(contact,settings.MaximumOffsetRatio*posedSize+settings.ContactMarginRatio*posedSize,settings.ContactQueryRatio*posedSize);else ActiveContactCandidates=0;
                if(profile) BroadphaseMilliseconds+=Milliseconds(broadStart);
                double targetVolume=Topology.Closed?Topology.SignedVolume(stepAnchors):0;
                for(int iteration=0;iteration<policy.Iterations;iteration++)
                {
                    AnchorConstraints(settings,h,posedSize);
                    Distances(Topology.Stretch,stretchRest,stretchLambda,settings.StretchCompliance/(h*h));
                    // Opposite-vertex distance across each interior edge is a bending
                    // surrogate, not a claim of an exact dihedral/FEM material law.
                    Distances(Topology.Bend,bendRest,bendLambda,settings.BendCompliance/(h*h));
                    if(Topology.Closed&&settings.PreserveVolume) Volume(targetVolume,settings.VolumeCompliance*Math.Pow(posedSize,4)/(h*h));
                    if(GuideTargets!=null){double alpha=1e-6/(h*h);for(int i=0;i<Positions.Length;i++)if(inverseMass[i]>0&&(GuideTargetEnabled==null||GuideTargetEnabled[i]))Positions[i]+=(GuideTargets[i]-Positions[i])/(1+alpha);}
                    if(ContourGuides!=null)ContourGuides.Project(Positions,inverseMass,h);
                    if(DirectionalLimit!=null)DirectionalLimit.Project(Positions);
                    if(Symmetry!=null)Symmetry.Project(Positions);
                    if(PatchContacts==null){if(!PreparingReceiver||PreparationHasMatchingContactSurface)ContactsConstraint(contact,settings.ContactMarginRatio*posedSize,settings.ContactQueryRatio*posedSize);}else{PatchContacts.Project(Positions,stepAnchors,inverseMass,currentOffset,coupled,h);ContactCount=PatchContacts.ActiveContacts;}
                    if(Cavity!=null)Cavity.Project(Positions,stepAnchors,inverseMass,currentOffset,coupled,h);
                    Limits(settings.MaximumOffsetRatio*posedSize);
                    if(coupled!=null) EnforceStrain();
                }
                Recovery(settings.RecoverySeconds,h+LastFrameDroppedTime/steps,posedSize);
                double speed=settings.MaximumSpeedRatio*posedSize;
                for(int i=0;i<Positions.Length;i++)
                {
                    if(inverseMass[i]>0) Positions[i]=previous[i]+Vec3.ClampLength(Positions[i]-previous[i],speed*h);
                    Velocities[i]=inverseMass[i]==0?new Vec3():(Positions[i]-previous[i])/h;
                    if(coupled!=null&&contactLambda[i]<=1e-15&&recoveryElapsed[i]>0) Velocities[i]=new Vec3();
                    oldStepAnchors[i]=stepAnchors[i];
                }
                if(coupled!=null) { EnforceStrain();if(SelfCollision!=null&&SelfCollision.Resolve(Positions,previous,stepAnchors)>0)EnforceStrain();if(PatchContacts!=null){for(int pass=0;pass<=MaximumCavityResidualPasses;pass++){if(PatchContacts.RefreshResidual(Positions)<=.0001||pass==MaximumCavityResidualPasses)break;PatchContacts.Project(Positions,stepAnchors,inverseMass,currentOffset,coupled,h);Limits(currentOffset);EnforceStrain();if(SelfCollision!=null&&SelfCollision.Resolve(Positions,previous,stepAnchors)>0)EnforceStrain();}MaximumResidualPenetration=PatchContacts.MaximumViolation;}else{if(Cavity!=null&&Cavity.UseCachedContacts){for(int pass=0;pass<=MaximumCavityResidualPasses;pass++){if(ContourGuides!=null&&ContourGuides.EntryTranslationSource!=null)ContourGuides.Project(Positions,inverseMass,h);if(Cavity.RefreshResidual(Positions)<=.0001||pass==MaximumCavityResidualPasses)break;Cavity.Project(Positions,stepAnchors,inverseMass,currentOffset,coupled,h);Limits(currentOffset);EnforceStrain();if(SelfCollision!=null&&SelfCollision.Resolve(Positions,previous,stepAnchors)>0)EnforceStrain();}}if(MeshContacts!=null&&!PreparingReceiver){if(Cavity!=null)MeshContacts.OutwardSign=Cavity.ReceiverOutwardSign;for(int pass=0;pass<8;pass++){int crossing=MeshContacts.Project(Positions,stepAnchors,inverseMass,currentOffset,coupled,h);if(ContinuousMaterialSelfContact&&IncrementalMaterialGuard)EnforceStrain();int self=SelfCollision==null?0:SelfCollision.Resolve(Positions,previous,stepAnchors);if(crossing==0&&self==0)break;EnforceStrain();}}MeasureResidual(contact,settings.ContactMarginRatio*posedSize,settings.ContactQueryRatio*posedSize);} }
                if(SelfCollision!=null){for(int pass=0;pass<24;pass++){if(ContourGuides!=null&&ContourGuides.EntryTranslationSource!=null)ContourGuides.Project(Positions,inverseMass,h);EnforceStrain();if(SelfCollision.Resolve(Positions,previous,stepAnchors)==0)break;}}
                // The final self-contact correction is not automatically feasible
                // for incident material faces. Never return it without a guard.
                if(coupled!=null)EnforceStrain();
                if(DirectionalLimit!=null){for(int pass=0;pass<4;pass++){if(!DirectionalLimit.Project(Positions))break;if(ContourGuides!=null&&ContourGuides.EntryTranslationSource!=null)ContourGuides.Project(Positions,inverseMass,h);EnforceStrain();}}
                for(int i=0;i<Positions.Length;i++)Velocities[i]=inverseMass[i]==0||(coupled!=null&&contactLambda[i]<=1e-15&&recoveryElapsed[i]>0)?new Vec3():Vec3.ClampLength((Positions[i]-previous[i])/h,speed);
                if(Symmetry!=null)Symmetry.MeasureResidual(Positions);
                accumulator-=h;
            }
            Interpolation=Math.Max(0,Math.Min(1,accumulator/h));
            // Render anchored relative displacements. Hard pins match the current
            // animated pose even on frames without a physics step.
            for(int i=0;i<Positions.Length;i++) renderDisplacements[i]=inverseMass[i]==0?new Vec3():coupled!=null?Positions[i]-oldStepAnchors[i]:lastStepDisplacements[i]*(1-Interpolation)+(Positions[i]-oldStepAnchors[i])*Interpolation;
            if(profile) AdvanceMilliseconds=Milliseconds(frameStart);
        }
        void Recovery(double seconds,double elapsed,double size)
        {
            if(GuideTargets!=null||ContourGuides!=null||(Cavity!=null&&Cavity.Count>0)||(PatchContacts!=null&&PatchContacts.ActiveContacts>0)){previousStepHadContact=true;return;}
            if(coupled!=null&&coupled.RecoveryTime>0) {
                bool anyContact=ContactCount>0;
                for(int i=0;i<Positions.Length;i++) {
                    if(inverseMass[i]==0) continue;
                    if(previousStepHadContact&&!anyContact) { recoveryOffsets[i]=previous[i]-stepAnchors[i]; recoveryElapsed[i]=0; }
                    if(contactLambda[i]>1e-15) { recoveryOffsets[i]=Positions[i]-stepAnchors[i]; recoveryElapsed[i]=0; }
                    else if(recoveryOffsets[i].LengthSquared>0) {
                        recoveryElapsed[i]=Math.Min(coupled.RecoveryTime,recoveryElapsed[i]+elapsed);
                        double t=recoveryElapsed[i]/coupled.RecoveryTime,weight=1-t*t*(3-2*t);
                        Positions[i]=stepAnchors[i]+recoveryOffsets[i]*weight;
                        if(t>=1) { recoveryOffsets[i]=new Vec3(); Velocities[i]=new Vec3(); }
                    }
                }
                previousStepHadContact=anyContact;
                return;
            }
            bool contact=ContactCount>0;
            for(int i=0;i<Positions.Length;i++)
            {
                if(inverseMass[i]==0||seconds<=0) { recoveryOffsets[i]=new Vec3(); continue; }
                if(contact)
                {
                    Vec3 displacement=Positions[i]-stepAnchors[i];
                    if(!previousStepHadContact||displacement.LengthSquared>recoveryOffsets[i].LengthSquared) recoveryOffsets[i]=displacement;
                    continue;
                }
                Vec3 before=previous[i]-stepAnchors[i],after=Positions[i]-stepAnchors[i],delta=after-before;
                // Only slow restorative motion after contact has ended. New
                // contacts, animated attachments and hard pins remain immediate.
                if(recoveryOffsets[i].LengthSquared>0&&after.LengthSquared<before.LengthSquared&&Vec3.Dot(delta,before)<0)
                    Positions[i]=previous[i]+Vec3.ClampLength(delta,recoveryOffsets[i].Length*elapsed/seconds);
                if((Positions[i]-stepAnchors[i]).LengthSquared<size*size*1e-16) recoveryOffsets[i]=new Vec3();
            }
            previousStepHadContact=contact;
        }
        void RestLengths(EdgeConstraint[] edges,double[] rest) { for(int i=0;i<edges.Length;i++){int a=edges[i].A,b=edges[i].B;rest[i]=(stepAnchors[a]-stepAnchors[b]).Length;if(edges==Topology.Stretch&&MaterialPositions!=null&&MaterialMetrics!=null){Vec3 edge=MaterialPositions[a]-MaterialPositions[b];rest[i]=Math.Max(rest[i],Math.Sqrt((MaterialMetrics[a].SquaredLength(edge)+MaterialMetrics[b].SquaredLength(edge))*.5));}} }
        void Distances(EdgeConstraint[] edges,double[] rest,double[] multipliers,double alpha)
        {
            for(int i=0;i<edges.Length;i++)
            {
                int a=edges[i].A,b=edges[i].B; Vec3 d=Positions[a]-Positions[b]; double length=d.Length,denom=inverseMass[a]+inverseMass[b]+alpha;
                if(length<1e-15||denom<1e-30) continue;
                double dl=(-(length-rest[i])-alpha*multipliers[i])/denom;if(edges==Topology.Stretch&&MaterialPositions!=null){double next=Math.Min(0,multipliers[i]+dl);dl=next-multipliers[i];} multipliers[i]+=dl; Vec3 correction=d*(dl/length);
                Positions[a]+=correction*inverseMass[a]; Positions[b]-=correction*inverseMass[b];
            }
        }
        void AnchorConstraints(SoftBodySettings s,double h,double size)
        {
            for(int i=0;i<Positions.Length;i++)
            {
                if(inverseMass[i]==0) { Positions[i]=stepAnchors[i]; continue; }
                double compliance=pin[i]>0?s.PinCompliance*(1-pin[i])/Math.Max(pin[i],1e-8):s.ShapeCompliance;
                double alpha=compliance/(h*h); Vec3 delta=(-(Positions[i]-stepAnchors[i])-anchorLambda[i]*alpha)/(inverseMass[i]+alpha);
                anchorLambda[i]+=delta; Positions[i]+=delta*inverseMass[i];
            }
        }
        void Volume(double rest,double alpha)
        {
            Array.Clear(volumeGradient,0,volumeGradient.Length); double volume=0; Vec3 origin=stepAnchors[0];
            for(int t=0;t<Topology.Triangles.Length;t+=3)
            {
                int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2]; Vec3 p=Positions[a]-origin,q=Positions[b]-origin,r=Positions[c]-origin;
                volume+=Vec3.Dot(p,Vec3.Cross(q,r))/6;
                volumeGradient[a]+=Vec3.Cross(q,r)/6; volumeGradient[b]+=Vec3.Cross(r,p)/6; volumeGradient[c]+=Vec3.Cross(p,q)/6;
            }
            double denom=alpha; for(int i=0;i<Positions.Length;i++) denom+=inverseMass[i]*volumeGradient[i].LengthSquared;
            if(denom<1e-30) return; double dl=(-(volume-rest)-alpha*volumeLambda)/denom; volumeLambda+=dl;
            for(int i=0;i<Positions.Length;i++) Positions[i]+=volumeGradient[i]*(inverseMass[i]*dl);
        }
        void ContactsConstraint(IContactSurface surface,double margin,double query)
        {
            ContactCount=0; if(surface==null) return;
            if(coupled!=null) { CoupledContacts(surface,margin,query); return; }
            var bounded=surface as IContactBounds; bool useBounds=bounded!=null&&bounded.HasVolumeBounds; Bounds3 bounds=useBounds?bounded.ContactBounds:new Bounds3();
            for(int active=0;active<ActiveContactCandidates;active++)
            {
                int i=contactCandidates[active];
                if(useBounds&&contactLambda[i]<=0&&bounds.DistanceSquared(Positions[i])>margin*margin) continue;
                ContactQueries++;
                SurfaceSample sample; if(!surface.TrySample(Positions[i],query,out sample)) continue;
                double c=sample.SignedDistance-margin;
                if(c>=0&&contactLambda[i]<=0) continue;
                double dl=-c/inverseMass[i],next=Math.Max(0,contactLambda[i]+dl); dl=next-contactLambda[i]; contactLambda[i]=next;
                Positions[i]+=sample.Normal*(inverseMass[i]*dl);
                if(next>1e-15) Contacts[ContactCount++]=new PhysicsContact { Vertex=i,Point=sample.Point,Normal=sample.Normal,Penetration=Math.Max(0,-c) };
            }
        }
        void CoupledContacts(IContactSurface surface,double margin,double query)
        {
            var contacts=surface as ICoupledContactSurface;
            var bounded=surface as IContactBounds; bool local=bounded!=null&&bounded.HasVolumeBounds;
            var hinted=surface as IHintedCoupledSurface;
            int checkedCount=0;
            // Small ordered blocks preserve the full sweep's Gauss-Seidel order.
            // If the shared source moves, reselect only the unprocessed tail of
            // this block, so a newly entered contact is never silently skipped.
            const int blockSize=32;
            for(int first=0;first<ActiveContactCandidates;first+=blockSize) {
                int end=Math.Min(first+blockSize,ActiveContactCandidates);
                FillActiveContacts(first,end,bounded,local,margin,query);
                for(int active=0;active<activeCount;active++) {
                    int candidate=activeContacts[active],i=contactCandidates[candidate];checkedCount++;
                    if(ProjectContact(surface,contacts,hinted,bounded,local,i,margin,query)&&coupled.UseActiveContactList) {
                        FillActiveContacts(candidate+1,end,bounded,local,margin,query);active=-1;
                    }
                }
            }
            ActiveContactChecks=Math.Max(ActiveContactChecks,checkedCount);
        }
        void FillActiveContacts(int first,int end,IContactBounds bounded,bool local,double margin,double query)
        {
            long broadStart=profile?Stopwatch.GetTimestamp():0;
            activeCount=0;
            if(coupled.UseActiveContactList) {
                var bounds=bounded==null?new Bounds3():bounded.ContactBounds; double pad=local?margin:query;
                for(int k=first;k<end;k++) { int i=contactCandidates[k]; if(bounded==null||contactLambda[i]>0||bounds.DistanceSquared(Positions[i])<=pad*pad) activeContacts[activeCount++]=k; }
            } else for(int k=first;k<end;k++)activeContacts[activeCount++]=k;
            if(profile) BroadphaseMilliseconds+=Milliseconds(broadStart);
        }
        bool ProjectContact(IContactSurface surface,ICoupledContactSurface contacts,IHintedCoupledSurface hinted,IContactBounds bounded,bool local,int i,double margin,double query)
        {
                SourceContactBody source=null; SurfaceSample sample;bool moved=false;
                // Read current corrected bounds: another contact may have moved the
                // shared source since the previous particle was projected.
                if(local&&contactLambda[i]<=0&&bounded.ContactBounds.DistanceSquared(Positions[i])>margin*margin) return false;
                ContactQueries++;
                long searchStart=profile?Stopwatch.GetTimestamp():0;
                bool found=coupled.UseTriangleHints&&hinted!=null?hinted.TryContactHinted(Positions[i],query,ref triangleHints[i],out sample,out source):contacts!=null?contacts.TryContact(Positions[i],query,out sample,out source):surface.TrySample(Positions[i],query,out sample);
                if(profile) SearchMilliseconds+=Milliseconds(searchStart);
                if(!found) return false;
                long correctionStart=profile?Stopwatch.GetTimestamp():0;
                if(source!=contactSources[i]) { contactLambda[i]=0; contactSources[i]=source; }
                double gap=sample.SignedDistance-margin; if(gap>=0&&contactLambda[i]<=0) return false;
                Vec3 n=sample.Normal.Normalized; if(n.LengthSquared<.5) return false;
                double wr=inverseMass[i]*coupled.TissueDeformationStrength;
                Vec3 sourceGradient=source==null?n:source.ProjectContactGradient(n);
                double sourceProjection=sourceGradient.Length;
                Vec3 sourceDirection=sourceProjection>1e-12?-sourceGradient/sourceProjection:new Vec3();
                double ws=source==null||coupled.Mode!=ContactMode.Bidirectional?0:source.InverseMass*sourceProjection*sourceProjection;
                double compliance=coupled.RegionCompliance==null?coupled.TissueCompliance:coupled.RegionCompliance[i];
                double alpha=compliance/(contactStep*contactStep);
                double next=BidirectionalContactSolver.Multiplier(gap,contactLambda[i],wr,ws,alpha),dl=next-contactLambda[i]; contactLambda[i]=next;
                if(dl>0) {
                    double travel=TissueTravel(i,n),receiver=Math.Min(travel,wr*dl);
                    double sourceMove=ws*dl;
                    // Transfer blocked tissue correction to the shared source DOF.
                    if(ws>0) sourceMove+=Math.Max(0,wr*dl-receiver);
                    sourceMove=source==null||sourceProjection<1e-12?0:Math.Min(source.Travel(sourceDirection)*sourceProjection,sourceMove);
                    if(wr>0) receiver=Math.Min(travel,Math.Max(receiver,(wr+ws)*dl-sourceMove));
                    Positions[i]+=n*receiver; if(source!=null&&sourceMove>0) {Vec3 old=source.Offset;source.Correct(sourceDirection*(sourceMove/sourceProjection));moved=(source.Offset-old).LengthSquared>0;}
                }
                // Negative multiplier changes release pressure; animated shape tethers
                // restore tissue. They never attract a separated source to the surface.
                if(next>1e-15) { if(source!=null) source.MarkContact(); Contacts[ContactCount++]=new PhysicsContact { Vertex=i,Point=sample.Point,Normal=n,Penetration=Math.Max(0,-gap) }; }
                if(profile) CorrectionMilliseconds+=Milliseconds(correctionStart);
                return moved;
        }
        double TissueTravel(int i,Vec3 normal)
        {
            if(inverseMass[i]==0||coupled.TissueDeformationStrength==0) return 0;
            double travel=BidirectionalContactSolver.RayLimit(Positions[i]-stepAnchors[i],normal,currentOffset);
            if(LocalGuardEnabled)return travel;
            for(int j=neighborOffsets[i];j<neighborOffsets[i+1];j++) {
                int other=neighbors[j]; double length=(stepAnchors[i]-stepAnchors[other]).Length*(1+coupled.MaxTissueStrain);
                travel=Math.Min(travel,BidirectionalContactSolver.RayLimit(Positions[i]-Positions[other],normal,length));
            }
            return travel;
        }
        double currentOffset;
        void EnforceStrain()
        {
            long started=profile?Stopwatch.GetTimestamp():0;
            EnforceStrainCore();
            if(profile)MaterialGuardMilliseconds+=Milliseconds(started);
        }
        void EnforceStrainCore()
        {
            if(!LocalGuardEnabled) { EnforceGlobalStrain(); return; }
            LocalGuardPasses++;
            if(IncrementalMaterialGuard){materialProjector.Project(Positions);MaterialGuardSweeps+=materialProjector.LastSweeps;MaximumEdgeStrain=materialProjector.MaximumStrain;return;}
            // Experimental local feasibility projection: constraints are solved
            // only on their incident particles, rather than scaling the whole
            // receiver when one triangle approaches its reference hemisphere.
            for(int sweep=0;sweep<(NormAreaProjection?192:16);sweep++) {
                MaterialGuardSweeps++;
                for(int edgeIndex=0;edgeIndex<Topology.Stretch.Length;edgeIndex++) {var edge=Topology.Stretch[edgeIndex];
                    int a=edge.A,b=edge.B;Vec3 delta=Positions[b]-Positions[a];double length=delta.Length;
                    double maximum=stretchRest[edgeIndex]*(1+coupled.MaxTissueStrain)*.999;
                    double wa=inverseMass[a],wb=inverseMass[b],denominator=wa+wb;
                    if(length<=maximum||length<1e-20||denominator<=0)continue;
                    Vec3 correction=delta*((length-maximum)/(length*denominator));Positions[a]+=correction*wa;Positions[b]-=correction*wb;
                }
                for(int t=0;t<Topology.Triangles.Length;t+=3) {
                    int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];
                    Vec3 normal=NormAreaProjection?Vec3.Cross(Positions[b]-Positions[a],Positions[c]-Positions[a]):Vec3.Cross(previous[b]-previous[a],previous[c]-previous[a]);double oldArea=normal.Length;double referenceArea=Vec3.Cross(stepAnchors[b]-stepAnchors[a],stepAnchors[c]-stepAnchors[a]).Length;
                    if(referenceArea<1e-24||oldArea<1e-24)continue;normal/=oldArea;
                    Vec3 e1=Positions[b]-Positions[a],e2=Positions[c]-Positions[a];double area=Vec3.Dot(Vec3.Cross(e1,e2),normal),target=referenceArea*.02;
                    if(area>=target)continue;
                    Vec3 ga=Vec3.Cross(normal,e2-e1),gb=Vec3.Cross(e2,normal),gc=Vec3.Cross(normal,e1);
                    double wa=inverseMass[a],wb=inverseMass[b],wc=inverseMass[c],denominator=wa*ga.LengthSquared+wb*gb.LengthSquared+wc*gc.LengthSquared;
                    if(denominator<1e-30)continue;
                    double multiplier=(target-area)/denominator;
                    Positions[a]+=ga*(wa*multiplier);Positions[b]+=gb*(wb*multiplier);Positions[c]+=gc*(wc*multiplier);
                }
                Limits(currentOffset);
                if(NormAreaProjection){bool feasible=true;for(int e=0;e<Topology.Stretch.Length;e++){var edge=Topology.Stretch[e];if((Positions[edge.A]-Positions[edge.B]).Length>stretchRest[e]*(1+coupled.MaxTissueStrain)){feasible=false;break;}}if(feasible){for(int t=0;t<Topology.Triangles.Length;t+=3){int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];if(Vec3.Cross(Positions[b]-Positions[a],Positions[c]-Positions[a]).Length<Vec3.Cross(stepAnchors[b]-stepAnchors[a],stepAnchors[c]-stepAnchors[a]).Length*.010001){feasible=false;break;}}}if(feasible)break;}
            }
            bool fallback=false;MaximumEdgeStrain=0;
            for(int edgeIndex=0;edgeIndex<Topology.Stretch.Length;edgeIndex++) {var edge=Topology.Stretch[edgeIndex];
                double reference=stretchRest[edgeIndex];
                if(reference<=1e-15)continue;double strain=(Positions[edge.A]-Positions[edge.B]).Length/reference-1;
                MaximumEdgeStrain=Math.Max(MaximumEdgeStrain,strain);if(strain>coupled.MaxTissueStrain+1e-9)fallback=true;
            }
            for(int t=0;!NormAreaProjection&&t<Topology.Triangles.Length;t+=3) {
                int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];Vec3 normal=Vec3.Cross(previous[b]-previous[a],previous[c]-previous[a]);
                if(normal.LengthSquared>1e-48&&Vec3.Dot(Vec3.Cross(Positions[b]-Positions[a],Positions[c]-Positions[a]),normal)<=normal.LengthSquared*.010000001)fallback=true;
            }
            if(NormAreaProjection)for(int t=0;t<Topology.Triangles.Length;t+=3){int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];double reference=Vec3.Cross(stepAnchors[b]-stepAnchors[a],stepAnchors[c]-stepAnchors[a]).Length;if(reference>1e-24&&Vec3.Cross(Positions[b]-Positions[a],Positions[c]-Positions[a]).Length<reference*.010001){fallback=true;break;}}
            if(fallback) { LocalGuardFallbacks++;if(SequentialLocalGuard)EnforceSequentialStrain();else EnforceGlobalStrain(); }
        }
        void EnforceSequentialStrain()
        {
            Array.Copy(Positions,guardTargets,Positions.Length);Array.Copy(previous,Positions,Positions.Length);
            // A changed pose/scale can reduce the allowed displacement sphere.
            // Previous-frame feasibility does not prove feasibility in this sphere.
            for(int i=0;i<Positions.Length;i++)Positions[i]=stepAnchors[i]+Vec3.ClampLength(Positions[i]-stepAnchors[i],currentOffset);
            bool bad=false;for(int e=0;e<Topology.Stretch.Length;e++){var edge=Topology.Stretch[e];if((Positions[edge.A]-Positions[edge.B]).Length>stretchRest[e]*(1+coupled.MaxTissueStrain)+1e-9){bad=true;GuardBaselineViolations++;}}
            if(NormAreaProjection)for(int t=0;t<Topology.Triangles.Length;t+=3){int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];double reference=Vec3.Cross(stepAnchors[b]-stepAnchors[a],stepAnchors[c]-stepAnchors[a]).Length;if(reference>1e-24&&Vec3.Cross(Positions[b]-Positions[a],Positions[c]-Positions[a]).Length<reference*.010001){bad=true;GuardBaselineViolations++;}}
            if(bad)Array.Copy(stepAnchors,Positions,Positions.Length);
            for(int sweep=0;sweep<12;sweep++)for(int k=0;k<Positions.Length;k++){
                int i=sweep%2==0?k:Positions.Length-1-k;if(inverseMass[i]==0)continue;Vec3 delta=guardTargets[i]-Positions[i];double length=delta.Length;if(length<1e-12)continue;Vec3 direction=delta/length;double travel=length;
                travel=Math.Min(travel,BidirectionalContactSolver.RayLimit(Positions[i]-stepAnchors[i],direction,currentOffset));
                for(int j=neighborOffsets[i];j<neighborOffsets[i+1];j++)travel=Math.Min(travel,BidirectionalContactSolver.RayLimit(Positions[i]-Positions[neighbors[j]],direction,stretchRest[neighborEdges[j]]*(1+coupled.MaxTissueStrain)));
                for(int j=faceOffsets[i];j<faceOffsets[i+1];j++){
                    int t=vertexFaces[j]*3,a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];Vec3 e1=Positions[b]-Positions[a],e2=Positions[c]-Positions[a],n=Vec3.Cross(e1,e2);double area=n.Length;if(area<1e-24)continue;n/=area;
                    double minimum=Vec3.Cross(stepAnchors[b]-stepAnchors[a],stepAnchors[c]-stepAnchors[a]).Length*.010001;
                    Vec3 gradient=i==a?Vec3.Cross(n,e2-e1):i==b?Vec3.Cross(e2,n):Vec3.Cross(n,e1);double slope=Vec3.Dot(gradient,direction);if(slope<0){double gap=area-Math.Max(minimum,area*.01);travel=Math.Min(travel,Math.Max(0,gap/-slope));}
                }
                if(travel<length-1e-9)GuardLimitedVertices++;Positions[i]+=direction*Math.Max(0,travel*(travel<length?.999999:1));
            }
            MaximumEdgeStrain=0;for(int e=0;e<Topology.Stretch.Length;e++){var edge=Topology.Stretch[e];if(stretchRest[e]>1e-15)MaximumEdgeStrain=Math.Max(MaximumEdgeStrain,(Positions[edge.A]-Positions[edge.B]).Length/stretchRest[e]-1);}
        }
        void EnforceGlobalStrain()
        {
            double scale=1;
            foreach(var edge in Topology.Stretch) {
                Vec3 rest=stepAnchors[edge.A]-stepAnchors[edge.B],delta=Positions[edge.A]-Positions[edge.B]-rest;
                if(delta.LengthSquared<1e-30) continue;
                double allowed=BidirectionalContactSolver.RayLimit(rest,delta.Normalized,rest.Length*(1+coupled.MaxTissueStrain));
                scale=Math.Min(scale,allowed/delta.Length);
            }
            // Keep every triangle in the same local hemisphere as its animated
            // counterpart. This is a local fold guard, not a self-collision solver.
            for(int t=0;t<Topology.Triangles.Length;t+=3) {
                int a=Topology.Triangles[t],b=Topology.Triangles[t+1],c=Topology.Triangles[t+2];
                Vec3 normal=Vec3.Cross(stepAnchors[b]-stepAnchors[a],stepAnchors[c]-stepAnchors[a]);
                double lo=0,hi=Math.Min(1,scale);
                for(int k=0;k<12;k++) {
                    Vec3 pa=stepAnchors[a]+(Positions[a]-stepAnchors[a])*hi,pb=stepAnchors[b]+(Positions[b]-stepAnchors[b])*hi,pc=stepAnchors[c]+(Positions[c]-stepAnchors[c])*hi;
                    if(Vec3.Dot(Vec3.Cross(pb-pa,pc-pa),normal)>normal.LengthSquared*.01) { lo=hi; break; }
                    hi=(lo+hi)*.5;
                }
                scale=Math.Min(scale,lo);
            }
            if(scale<1) { StrainLimitedSteps++; for(int i=0;i<Positions.Length;i++) Positions[i]=stepAnchors[i]+(Positions[i]-stepAnchors[i])*Math.Max(0,scale); }
            MaximumEdgeStrain=0;
            foreach(var edge in Topology.Stretch) { double rest=(stepAnchors[edge.A]-stepAnchors[edge.B]).Length; if(rest>1e-15) MaximumEdgeStrain=Math.Max(MaximumEdgeStrain,(Positions[edge.A]-Positions[edge.B]).Length/rest-1); }
        }
        void MeasureResidual(IContactSurface surface,double margin,double query)
        {
            MaximumResidualPenetration=0; if(surface==null) return;
            long started=profile?Stopwatch.GetTimestamp():0;
            var bounded=surface as IContactBounds; bool local=bounded!=null&&bounded.HasVolumeBounds; var bounds=bounded==null?new Bounds3():bounded.ContactBounds; double pad=local?margin:query;
            var hinted=surface as IHintedCoupledSurface;
            for(int active=0;active<ActiveContactCandidates;active++) { int i=contactCandidates[active]; var p=Positions[i]; if(bounded!=null&&(local||coupled.UseActiveContactList)&&bounds.DistanceSquared(p)>pad*pad) continue;
                SurfaceSample sample; SourceContactBody source; ResidualQueries++;
                bool found=coupled.UseTriangleHints&&hinted!=null?hinted.TryContactHinted(p,query,ref triangleHints[i],out sample,out source):surface.TrySample(p,query,out sample);
                if(found) MaximumResidualPenetration=Math.Max(MaximumResidualPenetration,margin-sample.SignedDistance);
            }
            if(profile) ResidualMilliseconds+=Milliseconds(started);
        }
        void BuildContactCandidates(IContactSurface contact,double reach,double query)
        {
            ActiveContactCandidates=0; if(contact==null) return;
            var bounded=contact as IContactBounds; bool local=bounded!=null&&bounded.HasVolumeBounds; Bounds3 bounds=local?bounded.ContactBounds:new Bounds3();
            // A moving source can enter a new patch during the solve. Include its
            // full permitted correction in the broad phase and hard pins for reaction.
            if(coupled!=null) {
                reach+=2*coupled.MaxSourceDisplacement;
                if(coupled.UseActiveContactList&&bounded!=null&&!local) {local=true;bounds=bounded.ContactBounds;reach+=query;}
            }
            for(int i=0;i<Positions.Length;i++) if((inverseMass[i]>0||coupled!=null)&&(!local||bounds.DistanceSquared(stepAnchors[i])<=reach*reach)) contactCandidates[ActiveContactCandidates++]=i;
        }
        void Limits(double offset)
        {
            currentOffset=offset;
            SaturatedVertices=0;
            for(int i=0;i<Positions.Length;i++)
            {
                Vec3 d=Positions[i]-stepAnchors[i];
                if(d.LengthSquared>=offset*offset*0.998001) SaturatedVertices++;
                if(d.LengthSquared>offset*offset) Positions[i]=stepAnchors[i]+Vec3.ClampLength(d,offset);
                if(!Positions[i].IsFinite) throw new InvalidOperationException("Physics diverged; restore Original.");
            }
        }
        public Vec3 Displacement(int vertex) { return renderDisplacements[vertex]; }
        public Vec3 SolvedDisplacement(int vertex) { return Positions[vertex]-oldStepAnchors[vertex]; }
        public void Transport(int vertex,Linear3 previousFrame,Linear3 currentFrame)
        {
            Vec3 local;
            if(previousFrame.TryInverseMultiply(Positions[vertex]-oldStepAnchors[vertex],out local)) Positions[vertex]=oldStepAnchors[vertex]+currentFrame.Multiply(local);
            if(previousFrame.TryInverseMultiply(Velocities[vertex],out local)) Velocities[vertex]=currentFrame.Multiply(local);
            if(previousFrame.TryInverseMultiply(lastStepDisplacements[vertex],out local)) lastStepDisplacements[vertex]=currentFrame.Multiply(local);
            if(previousFrame.TryInverseMultiply(recoveryOffsets[vertex],out local)) recoveryOffsets[vertex]=currentFrame.Multiply(local);
        }
    }
}
