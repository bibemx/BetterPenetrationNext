using System;
using System.Diagnostics;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    [Serializable]public sealed class ReactiveMaterialChart {public double[] Axial,AngleTurns,ReferenceArc,ReferenceRadius,SupportAxial,SupportAngle,FrameTwist;public double AngularDirection=1;public int[] Triangles,Regions,TriangleRegions;}
    // A reduced material rig. Original skin identities and triangles are never
    // welded or edited. Only section frames and radial expansion are solved.
    public sealed class ReactiveRigInput
    {
        public readonly Linear3[] Bones; public readonly Vec3[] Translation,Source;
        public readonly double[] Morph;
        public Vec3 SourceRoot; public Linear3 SourceFrame; public int Frame; public double Dt,PosedSize;
        public bool SourceFollowsPath,HasSource=true,WholeSource=true;
        public double CervixMaxDisplacement=.025,CervixFollowStrength=1,CanalLengthScale=1;
        public double MaxSource=.12,MaxStrain=4,SourceCompliance=.35,TissueStrength=1,Compliance=1e-7,Response=12,Recovery=4.5,MaximumOffsetRatio=1.5;
        public ReactiveRigInput(int bones,int morphs,int source){Bones=new Linear3[bones];Translation=new Vec3[bones];Morph=new double[morphs];Source=new Vec3[source];}
    }
    public sealed class ReactiveRigOutput
    {
        public readonly Vec3[] Offsets,FullWorld,Physical,SourceOffsets,SourceWorld,Path,LandmarkPoints;
        public int LandmarkCount,ReceiverLandmarkCount;public bool ReceiverActive,EntryContact,SourceAdmitted;public double ReceiverSpacing,CanalExtension;
        public readonly Vec3[] ReceiverLandmarkPoints=new Vec3[ReceiverAdaptiveLandmarks.Capacity*5],ReceiverSourcePairs=new Vec3[ReceiverAdaptiveLandmarks.Capacity],ExternalLandmarkPoints=new Vec3[5];
        public int Frame,Contacts,SectionTests,TriangleTests,ActiveSections,ContainedVertices,EngagedVertices,SourceVertices,WorkerThread,LimitingEdge;
        public double CaptureMs,PathMs,ContactMs,TransferMs,TotalMs,MaximumStrain,MaximumPrimaryStrain,MaterialBlend,MaxSourceMovement,MaximumResidual,EntranceRadius,RequiredEntranceRadius,RecoveryFraction;
        public Linear3 Carrier,SourceFrame; public Vec3 CarrierTranslation;
        public readonly Vec3[] SourceAnimatedCentres=new Vec3[25],SourceGuidedCentres=new Vec3[25],SourceSolvedCentres=new Vec3[25];
        public double SourceBudgetBlend,SourceMaximumTurn,CervicalPenetration,CervicalResidual;public Vec3 CervicalDisplacement;public int CervicalQueries;
        public Vec3 CervicalMotionAxis,GuidanceCervicalDisplacement;public int CervicalReactionAttempts,CervicalReactionRejected;
        public double SourceArcLength,TerminalRootDistance,SourceGuideWeight;public bool TerminalWithinSourceLength;public int MaterialTipVertex;
        public int MaterialEdgeCount,MaterialFaceCount;
        public int EntranceRegularizedVertices,EntranceSymmetryPairs,EntranceMidlineVertices;public double EntranceSymmetryResidual,EntranceMidlineResidual;
        public double CervicalTipPressure,EffectiveCervicalTravel;
        public double NativeCanalArc,ConfiguredCanalArc;public Vec3 CanalLengthDisplacement;
        public Vec3 UterusRotation,UterusRotationPelvis,UterusTranslationPelvis,UterusPelvisUp,GuidanceCervicalRotationPelvis;
        public double CervicalMaterialBlend,CervicalContactTolerance;public bool TerminalCanalFacing;public int TerminalSurfacePoints;
        public bool TerminalAvailable;public Vec3 TerminalCentre,TerminalNormal;public double TerminalRadius,TerminalTipAxial,TerminalTipLateral,TerminalTipDistance,ContactCanalExtension;
        public readonly Vec3[] TerminalPoints=new Vec3[5];
        public ReactiveRigOutput(int support,int physical,int source,int sections){Offsets=new Vec3[support];FullWorld=new Vec3[support];Physical=new Vec3[physical];SourceOffsets=new Vec3[source];SourceWorld=new Vec3[source];Path=new Vec3[sections];LandmarkPoints=new Vec3[Math.Max(700,checked(2+33*17+sections*5))];}
    }
    public sealed class ReactiveReceiverRig
    {
        readonly SimulationDefinition physical,support; readonly ReceiverSupportTransfer transfer; readonly ReceiverGuideDefinition guide;
        readonly CervicalSurfaceContact cervix;readonly CervicalOpeningLandmark terminal;Vec3 cervicalLocal,cervicalReleasedLocal;readonly OrientedEntryFrame entryFrame;
        readonly Vec3[] savedCervicalSource=new Vec3[25];
        readonly Linear3 inversePelvisBind;Linear3 currentPelvis;
        readonly int pelvisBone;readonly Vec3 pelvisLocalUp,pelvisLocalRight;double unloadedTerminalPressure;
        Vec3 cervicalRotationPelvis,releasedCervicalRotationPelvis,cervicalTranslationPelvis,releasedCervicalTranslationPelvis,rawTip,rawForward;double rawHeadRadius;
        Vec3 cervicalVelocityPelvis,cervicalAngularVelocityPelvis;
        RigidOrganPose cervicalPose,previousCervicalPose;
        readonly InextensibleSourceRig rod; readonly SourceSectionRig sourceRig; readonly int[] sourceTriangles; readonly int root;
        readonly ReceiverSupportTransfer mucosalTransfer;
        readonly int[] primaryIndex;
        readonly ReactiveMaterialChart chart;
        public readonly AdaptiveMeshLandmarks Landmarks;
        public readonly ReceiverAdaptiveLandmarks ReceiverLandmarks;
        readonly ReceiverBoundaryProfiles boundaryProfiles;
        readonly EntranceSurfaceRegularizer entranceRegularizer;
        public bool PreserveEntranceProfile=false,RegularizeEntrance=true;
        public bool SymmetricEntranceOpening=true,ContinuousEntranceJunction=true;
        readonly double[] collarSeamBlend;readonly bool[] collarAnchors;
        readonly ReceiverContactPhase contactPhase=new ReceiverContactPhase();readonly ReceiverSectionField sectionField;
        readonly ReceiverRadialField radialField;readonly SourceSectionContacts sourceSectionContacts;bool baselineMapping;
        public bool FiniteSurfaceCuts=true,CurrentContactProfile=true;
        readonly double[] fieldRadii,savedFieldRadii;readonly Vec3[] materialTargets;Vec3 lengthExtension;double sourceSpacing;
        public readonly bool[] SourceAttachmentMask;public readonly double[] ReceiverOwnership;
        readonly MaterialFeasibilityProjector materialGuard;readonly Vec3[] materialBaseline,guardAnchors;readonly double[] mobility,guardMobility;
        
        readonly int[] collarA,collarB;readonly double[] collarT,collarWeight,sectionReleaseAge;readonly bool[] sourceEntered,wasLocallyLoaded;readonly Vec3[] releasedLocal;bool anySourceEntered,tissueLoaded;
        
        readonly Vec3[] nativeBaseFull,baseFull,basePhysical,centres,targets,tangents,newTangents,xAxis,newX,candidate,previousLocal,sourceGuided,sourceCorrected,sourceOffset;
        readonly double[] station,scale,stateScale,baseRadius,nominalRadius,sourceRadius,sourceDistances,receiverDistances,blend,sourceCoordinate,compression,compressionTarget;
        readonly int[] section; readonly EdgeConstraint[] edges; readonly double[] referenceLengths;
        readonly int[] materialFaces;
        readonly double[] cervicalCollarWeight;
        readonly Vec3[] cervicalRelaxation;readonly double[] cervicalNeighbourWeight;
        readonly EdgeConstraint[] cervicalEdges;readonly int[] cervicalFaces;readonly double[] cervicalReference;
        double cervicalMaterialBlend=1;
        readonly Vec3[] referenceNormals; readonly bool[] primary,primarySupport; readonly double[] faceUmin,faceUmax;int limitingEdge;
        AnatomicalPathConstraintSystem path;readonly Vec3[] guidanceTargets,extendedGuidanceTargets;int guidanceControlCount;double releaseElapsed;
        bool targetsContainCervicalPose;
        public bool ConsistentCervicalGuide=true;
        public bool LocalCervixOnly=true;
        readonly LocalCervixCompression localCervix;
        public AnatomicalPathConstraintSystem SourcePath {get{return path;}}
        public int SectionCount {get{return centres.Length;}}
        public CervicalOpeningLandmark TerminalLandmark {get{return terminal;}}
        public bool IsSourceAttachment(int index){return SourceAttachmentMask[index];}
        public bool IsProtectedOrgan(int index){return chart!=null&&chart.Regions!=null&&(chart.Regions[index]==4||chart.Regions[index]==5||chart.Regions[index]==9);}
        public bool EnableMaterialProjection=true;
        // Isolation switches for recorded-data audits; never persisted in user
        // settings. The guide uses the previous accepted joint position; its
        // normal remains anatomical. It never locks an endpoint to the tip.
        public bool FollowLoadedTerminal=true,SourceCervicalReaction=true,CervixTravelBeforeSource=true,AnatomicalCervicalMotion=true,SharedCanalGuide=false,ExactSourceCuts=false;
        public bool TipDrivenCervix=true;
        public bool StylizedMotion=true;
        public bool UseContactPrecision=true;
        public ReactiveReceiverRig(SimulationDefinition physical,SimulationDefinition support,ReceiverSupportTransfer transfer,ReceiverGuideDefinition guide,Vec3[] sourceReference,int[] sourceFaces,Vec3 sourceAxis,SourceSectionRig sourceRig,ReceiverSupportTransfer mucosalTransfer=null,ReactiveMaterialChart chart=null)
        {
            this.mucosalTransfer=mucosalTransfer;this.chart=chart;if(chart!=null)boundaryProfiles=new ReceiverBoundaryProfiles(chart);
            Landmarks=new AdaptiveMeshLandmarks(sourceReference,sourceFaces,sourceAxis);cervix=new CervicalSurfaceContact(support.Rest,support.Triangles,chart==null?null:chart.TriangleRegions);terminal=new CervicalOpeningLandmark(support.Rest,support.Triangles,chart==null?null:chart.TriangleRegions,(physical.Rest[guide.Boundaries[1][0]]-physical.Rest[guide.Boundaries[0][0]]).Normalized);
            this.physical=physical;this.support=support;this.transfer=transfer;this.guide=guide;entryFrame=new OrientedEntryFrame(guide.Boundaries[0].Length);this.sourceRig=sourceRig;rod=new InextensibleSourceRig(sourceRig);if(rod.Count!=25)throw new ArgumentException("Published source section count differs");sourceTriangles=(int[])sourceFaces.Clone();root=Array.IndexOf(physical.BoneNames,"MED_Pelvis");if(root<0)throw new ArgumentException("Receiver carrier missing");
            cervix.BindSource(sourceReference,sourceFaces,sourceRig);sourceSectionContacts=new SourceSectionContacts(sourceReference,sourceFaces);
            rod.EndpointVertex=Landmarks.MaterialTipVertex;
            cervix.OpeningIndices=terminal.PointIndices;
            pelvisBone=Array.IndexOf(physical.BoneNames,"cf_s_waist02");if(pelvisBone<0)pelvisBone=root;
            int bind=pelvisBone*16;var bindLinear=BindLinear(physical,pelvisBone);
            if(!bindLinear.TryInverse(out inversePelvisBind))throw new ArgumentException("Singular pelvis bind pose");
            int spine=Array.IndexOf(physical.BoneNames,"cf_s_spine01"),left=Array.IndexOf(physical.BoneNames,"cf_s_leg_L"),right=Array.IndexOf(physical.BoneNames,"cf_s_leg_R");
            Vec3 up=spine>=0?(RestBonePosition(physical,spine)-RestBonePosition(physical,pelvisBone)).Normalized:new Vec3(0,1,0),side=left>=0&&right>=0?(RestBonePosition(physical,right)-RestBonePosition(physical,left)).Normalized:new Vec3(1,0,0);
            pelvisLocalUp=bindLinear.Multiply(up).Normalized;pelvisLocalRight=bindLinear.Multiply(side).Normalized;
            SourceAttachmentMask=new bool[sourceReference.Length];for(int i=0;i<SourceAttachmentMask.Length;i++)SourceAttachmentMask[i]=sourceRig.MaterialCoordinate(i)<=1e-8;
            int n=guide.Contours.Length;guidanceTargets=new Vec3[n];extendedGuidanceTargets=new Vec3[n+4];sectionField=new ReceiverSectionField(n);radialField=new ReceiverRadialField(n);fieldRadii=new double[n];savedFieldRadii=new double[n];materialTargets=new Vec3[support.Rest.Length];centres=new Vec3[n];targets=new Vec3[n];tangents=new Vec3[n];newTangents=new Vec3[n];xAxis=new Vec3[n];newX=new Vec3[n];station=new double[n];scale=new double[n];stateScale=new double[n];baseRadius=new double[n];nominalRadius=new double[n];sourceRadius=new double[n];compression=new double[sourceRig.SectionCount];compressionTarget=new double[compression.Length];
            nativeBaseFull=new Vec3[support.Rest.Length];baseFull=new Vec3[support.Rest.Length];candidate=new Vec3[baseFull.Length];previousLocal=new Vec3[baseFull.Length];basePhysical=new Vec3[physical.Rest.Length];referenceNormals=new Vec3[support.Triangles.Length/3];
            section=new int[baseFull.Length];blend=new double[baseFull.Length];sourceGuided=new Vec3[sourceReference.Length];sourceCorrected=new Vec3[sourceReference.Length];sourceOffset=new Vec3[sourceReference.Length];sourceDistances=new double[sourceReference.Length];receiverDistances=new double[basePhysical.Length];sourceCoordinate=new double[sourceReference.Length];primary=new bool[basePhysical.Length];primarySupport=new bool[baseFull.Length];
            foreach(int i in physical.ConfinementTriangles)primary[i]=true;
            primaryIndex=new int[baseFull.Length];for(int i=0;i<primaryIndex.Length;i++)primaryIndex[i]=-1;for(int i=0;i<primary.Length;i++)if(primary[i])primaryIndex[transfer.PhysicalIndices[i]]=i;
            for(int i=0;i<primary.Length;i++)if(primary[i])primarySupport[transfer.PhysicalIndices[i]]=true;
            for(int s=0;s<n;s++){var c=guide.Contours[s];foreach(int i in c.Vertices)if(i<0||i>=basePhysical.Length)throw new ArgumentException("Invalid material section");for(int j=0;j<c.Vertices.Length;j++)station[s]+=physical.GuideCoordinates[c.Vertices[j]]*c.Weights[j];stateScale[s]=1;}
            station[0]=0;station[n-1]=1;for(int s=1;s<n;s++)if(station[s]<=station[s-1])throw new ArgumentException("Unordered section coordinates");
            for(int i=0;i<section.Length;i++){double u=support.GuideCoordinates[i];int s=0;while(s<n-2&&u>station[s+1])s++;section[i]=s;blend[i]=Math.Max(0,Math.Min(1,(u-station[s])/(station[s+1]-station[s])));}
            Vec3 axis=sourceAxis.Normalized;double min=double.PositiveInfinity,max=double.NegativeInfinity;for(int i=0;i<sourceReference.Length;i++){double d=Vec3.Dot(sourceReference[i],axis);sourceCoordinate[i]=d;min=Math.Min(min,d);max=Math.Max(max,d);}for(int i=0;i<sourceCoordinate.Length;i++)sourceCoordinate[i]=(sourceCoordinate[i]-min)/Math.Max(1e-12,max-min);
            cervicalCollarWeight=new double[baseFull.Length];cervicalRelaxation=new Vec3[baseFull.Length];cervicalNeighbourWeight=new double[baseFull.Length];
            if(chart!=null&&terminal.Available)
            {
                Vec3 approach=physical.Rest[guide.Boundaries[1][0]]-physical.Rest[guide.Boundaries[0][0]];terminal.Update(support.Rest,approach,new Vec3(1,0,0));double depth=0;
                for(int i=0;i<baseFull.Length;i++)if(chart.Regions[i]==3)depth=Math.Max(depth,-Vec3.Dot(support.Rest[i]-terminal.Centre,terminal.Normal));
                if(depth>1e-6)for(int i=0;i<baseFull.Length;i++)if(chart.Regions[i]==3){double d=-Vec3.Dot(support.Rest[i]-terminal.Centre,terminal.Normal);cervicalCollarWeight[i]=d>depth*1e-5?SmoothUnit(d/depth):0;}
                localCervix=new LocalCervixCompression(support.Rest,chart.Regions,terminal.Centre,terminal.Normal);
            }
            var topology=new SimulationTopology(support.Rest,support.Triangles,true);
            edges=topology.Stretch;materialFaces=support.Triangles;
            var collarEdges=new List<EdgeConstraint>();var collarFaces=new List<int>();
            if(chart!=null){foreach(var e in edges)if(chart.Regions[e.A]==3&&chart.Regions[e.B]==3)collarEdges.Add(e);for(int f=0;f<support.Triangles.Length;f+=3){int a=support.Triangles[f],b=support.Triangles[f+1],c=support.Triangles[f+2];if(chart.Regions[a]==3&&chart.Regions[b]==3&&chart.Regions[c]==3){collarFaces.Add(a);collarFaces.Add(b);collarFaces.Add(c);}}}
            cervicalEdges=collarEdges.ToArray();cervicalFaces=collarFaces.ToArray();cervicalReference=new double[cervicalEdges.Length];
            if(chart!=null)
            {
                // The complete organ assembly receives one rigid translation.
                // Internal edges/areas therefore have exactly zero procedural
                // strain; retain every boundary to deformable material instead.
                var deformableEdges=new List<EdgeConstraint>();foreach(var e in edges)if(!(RigidAssembly(e.A)&&RigidAssembly(e.B))&&chart.Regions[e.A]!=0&&chart.Regions[e.B]!=0)deformableEdges.Add(e);edges=deformableEdges.ToArray();
                var deformableFaces=new List<int>();for(int f=0;f<support.Triangles.Length;f+=3){int a=support.Triangles[f],b=support.Triangles[f+1],c=support.Triangles[f+2];if(RigidAssembly(a)&&RigidAssembly(b)&&RigidAssembly(c)||chart.Regions[a]==0||chart.Regions[b]==0||chart.Regions[c]==0)continue;deformableFaces.Add(a);deformableFaces.Add(b);deformableFaces.Add(c);}materialFaces=deformableFaces.ToArray();
            }
            referenceLengths=new double[edges.Length];materialGuard=new MaterialFeasibilityProjector(topology);materialBaseline=new Vec3[baseFull.Length];guardAnchors=new Vec3[baseFull.Length];mobility=new double[baseFull.Length];guardMobility=new double[baseFull.Length];for(int i=0;i<mobility.Length;i++)mobility[i]=support.PinWeights[i]>=1?0:1;
            collarA=new int[baseFull.Length];collarB=new int[baseFull.Length];collarT=new double[baseFull.Length];collarWeight=new double[baseFull.Length];collarSeamBlend=new double[baseFull.Length];collarAnchors=new bool[baseFull.Length];sectionReleaseAge=new double[n];sourceEntered=new bool[sourceReference.Length];wasLocallyLoaded=new bool[baseFull.Length];releasedLocal=new Vec3[baseFull.Length];
            if(chart!=null&&chart.Regions!=null)
            {
                var bounds=Bounds3.Empty;foreach(var p in physical.Rest)bounds.Include(p);double width=Math.Max(1e-6,(bounds.Max-bounds.Min).Length*.3);var rim=guide.Boundaries[0];
                for(int i=0;i<baseFull.Length;i++)if(chart.Regions[i]==0&&support.PinWeights[i]<1)
                {
                    double best=double.PositiveInfinity;
                    for(int j=0;j<rim.Length;j++)
                    {
                        int a=transfer.PhysicalIndices[rim[j]],b=transfer.PhysicalIndices[rim[(j+1)%rim.Length]];Vec3 e=support.Rest[b]-support.Rest[a];double t=e.LengthSquared<1e-24?0:Math.Max(0,Math.Min(1,Vec3.Dot(support.Rest[i]-support.Rest[a],e)/e.LengthSquared));double d=(support.Rest[i]-support.Rest[a]-e*t).LengthSquared;
                        if(d<best){best=d;collarA[i]=a;collarB[i]=b;collarT[i]=t;}
                    }
                    double u=Math.Min(1,Math.Sqrt(best)/width);collarWeight[i]=(1-u*u*(3-2*u))*(1-support.PinWeights[i]);
                    collarSeamBlend[i]=1-SmoothUnit((Math.Sqrt(best)-width*.15)/Math.Max(1e-9,width*.25));
                }
            }

            if(chart!=null)for(int f=0;f<support.Triangles.Length;f+=3)for(int k=0;k<3;k++){int a=support.Triangles[f+k],b=support.Triangles[f+(k+1)%3];if(chart.Regions[a]==0&&chart.Regions[b]==1)collarAnchors[a]=true;if(chart.Regions[b]==0&&chart.Regions[a]==1)collarAnchors[b]=true;}

            if(chart!=null&&chart.Regions!=null)for(int i=0;i<mobility.Length;i++)
            {
                int region=chart.Regions[i];if(region==2||region==4||region==5||region==9)mobility[i]=0;
                if(region==3)
                {
                    double nearest=double.PositiveInfinity;foreach(int v in guide.Boundaries[1])nearest=Math.Min(nearest,(support.Rest[i]-physical.Rest[v]).Length);
                    var bounds=Bounds3.Empty;foreach(var p in physical.Rest)bounds.Include(p);double width=Math.Max(1e-6,(bounds.Max-bounds.Min).Length*.15);double w=Math.Max(0,1-nearest/width);mobility[i]*=w*w;
                }
            }
            if(chart!=null)
            {
                int[] ext=new int[4];Vec3 c=physical.Rest[guide.Boundaries[0][0]],normal=(physical.Rest[guide.Boundaries[1][0]]-c).Normalized,x=Axis(new Vec3(1,0,0),normal),y=Vec3.Cross(normal,x);
                for(int k=0;k<4;k++){ext[k]=-1;double best=double.NegativeInfinity;Vec3 direction=k==0?x:k==1?y:k==2?-x:-y;for(int i=0;i<baseFull.Length;i++)if(chart.Regions[i]==0&&collarWeight[i]>.4){Vec3 d=support.Rest[i]-c;double score=Vec3.Dot(d,direction)-Math.Abs(Vec3.Dot(d,normal));if(score>best){best=score;ext[k]=i;}}}
                ReceiverLandmarks=new ReceiverAdaptiveLandmarks(support,chart,n,ext);
            }
            for(int i=0;i<guardMobility.Length;i++)guardMobility[i]=chart==null?mobility[i]:(chart.Regions[i]==1||chart.Regions[i]==2?1:0);
            if(chart!=null)for(int i=0;i<guardMobility.Length;i++)if(chart.SupportAxial[i]>=0&&(chart.SupportAxial[i]<1e-9||chart.SupportAxial[i]>1-1e-9))guardMobility[i]=0;
            ReceiverOwnership=new double[baseFull.Length];if(chart!=null)for(int i=0;i<ReceiverOwnership.Length;i++)ReceiverOwnership[i]=chart.Regions[i]==1||chart.Regions[i]==2?1:chart.Regions[i]==0?collarWeight[i]:0;
            if(chart!=null)entranceRegularizer=new EntranceSurfaceRegularizer(support,chart.Regions,collarWeight,collarAnchors);
            faceUmin=new double[physical.ConfinementTriangles.Length/3];faceUmax=new double[faceUmin.Length];for(int f=0;f<faceUmin.Length;f++){double a=physical.GuideCoordinates[physical.ConfinementTriangles[f*3]],b=physical.GuideCoordinates[physical.ConfinementTriangles[f*3+1]],c=physical.GuideCoordinates[physical.ConfinementTriangles[f*3+2]];faceUmin[f]=Math.Min(a,Math.Min(b,c));faceUmax[f]=Math.Max(a,Math.Max(b,c));}
        }
        static double Ms(long t){return (Stopwatch.GetTimestamp()-t)*1000.0/Stopwatch.Frequency;}
        bool RigidAssembly(int i){return chart!=null&&(!LocalCervixOnly&&chart.Regions[i]==3&&cervicalCollarWeight[i]==0||chart.Regions[i]==4||chart.Regions[i]==5);}
        public bool IsRigidOrganControl(int i){return RigidAssembly(i);}
        static Linear3 BindLinear(SimulationDefinition d,int bone){int b=bone*16;return new Linear3(new Vec3(d.Bindposes[b],d.Bindposes[b+4],d.Bindposes[b+8]),new Vec3(d.Bindposes[b+1],d.Bindposes[b+5],d.Bindposes[b+9]),new Vec3(d.Bindposes[b+2],d.Bindposes[b+6],d.Bindposes[b+10]));}
        static Vec3 RestBonePosition(SimulationDefinition d,int bone){Linear3 inverse;if(!BindLinear(d,bone).TryInverse(out inverse))throw new ArgumentException("Singular anatomical bone bind");int b=bone*16;return -inverse.Multiply(new Vec3(d.Bindposes[b+3],d.Bindposes[b+7],d.Bindposes[b+11]));}
        Linear3 PelvisFrame(ReactiveRigInput input)
        {
            var world=new Linear3(input.Bones[pelvisBone].Multiply(inversePelvisBind.X),input.Bones[pelvisBone].Multiply(inversePelvisBind.Y),input.Bones[pelvisBone].Multiply(inversePelvisBind.Z));
            Vec3 y=world.Multiply(pelvisLocalUp).Normalized,x=world.Multiply(pelvisLocalRight);x=(x-y*Vec3.Dot(x,y)).Normalized;
            return new Linear3(x,y,Vec3.Cross(x,y).Normalized);
        }
        static Vec3 Axis(Vec3 proposed,Vec3 normal){proposed-=normal*Vec3.Dot(proposed,normal);if(proposed.LengthSquared<1e-16)proposed=Vec3.Cross(normal,Math.Abs(normal.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0));return proposed.Normalized;}
        void UpdateGuidance(Vec3 displacement)
        {
            if(!FollowLoadedTerminal)displacement=new Vec3();
            Vec3 terminalDirection=FollowLoadedTerminal?cervicalPose.Direction(terminal.Normal):terminal.Normal;
            Array.Copy(targets,guidanceTargets,targets.Length);
            if(ConsistentCervicalGuide&&!targetsContainCervicalPose&&!LocalCervixOnly)
            {
                // The main-thread guide and worker wall must use the SAME
                // whole-channel organ displacement, not only the terminal span.
                for(int i=0;i<guidanceTargets.Length;i++)
                {
                    double w=AnatomicalLengthConstraint.Profile(station[i]);
                    if(StylizedMotion){var partial=new RigidOrganPose{Pivot=terminal.Centre,Translation=displacement*w,Rotation=cervicalPose.Rotation*w};guidanceTargets[i]=partial.Point(guidanceTargets[i]);}
                    else guidanceTargets[i]+=displacement*w;
                }
            }
            if(SharedCanalGuide&&terminal.Available)
            {
                // The os is beyond the material wall. Keep every canal guide
                // station shared with that wall and append a smooth approach;
                // replacing its final third made Source follow a different tube.
                for(int i=0;i<targets.Length;i++)extendedGuidanceTargets[i]=ConsistentCervicalGuide?guidanceTargets[i]:targets[i]+displacement*Math.Pow(station[i],3);
                Vec3 a=extendedGuidanceTargets[targets.Length-1],b=terminal.Centre+displacement;
                Vec3 first=(a-extendedGuidanceTargets[targets.Length-2]).Normalized,last=terminalDirection;double length=(b-a).Length;
                for(int j=1;j<=4;j++){double t=j/4.0,t2=t*t,t3=t2*t;extendedGuidanceTargets[targets.Length-1+j]=a*(2*t3-3*t2+1)+first*(length*(t3-2*t2+t))+b*(-2*t3+3*t2)+last*(length*(t3-t2));}
                if(path==null||guidanceControlCount!=extendedGuidanceTargets.Length){path=new AnatomicalPathConstraintSystem(extendedGuidanceTargets,true,4);guidanceControlCount=extendedGuidanceTargets.Length;}else path.UpdateControls(extendedGuidanceTargets);
                return;
            }
            if(terminal.Available)
            {
                // The material wall ends at the fornix; Rn is the separate os.
                // Blend the final guide span to its live oriented opening without
                // moving protected anatomy merely to match a visual target.
                int start=Math.Max(0,targets.Length*2/3);Vec3 a=guidanceTargets[start],b=terminal.Centre+displacement;
                Vec3 first=(guidanceTargets[Math.Min(start+1,targets.Length-1)]-a).Normalized,last=terminalDirection;double length=(b-a).Length;
                for(int i=start+1;i<targets.Length;i++){double t=(i-start)/(double)(targets.Length-1-start),t2=t*t,t3=t2*t;guidanceTargets[i]=a*(2*t3-3*t2+1)+first*(length*(t3-2*t2+t))+b*(-2*t3+3*t2)+last*(length*(t3-t2));}
            }
            if(path==null||guidanceControlCount!=guidanceTargets.Length){path=new AnatomicalPathConstraintSystem(guidanceTargets,true,4);guidanceControlCount=guidanceTargets.Length;}else path.UpdateControls(guidanceTargets);
        }
        static Vec3 Skin(SimulationDefinition d,int i,ReactiveRigInput input){Vec3 p=d.Rest[i];if(d.Morphs!=null)for(int m=0;m<d.Morphs.Length;m++)p+=d.Morphs[m].Deltas[i]*input.Morph[m];Vec3 q=new Vec3();for(int j=d.InfluenceOffsets[i];j<d.InfluenceOffsets[i+1];j++){int b=d.BoneIndices[j];q+=(input.Bones[b].Multiply(p)+input.Translation[b])*d.BoneWeights[j];}return q;}
        static Vec3 Segment(Vec3 p,Vec3 a,Vec3 b){Vec3 e=b-a;return a+e*(e.LengthSquared<1e-20?0:Math.Max(0,Math.Min(1,Vec3.Dot(p-a,e)/e.LengthSquared)));}
        static double SmoothUnit(double t){t=Math.Max(0,Math.Min(1,t));return t*t*(3-2*t);}
        Vec3 MapAlongAnatomy(Vec3 point,double coordinate,double length,Vec3 orientation)
        {
            var sections=Landmarks.Sections;int a=0;while(a<sections.Length-2&&sections[a+1].Coordinate<coordinate)a++;double t=(coordinate-sections[a].Coordinate)/Math.Max(1e-12,sections[a+1].Coordinate-sections[a].Coordinate);t=Math.Max(0,Math.Min(1,t));Vec3 oldCentre=sections[a].Centre*(1-t)+sections[a+1].Centre*t,oldTangent=(sections[a+1].Centre-sections[a].Centre).Normalized,oldX=Axis(orientation,oldTangent),oldY=Vec3.Cross(oldTangent,oldX),delta=point-oldCentre;
            double distance=.002+coordinate*length;var target=path.AtDistance(Math.Min(path.Length,distance));if(distance>path.Length)target.Point+=target.Tangent*(distance-path.Length);Vec3 x=Axis(orientation,target.Tangent),y=Vec3.Cross(target.Tangent,x);return target.Point+x*Vec3.Dot(delta,oldX)+y*Vec3.Dot(delta,oldY)+target.Tangent*Vec3.Dot(delta,oldTangent);
        }
        static int Cut(Vec3 a,Vec3 b,Vec3 c,double da,double db,double dc,out Vec3 p,out Vec3 q)
        {
            p=q=new Vec3();int count=0;
            if((da<=0&&db>0)||(db<=0&&da>0)){p=a+(b-a)*(da/(da-db));count++;}
            if((db<=0&&dc>0)||(dc<=0&&db>0)){var v=b+(c-b)*(db/(db-dc));if(count++==0)p=v;else q=v;}
            if((dc<=0&&da>0)||(da<=0&&dc>0)){var v=c+(a-c)*(dc/(dc-da));if(count++==0)p=v;else q=v;}return count;
        }
        void Frames(Vec3[] points,Vec3[] normals,Vec3[] axes,Vec3 orientation)
        {for(int s=0;s<points.Length;s++){normals[s]=(points[Math.Min(points.Length-1,s+1)]-points[Math.Max(0,s-1)]).Normalized;axes[s]=Axis(s==0?orientation:axes[s-1],normals[s]);}}
        double ReceiverRadius(int s)
        {
            if(chart!=null)
            {
                var row=guide.Contours[s].Vertices;Vec3 centre=centres[s],normal=tangents[s];double minimum=double.PositiveInfinity;
                for(int j=0;j<row.Length;j++){Vec3 a=basePhysical[row[j]]-centre,b=basePhysical[row[(j+1)%row.Length]]-centre;a-=normal*Vec3.Dot(a,normal);b-=normal*Vec3.Dot(b,normal);minimum=Math.Min(minimum,Segment(new Vec3(),a,b).Length);}
                return Math.Max(.0005,minimum);
            }
            Vec3 c=centres[s],n=tangents[s];for(int i=0;i<basePhysical.Length;i++)receiverDistances[i]=Vec3.Dot(basePhysical[i]-c,n);double radius=double.PositiveInfinity;var faces=physical.ConfinementTriangles;
            for(int f=0;f<faces.Length;f+=3){int face=f/3;if(faceUmin[face]>station[s]+.07||faceUmax[face]<station[s]-.07)continue;int a=faces[f],b=faces[f+1],d=faces[f+2];Vec3 p,q;if(Cut(basePhysical[a],basePhysical[b],basePhysical[d],receiverDistances[a],receiverDistances[b],receiverDistances[d],out p,out q)!=2)continue;radius=Math.Min(radius,(Segment(c,p,q)-c).Length);}
            if(double.IsInfinity(radius)||radius<1e-5){radius=double.PositiveInfinity;var contour=guide.Contours[s];for(int j=0;j<contour.Vertices.Length;j++){Vec3 v=basePhysical[contour.Vertices[j]]-c;radius=Math.Min(radius,(v-n*Vec3.Dot(v,n)).Length);}}
            return Math.Max(.0005,radius);
        }
        double SourceRadius(int s,Vec3[] vertices,ReactiveRigOutput result)
        {
            if(!anySourceEntered)return 0;
            if(ExactSourceCuts)
            {
                int cuts;double r=SourceSurfaceSection.Radius(vertices,sourceTriangles,targets[s],newTangents[s],sourceDistances,out cuts);
                result.SectionTests++;result.TriangleTests+=cuts;if(r>0)result.ActiveSections++;return r;
            }
            double minimum=double.PositiveInfinity,maximum=double.NegativeInfinity;
            foreach(var p in vertices){double d=Vec3.Dot(p-targets[s],newTangents[s]);minimum=Math.Min(minimum,d);maximum=Math.Max(maximum,d);}
            if(minimum>.0001||maximum<-.0001)return 0;
            var c=targets[s];var n=newTangents[s];var slices=Landmarks.Sections;int nearest=0;double best=double.PositiveInfinity;
            for(int i=0;i<slices.Length;i++){double d=(slices[i].Centre-c).LengthSquared;if(d<best){best=d;nearest=i;}}
            double radius=0;
            for(int j=Math.Max(0,nearest-1);j<=Math.Min(slices.Length-1,nearest+1);j++)
            {
                var slice=slices[j];double den=Vec3.Dot(slice.Tangent,n);if(Math.Abs(den)<.15)continue;
                foreach(var p in slice.Surface){Vec3 q=p-slice.Tangent*(Vec3.Dot(p-c,n)/den);radius=Math.Max(radius,(q-c).Length);}
            }
            result.SectionTests++;if(radius>0)result.ActiveSections++;return radius;
        }
        double WallRadius(int section,double t,double angle)
        {return chart==null||baselineMapping?sectionField.Radius(section,t):radialField.Radius(section,t,angle);}
        void MapReceiver(ReactiveRigInput input)
        {
            for(int i=0;i<candidate.Length;i++)
            {
                candidate[i]=baseFull[i];if(chart!=null)continue;
                int a=section[i],b=a+1;double t=blend[i],w=support.GuideWeights[i];if(support.PinWeights[i]>=1||w<=0){candidate[i]=baseFull[i];continue;}
                Vec3 c=centres[a]*(1-t)+centres[b]*t,q=targets[a]*(1-t)+targets[b]*t,n=(tangents[a]*(1-t)+tangents[b]*t).Normalized,m=(newTangents[a]*(1-t)+newTangents[b]*t).Normalized,x=Axis(xAxis[a]*(1-t)+xAxis[b]*t,n),y=Vec3.Cross(n,x),u=Axis(newX[a]*(1-t)+newX[b]*t,m),v=Vec3.Cross(m,u),d=baseFull[i]-c;double k=baseRadius[a]*scale[a]/nominalRadius[a]*(1-t)+baseRadius[b]*scale[b]/nominalRadius[b]*t;
                Vec3 mapped=q+(u*Vec3.Dot(d,x)+v*Vec3.Dot(d,y))*k+m*Vec3.Dot(d,n);candidate[i]=baseFull[i]+Vec3.ClampLength((mapped-baseFull[i])*w,input.MaximumOffsetRatio*input.PosedSize);
            }
            if(chart!=null&&chart.SupportAxial!=null)for(int i=0;i<candidate.Length;i++)
            {
                if(chart.SupportAxial[i]<0)continue;
                int a=section[i],b=a+1;double t=blend[i],u=chart.SupportAxial[i],angle=chart.SupportAngle[i]*Math.PI*2*chart.AngularDirection+(chart.FrameTwist==null?0:chart.FrameTwist[a]*(1-t)+chart.FrameTwist[b]*t);
                Vec3 n=(newTangents[a]*(1-t)+newTangents[b]*t).Normalized,x=Axis(newX[a]*(1-t)+newX[b]*t,n),y=Vec3.Cross(n,x),c=ReceiverSectionField.Centre(targets,a,t);
                double radius=WallRadius(a,t,angle);if(chart.Regions[i]==2)radius+=input.PosedSize*.007*Math.Sin(u*Math.PI*.5);
                Vec3 desired=c+(x*Math.Cos(angle)+y*Math.Sin(angle))*radius;
                // Boundary contours are non-planar: the entrance reaches farther
                // than the first interior ring. Fading that depth over 20% of
                // the canal reverses axial order. Loft its longitudinal part
                // across the full material length; only transverse shape fades.
                double theta=chart.SupportAngle[i],entryAngle=theta*Math.PI*2*chart.AngularDirection+(chart.FrameTwist==null?0:chart.FrameTwist[0]);
                Vec3 entryBoundary=boundaryProfiles.Point(chart.Regions[i],0,theta,baseFull),entryDelta=entryBoundary-centres[0];
                Vec3 entryRadial=entryDelta-newTangents[0]*Vec3.Dot(entryDelta,newTangents[0]);double oldRadius=entryRadial.Length,needed=WallRadius(0,0,entryAngle),canonicalNeeded=needed;
                Vec3 entryDirection=newX[0]*Math.Cos(entryAngle)+Vec3.Cross(newTangents[0],newX[0])*Math.Sin(entryAngle);
                if(PreserveEntranceProfile&&oldRadius>1e-9){entryDirection=entryRadial/oldRadius;double actualAngle=Math.Atan2(Vec3.Dot(entryDirection,Vec3.Cross(newTangents[0],newX[0])),Vec3.Dot(entryDirection,newX[0]));needed=WallRadius(0,0,actualAngle);}
                entryBoundary=centres[0]+newTangents[0]*Vec3.Dot(entryDelta,newTangents[0])+entryDirection*Math.Max(oldRadius,needed);
                Vec3 canonicalEntry=targets[0]+(newX[0]*Math.Cos(entryAngle)+Vec3.Cross(newTangents[0],newX[0])*Math.Sin(entryAngle))*canonicalNeeded;
                Vec3 entryCorrection=entryBoundary-canonicalEntry;double entryDepth=Vec3.Dot(entryCorrection,newTangents[0]);entryCorrection-=newTangents[0]*entryDepth;
                int last=targets.Length-1;double endAngle=theta*Math.PI*2*chart.AngularDirection+(chart.FrameTwist==null?0:chart.FrameTwist[chart.FrameTwist.Length-1]);
                Vec3 deepBoundary=StylizedMotion?cervicalPose.Point(boundaryProfiles.Point(1,1,theta,baseFull)):boundaryProfiles.Point(1,1,theta,baseFull)+input.Bones[root].Multiply(cervicalLocal);
                if(chart.Regions[i]==2){Vec3 radial=deepBoundary-targets[last];radial-=newTangents[last]*Vec3.Dot(radial,newTangents[last]);deepBoundary+=radial.Normalized*(input.PosedSize*.007);}
                double endRadius=WallRadius(last-1,1,endAngle);if(chart.Regions[i]==2)endRadius+=input.PosedSize*.007;
                Vec3 deepRadial=deepBoundary-targets[last];deepRadial-=newTangents[last]*Vec3.Dot(deepRadial,newTangents[last]);
                if(deepRadial.Length<endRadius)deepBoundary+=deepRadial.Normalized*(endRadius-deepRadial.Length);
                Vec3 canonicalDeep=targets[last]+(newX[last]*Math.Cos(endAngle)+Vec3.Cross(newTangents[last],newX[last])*Math.Sin(endAngle))*endRadius;
                Vec3 deepCorrection=deepBoundary-canonicalDeep;double deepDepth=Vec3.Dot(deepCorrection,newTangents[last]);deepCorrection-=newTangents[last]*deepDepth;
                double ew=Math.Max(0,1-u/.3);ew=ew*ew*(3-2*ew);double dw=Math.Max(0,(u-.7)/.3);dw=dw*dw*(3-2*dw);
                desired+=entryCorrection*ew+deepCorrection*dw+n*(entryDepth*(1-u)+deepDepth*u);

                if(contactPhase.Active)candidate[i]=desired;
                else candidate[i]=baseFull[i]+input.Bones[root].Multiply(releasedLocal[i])*contactPhase.Weight(input.Recovery);

            }
            Vec3 entranceSide=input.Bones[root].X.Normalized;double entranceExpansion=0;
            if(SymmetricEntranceOpening&&chart!=null&&contactPhase.Active)foreach(int node in guide.Boundaries[0]){int v=transfer.PhysicalIndices[node];entranceExpansion=Math.Max(entranceExpansion,Math.Abs(Vec3.Dot(candidate[v]-centres[0],entranceSide))-Math.Abs(Vec3.Dot(baseFull[v]-centres[0],entranceSide)));}
            if(chart!=null&&chart.Regions!=null)for(int i=0;i<candidate.Length;i++)
            {
                int region=chart.Regions[i];if(region==1||region==2)continue;if(region==3||region==4||region==5){candidate[i]=LocalCervixOnly&&localCervix!=null?localCervix.Point(i,baseFull[i],terminal.Normal):StylizedMotion?CervicalCollarPoint(i,input):baseFull[i]+input.Bones[root].Multiply(cervicalLocal);continue;}if(region==9){candidate[i]=baseFull[i];continue;}
                if(region==0)
                {
                    candidate[i]=baseFull[i];
                    if(contactPhase.Active&&collarWeight[i]>0)
                    {
                        int a=collarA[i],b=collarB[i];Vec3 delta=(candidate[a]-baseFull[a])*(1-collarT[i])+(candidate[b]-baseFull[b])*collarT[i];
                        delta-=newTangents[0]*Vec3.Dot(delta,newTangents[0]);
                        if(SymmetricEntranceOpening)
                        {
                            double sign=Math.Max(-1,Math.Min(1,support.Rest[i].X/.0003));Vec3 lateral=entranceSide*(Math.Max(0,entranceExpansion)*sign);
                            // The actual rim attachment must follow ALL transverse
                            // rim motion. Lateral-only transport left horizontal
                            // body triangles stretched across the open aperture.
                            double junction=ContinuousEntranceJunction?collarAnchors[i]?1:collarSeamBlend[i]:0;
                            delta=lateral*(1-junction)+delta*junction;
                        }
                        candidate[i]+=delta*(ContinuousEntranceJunction&&collarAnchors[i]?1:collarWeight[i]);
                    }
                    else if(!contactPhase.Active)candidate[i]+=input.Bones[root].Multiply(releasedLocal[i])*contactPhase.Weight(input.Recovery);
                }

            }
            else if(mucosalTransfer!=null)for(int i=0;i<candidate.Length;i++)
            {
                if(primarySupport[i])continue;
                Vec3 delta=new Vec3();for(int j=mucosalTransfer.Offsets[i];j<mucosalTransfer.Offsets[i+1];j++){int node=mucosalTransfer.PhysicalIndices[mucosalTransfer.Indices[j]];delta+=(candidate[node]-baseFull[node])*mucosalTransfer.Weights[j];}candidate[i]=baseFull[i]+delta;
            }
            if(entranceRegularizer!=null){if(RegularizeEntrance&&contactPhase.Active)entranceRegularizer.Apply(baseFull,candidate,input.Bones);else entranceRegularizer.Reset();}
            if(!LocalCervixOnly&&StylizedMotion&&chart!=null&&contactPhase.Active&&!baselineMapping)RelaxCervicalCollar(input);
        }
        void RelaxCervicalCollar(ReactiveRigInput input)
        {
            for(int pass=0;pass<3;pass++)
            {
                Array.Clear(cervicalRelaxation,0,cervicalRelaxation.Length);Array.Clear(cervicalNeighbourWeight,0,cervicalNeighbourWeight.Length);
                foreach(var edge in edges)
                {
                    int a=edge.A,b=edge.B;if(chart.Regions[a]!=3||chart.Regions[b]!=3)continue;
                    double weight=1/Math.Max(1e-8,(baseFull[a]-baseFull[b]).Length);
                    cervicalRelaxation[a]+=(candidate[b]-cervicalPose.Point(baseFull[b]))*weight;cervicalNeighbourWeight[a]+=weight;
                    cervicalRelaxation[b]+=(candidate[a]-cervicalPose.Point(baseFull[a]))*weight;cervicalNeighbourWeight[b]+=weight;
                }
                for(int i=0;i<candidate.Length;i++)if(cervicalCollarWeight[i]>0&&cervicalNeighbourWeight[i]>0)
                {Vec3 baseline=cervicalPose.Point(baseFull[i]);candidate[i]=baseline+(candidate[i]-baseline)*.35+cervicalRelaxation[i]*(.65/cervicalNeighbourWeight[i]);}
            }
            // First enforce the local material budget of the soft collar.
            // A tiny cervical edge must not reduce every channel section.
            // The full-surface pass below still checks all region boundaries.
            for(int i=0;i<candidate.Length;i++)cervicalRelaxation[i]=chart.Regions[i]==3?cervicalPose.Point(baseFull[i]):candidate[i];
            for(int e=0;e<cervicalEdges.Length;e++)cervicalReference[e]=ReferenceLength(cervicalEdges[e].A,cervicalEdges[e].B,input);
            cervicalMaterialBlend=CoherentSurfaceLimiter.Project(cervicalRelaxation,candidate,cervicalEdges,cervicalReference,cervicalFaces,input.MaxStrain,null,true);
        }
        Vec3 CervicalCollarPoint(int i,ReactiveRigInput input)
        {
            Vec3 point=baseFull[i];double weight=cervicalCollarWeight[i];
            if(weight>0&&contactPhase.Active)
            {
                Vec3 d=point-terminal.Centre,radial=d-terminal.Normal*Vec3.Dot(d,terminal.Normal);double radius=radial.Length;
                if(radius>1e-8&&!baselineMapping)
                {
                    Vec3 posed=cervicalPose.Point(point);
                    double clearance=(input.PosedSize*.0025+rawHeadRadius*(1/Math.Cos(2*Math.PI/ReceiverRadialField.Directions)-1))*weight;
                    Vec3 normal;int span;double parameter;double depth=Landmarks.Envelope.Penetration(posed,0,false,out normal,out span,out parameter);
                    if(depth>1e-8)
                    {
                        // Local minimum-distance reaction, not a ray that may
                        // cross the entire source before finding another exit.
                        // The continuous material weight protects the real os.
                        posed+=normal*(depth+clearance)*weight;
                    }
                    point=cervicalPose.InversePoint(posed);
                }
            }
            else if(weight>0&&!contactPhase.Active)
            {
                var releasedPose=StylizedUterusMotion.Pose(currentPelvis,terminal.Centre,releasedCervicalTranslationPelvis,releasedCervicalRotationPelvis);
                Vec3 released=releasedPose.InversePoint(baseFull[i]+input.Bones[root].Multiply(releasedLocal[i]));point+=(released-baseFull[i])*contactPhase.Weight(input.Recovery);
            }
            return cervicalPose.Point(point);
        }
        double LimitMaterial(ReactiveRigInput input)
        {
            double limit=1;limitingEdge=-1;
            for(int e=0;e<edges.Length;e++){int a=edges[e].A,b=edges[e].B;Vec3 rest=input.Bones[root].Multiply(support.Rest[a]-support.Rest[b]);referenceLengths[e]=Math.Max(rest.Length,(baseFull[a]-baseFull[b]).Length);Vec3 initial=baseFull[a]-baseFull[b],delta=candidate[a]-candidate[b]-initial;double den=delta.LengthSquared,maximum=referenceLengths[e]*(1+input.MaxStrain*.995);if(den<1e-24)continue;double bb=Vec3.Dot(initial,delta),cc=initial.LengthSquared-maximum*maximum,discriminant=bb*bb-den*cc;if(discriminant>=0){double allowed=Math.Max(0,(-bb+Math.Sqrt(discriminant))/den);if(allowed<limit){limit=allowed;limitingEdge=e;}}}
            // Bounded parameter backtracking protects tiny material faces. This
            // is not the dense continuous triangle-pair reference solver.
            for(int f=0;f<referenceNormals.Length;f++){int k=f*3,a=support.Triangles[k],b=support.Triangles[k+1],c=support.Triangles[k+2];referenceNormals[f]=Vec3.Cross(baseFull[b]-baseFull[a],baseFull[c]-baseFull[a]);}
            for(int attempt=0;attempt<10;attempt++){bool valid=true;for(int f=0;f<referenceNormals.Length;f++){int k=f*3,a=support.Triangles[k],b=support.Triangles[k+1],c=support.Triangles[k+2];Vec3 ab=baseFull[b]-baseFull[a]+((candidate[b]-baseFull[b])-(candidate[a]-baseFull[a]))*limit,ac=baseFull[c]-baseFull[a]+((candidate[c]-baseFull[c])-(candidate[a]-baseFull[a]))*limit;double area=referenceNormals[f].LengthSquared;if(area>1e-28&&Vec3.Cross(ab,ac).LengthSquared<area*.0001){valid=false;break;}}if(valid)break;limit*=.5;}
            return Math.Min(1,limit);
        }
        void RelaxSecondary(ReactiveRigInput input)
        {
            // Section geometry is the driving rig. Relax attached layers locally
            // instead of shrinking every section because one tiny collar edge
            // is stretched. No triangle-pair scans occur in this stage.
            for(int sweep=0;sweep<96;sweep++)
            {
                double maximum=0;
                for(int k=0;k<edges.Length;k++)
                {
                    int e=(sweep&1)==0?k:edges.Length-1-k,a=edges[e].A,b=edges[e].B;double wa=primarySupport[a]?0:mobility[a],wb=primarySupport[b]?0:mobility[b];if(wa+wb<=0)continue;Vec3 d=candidate[b]-candidate[a];double length=d.Length,limit=referenceLengths[e]*(1+input.MaxStrain*.99),gap=length-limit;if(gap<=1e-9||length<1e-20)continue;maximum=Math.Max(maximum,gap);Vec3 correction=d*(gap/(length*(wa+wb)));candidate[a]+=correction*wa;candidate[b]-=correction*wb;
                }
                if(maximum<1e-7)break;
            }
        }
        double ReferenceLength(int a,int b,ReactiveRigInput input)
        {
            double length=input.Bones[root].Multiply(support.Rest[a]-support.Rest[b]).Length;
            if(chart!=null&&chart.ReferenceArc!=null&&chart.SupportAxial!=null&&chart.SupportAxial[a]>=0&&chart.SupportAxial[b]>=0)
            {
                int sa=section[a],sb=section[b];double ta=blend[a],tb=blend[b];double arcA=chart.ReferenceArc[sa]*(1-ta)+chart.ReferenceArc[sa+1]*ta,arcB=chart.ReferenceArc[sb]*(1-tb)+chart.ReferenceArc[sb+1]*tb,ra=chart.ReferenceRadius[sa]*(1-ta)+chart.ReferenceRadius[sa+1]*ta,rb=chart.ReferenceRadius[sb]*(1-tb)+chart.ReferenceRadius[sb+1]*tb,angle=Math.Abs(chart.SupportAngle[a]-chart.SupportAngle[b]);angle=Math.Min(angle,1-angle)*2*Math.PI;double axial=arcA-arcB,radial=ra-rb,circumference=angle*(ra+rb)*.5,scale=Math.Max(input.Bones[root].X.Length,Math.Max(input.Bones[root].Y.Length,input.Bones[root].Z.Length));
                // Unfolded material metric: folded neighbouring sections can be
                // close in space while retaining a finite material separation.
                length=Math.Max(length,Math.Sqrt(axial*axial+radial*radial+circumference*circumference)*scale);
            }
            return Math.Max(length,(baseFull[a]-baseFull[b]).Length);
        }
        struct GuideFrame {public Vec3 Orientation;public double Dt,Alpha,Radius;public bool EntryContact,Admitted;}
        public InextensibleSourceRig SourceRod {get{return rod;}}
        public Vec3 NativeSupportPoint(int index){return nativeBaseFull[index];}
        Vec3 AnatomicalDepthAxis()
        {
            Vec3 centre=new Vec3();int count=0;
            if(chart!=null)for(int i=0;i<baseFull.Length;i++)if(chart.Regions[i]==4){centre+=baseFull[i];count++;}
            Vec3 fallback=terminal.Available?terminal.Normal:(targets[targets.Length-1]-targets[0]).Normalized;
            if(count<3||!terminal.Available)return fallback;
            Vec3 axis=(centre/count-terminal.Centre).Normalized;if(axis.LengthSquared<.5)return fallback;
            if(Vec3.Dot(axis,fallback)<0)axis=-axis;return axis;
        }
        GuideFrame PrepareGuide(ReactiveRigInput input,ReactiveRigOutput output,double interval=-1)
        {
            targetsContainCervicalPose=false;
            currentPelvis=PelvisFrame(input);output.UterusPelvisUp=currentPelvis.Y;
            output.CervicalContactTolerance=UseContactPrecision?ContactPrecision.Tolerance(input.SourceRoot,input.PosedSize):1e-6;
            cervicalMaterialBlend=1;output.TerminalCanalFacing=terminal.IsCanalFacingThroat;output.TerminalSurfacePoints=terminal.SurfacePointCount;
            output.CervicalReactionAttempts=output.CervicalReactionRejected=0;
            output.MaterialEdgeCount=edges.Length;output.MaterialFaceCount=materialFaces.Length/3;
            long stage=Stopwatch.GetTimestamp();output.Frame=input.Frame;output.WorkerThread=System.Threading.Thread.CurrentThread.ManagedThreadId;output.SectionTests=output.TriangleTests=output.ActiveSections=output.Contacts=0;output.MaxSourceMovement=output.MaximumResidual=output.MaximumStrain=0;output.Carrier=input.Bones[root];output.CarrierTranslation=input.Translation[root];output.CervicalPenetration=output.CervicalResidual=0;
            for(int i=0;i<baseFull.Length;i++)nativeBaseFull[i]=baseFull[i]=Skin(support,i,input);for(int i=0;i<basePhysical.Length;i++)basePhysical[i]=baseFull[transfer.PhysicalIndices[i]];
            for(int s=0;s<centres.Length;s++){var c=guide.Contours[s];Vec3 p=new Vec3();for(int j=0;j<c.Vertices.Length;j++)p+=basePhysical[c.Vertices[j]]*c.Weights[j];centres[s]=p;targets[s]=p;}
            // Authored boundary centres, not the source root, own the aperture.
            for(int end=0;end<2;end++){int s=end==0?0:centres.Length-1;Vec3 p=new Vec3();foreach(int i in guide.Boundaries[end])p+=basePhysical[i]/guide.Boundaries[end].Length;centres[s]=targets[s]=p;}
            if(!Vec3.Finite(input.CanalLengthScale)||input.CanalLengthScale<1||input.CanalLengthScale>1.5||!Vec3.Finite(input.CervixMaxDisplacement)||input.CervixMaxDisplacement<0||input.CervixMaxDisplacement>.08)throw new ArgumentException("Invalid anatomical travel setting");
            Vec3 nativeOrientation=input.Bones[root].Multiply(new Vec3(1,0,0));
            terminal.Update(baseFull,(centres[centres.Length-1]-centres[centres.Length-2]).Normalized,nativeOrientation);
            Vec3 lengthAxis=AnatomicalDepthAxis();double carrierScale=(input.Bones[root].X.Length+input.Bones[root].Y.Length+input.Bones[root].Z.Length)/3;
            output.NativeCanalArc=AnatomicalLengthConstraint.Arc(centres,station,lengthAxis,0);
            double materialLength=chart==null||chart.ReferenceArc==null?output.NativeCanalArc:chart.ReferenceArc[chart.ReferenceArc.Length-1]*carrierScale;
            double extra=output.NativeCanalArc*(input.CanalLengthScale-1),shift=AnatomicalLengthConstraint.Solve(centres,station,lengthAxis,extra,Math.Max(materialLength,output.NativeCanalArc));
            lengthExtension=lengthAxis*shift;output.CanalLengthDisplacement=lengthExtension;
            output.ConfiguredCanalArc=AnatomicalLengthConstraint.Arc(centres,station,lengthAxis,shift);output.CanalExtension=output.ConfiguredCanalArc-output.NativeCanalArc;
            if(chart!=null&&lengthExtension.LengthSquared>0)
            {
                for(int i=0;i<baseFull.Length;i++){int region=chart.Regions[i];if(region==1||region==2)baseFull[i]+=lengthExtension*AnatomicalLengthConstraint.Profile(chart.SupportAxial[i]);else if(region==3||region==4||region==5)baseFull[i]+=lengthExtension;}
                for(int i=0;i<basePhysical.Length;i++)basePhysical[i]=baseFull[transfer.PhysicalIndices[i]];
                for(int s=0;s<centres.Length;s++){var contour=guide.Contours[s];Vec3 c=new Vec3();for(int j=0;j<contour.Vertices.Length;j++)c+=basePhysical[contour.Vertices[j]]*contour.Weights[j];centres[s]=targets[s]=c;}
            }
            sourceSpacing=Math.Max(1e-5,input.PosedSize*.04);
            if(input.HasSource){sourceRig.CaptureAnimation(input.Source,input.SourceRoot);double arc=0;for(int j=1;j<sourceRig.SectionCount;j++)arc+=(sourceRig.AnimatedCentre(j)-sourceRig.AnimatedCentre(j-1)).Length;sourceSpacing=Math.Max(1e-5,arc/(sourceRig.SectionCount-1));}
            if(ReceiverLandmarks!=null){ReceiverLandmarks.Update(baseFull,centres,station,sourceSpacing,null);for(int s=0;s<centres.Length;s++)centres[s]=targets[s]=ReceiverLandmarks.CentreAtMaterial(station[s]);}
            Vec3 orientation=input.Bones[root].Multiply(new Vec3(1,0,0));Frames(centres,tangents,xAxis,orientation);output.CaptureMs=Ms(stage);stage=Stopwatch.GetTimestamp();
            terminal.Update(baseFull,tangents[tangents.Length-1],orientation);
            if(StylizedMotion){Vec3 travel=currentPelvis.Multiply(cervicalTranslationPelvis);Linear3 carrierInverse;if(!input.Bones[root].TryInverse(out carrierInverse))throw new InvalidOperationException("Singular cervical carrier");cervicalLocal=carrierInverse.Multiply(travel);}
            previousCervicalPose=cervicalPose=new RigidOrganPose{Pivot=terminal.Centre,Translation=input.Bones[root].Multiply(cervicalLocal),Rotation=currentPelvis.Multiply(cervicalRotationPelvis)};
            if(LocalCervixOnly&&localCervix!=null){cervicalLocal=cervicalTranslationPelvis=cervicalRotationPelvis=new Vec3();previousCervicalPose=cervicalPose=new RigidOrganPose{Pivot=terminal.Centre};}
            output.GuidanceCervicalRotationPelvis=cervicalRotationPelvis;
            output.GuidanceCervicalDisplacement=LocalCervixOnly&&localCervix!=null?terminal.Normal*localCervix.Amount:FollowLoadedTerminal?input.Bones[root].Multiply(cervicalLocal):new Vec3();
            UpdateGuidance(output.GuidanceCervicalDisplacement);
            double dt=Math.Max(0,Math.Min(.25,interval>=0?interval:input.Dt)),alpha=1-Math.Exp(-Math.Max(0,input.Response)*dt);
            for(int s=0;s<centres.Length;s++)baseRadius[s]=ReceiverRadius(s);
            double sourceContactRadius=0,guideWeight=0;bool entryContact=false,admitted=false;entryFrame.Update(basePhysical,guide.Boundaries[0],centres[0],centres[1],orientation);
            if(input.HasSource)
            {
                // Guidance preserves the animated root and all material behind it.
                // Coverage never translates the entire object away from its rig.
                Landmarks.Update(input.Source,orientation);double radius=0;
                foreach(var slice in Landmarks.Sections)foreach(var p in slice.Radial)radius=Math.Max(radius,(p-slice.Centre).Length);
                rawTip=Landmarks.MaterialTip;rawForward=(sourceRig.AnimatedCentre(sourceRig.SectionCount-1)-sourceRig.AnimatedCentre(sourceRig.SectionCount-2)).Normalized;rawHeadRadius=0;
                for(int i=0;i<Landmarks.Sections.Length;i++)if(Landmarks.Sections[i].Coordinate>=.75)foreach(var p in Landmarks.Sections[i].Radial)rawHeadRadius=Math.Max(rawHeadRadius,(p-Landmarks.Sections[i].Centre).Length);
                entryContact=entryFrame.Intersects(Landmarks);
                // Initial mid-animation poses can already be inside. Admission
                // requires actual section-envelope overlap, not a half-space alone.
                foreach(var slice in Landmarks.Sections)
                {
                    double entryDepth=Vec3.Dot(slice.Centre-centres[0],entryFrame.Normal),endDepth=Vec3.Dot(slice.Centre-centres[centres.Length-1],tangents[tangents.Length-1]);
                    if(entryDepth<0||endDepth>0)continue;
                    double sr=0;foreach(var p in slice.Radial)sr=Math.Max(sr,(p-slice.Centre).Length);
                    // The almost point-sized terminal slice must not turn the
                    // complete rod on over a microscopic axial interval.
                    double width=Math.Max(1e-8,Math.Max(sr,radius)*.5),axialWeight=SmoothUnit(entryDepth/width)*SmoothUnit(-endDepth/width);
                    for(int j=0;j<centres.Length-1;j++)
                    {
                        double overlap=sr+Math.Max(baseRadius[j],baseRadius[j+1])-(slice.Centre-Segment(slice.Centre,centres[j],centres[j+1])).Length;
                        if(overlap>=0){admitted=true;guideWeight=Math.Max(guideWeight,axialWeight*SmoothUnit(overlap/width));}
                    }
                }
                admitted|=entryContact&&Vec3.Dot(Landmarks.MaterialTip-centres[0],entryFrame.Normal)>=0;
                sourceContactRadius=radius;
                double guideShare=Math.Max(0,input.SourceCompliance)/Math.Max(1e-9,input.SourceCompliance+input.TissueStrength);
                Vec3 chord=centres[centres.Length-1]-centres[0];double chordLength=chord.Length;Vec3 direction=chord.Normalized;
                Vec3 incoming=(Landmarks.MaterialTip-input.SourceRoot).Normalized,entryTangent=(tangents[0]*guideShare+incoming*(1-guideShare)).Normalized,endTangent=tangents[tangents.Length-1];
                // A bounded Hermite corridor retains both anatomical attachments
                // and removes reversals of the sampled/animated intermediate rig.
                // End tangent points into the protected terminal anatomy.
                if(Vec3.Dot(entryTangent,direction)<.2)entryTangent=(entryTangent+direction).Normalized;
                if(Vec3.Dot(endTangent,direction)<.2)endTangent=direction;
                if(entryContact||admitted)for(int s=0;s<targets.Length;s++)
                {
                    double t=station[s],t2=t*t,t3=t2*t;
                    targets[s]=centres[0]*(2*t3-3*t2+1)+entryTangent*(chordLength*.8*(t3-2*t2+t))+centres[centres.Length-1]*(-2*t3+3*t2)+endTangent*(chordLength*.8*(t3-t2));
                }
                var loadedPose=cervicalPose;cervicalPose=new RigidOrganPose{Pivot=terminal.Centre};UpdateGuidance(new Vec3());
                var unloadedEntry=path.AtDistance(0);unloadedTerminalPressure=terminal.Available?Math.Max(0,Vec3.Dot(input.SourceRoot-unloadedEntry.Point,unloadedEntry.Tangent)+sourceSpacing*(sourceRig.SectionCount-1)-path.Length):0;
                cervicalPose=loadedPose;UpdateGuidance(output.GuidanceCervicalDisplacement);
                // Fade by actual overlap rather than flipping all local bending
                // at one boolean admission boundary. Animation remains the input.
                rod.InputFrame=input.SourceFrame;rod.Guide(path,input.Source,input.SourceRoot,radius,input.SourceFollowsPath?guideShare*guideWeight:0,dt,input.Response,input.SourceFollowsPath?input.MaxSource*guideWeight:0,guideWeight,terminal.Available);
                rod.MapVertices(input.Source,sourceGuided);
            }

            output.SourceFrame=input.HasSource?rod.GuidanceFrame:input.SourceFrame;
            output.SourceGuideWeight=guideWeight;
            output.MaterialTipVertex=Landmarks.MaterialTipVertex;output.SourceArcLength=0;
            if(input.HasSource)for(int i=1;i<rod.Count;i++)output.SourceArcLength+=(rod.Animated[i]-rod.Animated[i-1]).Length;
            if(input.HasSource)Array.Copy(rod.Solved,output.SourceGuidedCentres,rod.Count);else Array.Clear(output.SourceGuidedCentres,0,output.SourceGuidedCentres.Length);
            output.PathMs=Ms(stage);
            return new GuideFrame{Orientation=orientation,Dt=dt,Alpha=alpha,Radius=sourceContactRadius,EntryContact=entryContact,Admitted=admitted};
        }
        // Run on a separate, main-thread-owned instance. Shares all reference
        // guidance mathematics with Step, without full tissue/contact transfer.
        public void GuideSourceOnly(ReactiveRigInput input,ReactiveRigOutput output,ReactiveRigOutput contactSnapshot=null,double deltaTime=-1)
        {
            // Transport the exact joint state used for the published guide,
            // not its newer contact result. This preserves guide/contact
            // separation and same-frame equivalence across the worker boundary.
            if(FollowLoadedTerminal&&contactSnapshot!=null)
            {
                if(LocalCervixOnly&&localCervix!=null)localCervix.Seed(contactSnapshot.CervicalDisplacement.Length);
                Linear3 oldInverse;
                if(contactSnapshot.Carrier.TryInverse(out oldInverse))cervicalLocal=oldInverse.Multiply(contactSnapshot.CervicalDisplacement);
                cervicalTranslationPelvis=contactSnapshot.UterusTranslationPelvis;cervicalRotationPelvis=contactSnapshot.UterusRotationPelvis;
                rod.SetGuideSeed(contactSnapshot.SourceAnimatedCentres,contactSnapshot.SourceGuidedCentres,contactSnapshot.SourceFrame);
            }
            long started=Stopwatch.GetTimestamp();var frame=PrepareGuide(input,output,deltaTime);
            if(input.HasSource)
            {
                // The receiver's accepted joint state is transported to this
                // frame, but contact is recomputed from CURRENT source points.
                // Reapplying old contact angles ejected a fast animated tip.
                if(!LocalCervixOnly&&contactSnapshot!=null&&(frame.EntryContact||frame.Admitted)&&cervix.Available)
                {
                    Linear3 inverse;Vec3 travel=new Vec3();
                    if(contactSnapshot.Carrier.TryInverse(out inverse))travel=input.Bones[root].Multiply(inverse.Multiply(contactSnapshot.CervicalDisplacement));
                    // Match the ACTUAL accepted Receiver, including its mapped
                    // cervical collar, rather than the undeformed native mesh.
                    if(contactSnapshot.Carrier.TryInverse(out inverse))for(int i=0;i<materialTargets.Length;i++)materialTargets[i]=nativeBaseFull[i]+input.Bones[root].Multiply(inverse.Multiply(contactSnapshot.Offsets[i]));
                    else Array.Copy(baseFull,materialTargets,baseFull.Length);
                    cervix.Refit(materialTargets);travel=new Vec3();rod.MapVertices(input.Source,sourceGuided);Landmarks.Update(sourceGuided,frame.Orientation);
                    Vec3 outward,direction=terminal.Available?terminal.Normal:path.AtDistance(path.Length).Tangent;
                    double remaining=cervix.Contact(Landmarks,travel,targets[0],direction,input.PosedSize*.001,input.PosedSize*.25,out outward);output.CervicalPenetration=remaining;
                    for(int iteration=0;iteration<3&&remaining>output.CervicalContactTolerance&&input.SourceCompliance>0&&SourceCervicalReaction;iteration++)
                    {
                        if(!ImproveCervicalSource(input,output,travel,direction,frame.Orientation,Math.Max(frame.Radius,input.PosedSize*.025),ref remaining,ref outward))break;
                    }
                    output.CervicalResidual=remaining;output.CervicalQueries=cervix.Queries;
                }
                rod.MapVertices(input.Source,output.SourceWorld);Landmarks.Update(output.SourceWorld,frame.Orientation);
                for(int i=0;i<input.Source.Length;i++)output.SourceOffsets[i]=output.SourceWorld[i]-input.Source[i];
            }
            else {Array.Copy(input.Source,output.SourceWorld,input.Source.Length);Array.Clear(output.SourceOffsets,0,output.SourceOffsets.Length);}
            Array.Copy(rod.Animated,output.SourceAnimatedCentres,rod.Count);Array.Copy(rod.Solved,output.SourceSolvedCentres,rod.Count);
            output.SourceBudgetBlend=rod.BudgetBlend;output.SourceMaximumTurn=rod.MaximumTurn;output.MaxSourceMovement=rod.MaximumDisplacement;
            output.EntryContact=frame.EntryContact;output.SourceAdmitted=frame.Admitted;output.TotalMs=Ms(started);
        }
        bool ImproveCervicalSource(ReactiveRigInput input,ReactiveRigOutput output,Vec3 travel,Vec3 direction,Vec3 orientation,double radius,ref double remaining,ref Vec3 outward)
        {
            Array.Copy(rod.Solved,savedCervicalSource,rod.Count);double oldBudget=rod.BudgetBlend,coordinate=cervix.SourceCoordinate;Vec3 correction=outward*remaining;
            for(int attempt=0;attempt<3;attempt++)
            {
                if(attempt>0)rod.RestoreSolution(savedCervicalSource,input.Source,oldBudget);
                output.CervicalReactionAttempts++;rod.ReactAtCoordinate(correction,coordinate,input.Source,radius,input.MaxSource);
                rod.MapVertices(input.Source,sourceGuided);Landmarks.Update(sourceGuided,orientation);Vec3 normal;
                double next=cervix.Contact(Landmarks,travel,targets[0],direction,input.PosedSize*.001,input.PosedSize*.25,out normal);
                // Finite-surface contact can change the active triangle. Reject
                // a projected rod move that worsens the actual constraint.
                if(next<remaining-output.CervicalContactTolerance*.05||next<=output.CervicalContactTolerance){remaining=next;outward=normal;return true;}
                output.CervicalReactionRejected++;correction*=.5;
            }
            rod.RestoreSolution(savedCervicalSource,input.Source,oldBudget);rod.MapVertices(input.Source,sourceGuided);Landmarks.Update(sourceGuided,orientation);return false;
        }
        void SolveTipDrivenCervix(ReactiveRigInput input,ReactiveRigOutput output,Vec3 axis,Vec3 orientation,double radius,double alpha,ref Vec3 travel)
        {
            bool loaded=false;output.CervicalTipPressure=output.EffectiveCervicalTravel=0;
            bool contact=input.HasSource&&cervix.Available&&(output.EntryContact||output.SourceAdmitted);
            // PrepareGuide returns admission separately; Step sets these fields
            // before calling this method. Surface contact never becomes a cap.
            if(contact)
            {
                cervix.Refit(baseFull);Landmarks.Update(sourceGuided,orientation);Vec3 outward;
                double initial=cervix.Contact(Landmarks,new Vec3(),targets[0],terminal.Normal,input.PosedSize*.001,input.PosedSize*.25,out outward);
                double pressure=Math.Max(0,Vec3.Dot(Landmarks.MaterialTip-terminal.Centre,terminal.Normal)+input.PosedSize*.001);
                double reach=output.SourceArcLength+(Landmarks.MaterialTip-rod.Solved[rod.Count-1]).Length;
                double maximum=CervicalFollowConstraint.TravelWithinReach(terminal.Centre,input.SourceRoot,axis,reach,input.CervixMaxDisplacement);
                output.CervicalTipPressure=pressure;output.EffectiveCervicalTravel=maximum;output.CervicalPenetration=initial;
                loaded=initial>0&&pressure>0&&maximum>0&&input.CervixFollowStrength>0;
                if(loaded)
                {
                    double share=Math.Max(0,input.TissueStrength)/Math.Max(1e-9,input.TissueStrength+input.SourceCompliance);
                    Vec3 desired=CervicalFollowConstraint.AlongAxis(pressure,-terminal.Normal,axis,share*input.CervixFollowStrength,maximum,new Vec3());travel+=(desired-travel)*alpha;
                    if(CervixTravelBeforeSource)for(int i=0;i<4;i++)
                    {
                        double remaining=Math.Max(0,Vec3.Dot(Landmarks.MaterialTip-terminal.Centre-travel,terminal.Normal)+input.PosedSize*.001);
                        if(remaining<=1e-6)break;Vec3 next=CervicalFollowConstraint.AlongAxis(remaining,-terminal.Normal,axis,input.CervixFollowStrength,maximum,travel);
                        if((next-travel).LengthSquared<1e-18)break;travel=next;
                    }
                }
            }
            if(!loaded){if(contactPhase.Active)travel*=Math.Exp(-Math.Max(0,input.Dt)*Math.Max(0,input.Response));else travel=input.Bones[root].Multiply(cervicalReleasedLocal)*contactPhase.Weight(input.Recovery);}
            if(contact)
            {
                Vec3 outward;double remaining=cervix.Contact(Landmarks,travel,targets[0],terminal.Normal,input.PosedSize*.001,input.PosedSize*.25,out outward);
                if(loaded&&CervixTravelBeforeSource)for(int i=0;i<4&&remaining>output.CervicalContactTolerance;i++)
                {
                    Vec3 next=CervicalFollowConstraint.AlongAxis(remaining,outward,axis,input.CervixFollowStrength,output.EffectiveCervicalTravel,travel),nextNormal;
                    if((next-travel).LengthSquared<1e-18)break;
                    double nextDepth=cervix.Contact(Landmarks,next,targets[0],terminal.Normal,input.PosedSize*.001,input.PosedSize*.25,out nextNormal);
                    if(nextDepth>=remaining-1e-8)break;travel=next;remaining=nextDepth;outward=nextNormal;
                }
                for(int i=0;i<3&&remaining>output.CervicalContactTolerance&&input.SourceCompliance>0&&SourceCervicalReaction;i++)if(!ImproveCervicalSource(input,output,travel,terminal.Normal,orientation,Math.Max(radius,input.PosedSize*.025),ref remaining,ref outward))break;
                output.CervicalResidual=remaining;
            }
        }
        void RefitPosedCervix(ReactiveRigInput input,RigidOrganPose pose)
        {
            // Keep the previously accepted deforming collar. Removing its old
            // rigid pose and applying the new one avoids an invisible native
            // collider left behind inside the visible, expanded Receiver.
            for(int i=0;i<materialTargets.Length;i++)materialTargets[i]=pose.Point(previousCervicalPose.InversePoint(baseFull[i]+input.Bones[root].Multiply(previousLocal[i])));
            cervix.Refit(materialTargets);
        }
        void SolveStylizedCervix(ReactiveRigInput input,ReactiveRigOutput output,Vec3 orientation,double radius,double alpha,ref Vec3 travel)
        {
            double groupRadius=0;for(int i=0;i<baseFull.Length;i++)if(RigidAssembly(i))groupRadius=Math.Max(groupRadius,(baseFull[i]-terminal.Centre).Length);
            double pressure=0;if(tissueLoaded&&input.HasSource)
            {
                pressure=input.SourceFollowsPath?unloadedTerminalPressure:Math.Max(0,Vec3.Dot(rawTip-terminal.Centre,terminal.Normal)+input.PosedSize*.001);
            }
            double reach=output.SourceArcLength+(input.HasSource?(rawTip-rod.Animated[rod.Count-1]).Length:0);
            double maximum=input.HasSource?CervicalFollowConstraint.TravelWithinReach(terminal.Centre,input.SourceRoot,currentPelvis.Y,reach,input.CervixMaxDisplacement):input.CervixMaxDisplacement;
            Vec3 intendedTip=input.HasSource?sourceGuided[Landmarks.MaterialTipVertex]:rawTip;
            double motionDemand=AnatomicalLengthConstraint.Solve(targets,station,currentPelvis.Y,pressure,Math.Max(1e-8,input.CervixMaxDisplacement*4));
            Vec3 targetTranslation,targetRotation;StylizedUterusMotion.Target(currentPelvis,intendedTip-terminal.Centre,terminal.Normal,rawForward,motionDemand,maximum,input.CervixFollowStrength,groupRadius,out targetTranslation,out targetRotation);
            Vec3 localTranslation=cervicalTranslationPelvis;
            if(contactPhase.Active){CriticalMotion.Advance(ref localTranslation,ref cervicalVelocityPelvis,targetTranslation,input.Response,Math.Min(.25,input.Dt));CriticalMotion.Advance(ref cervicalRotationPelvis,ref cervicalAngularVelocityPelvis,targetRotation,input.Response,Math.Min(.25,input.Dt));}
            else
            {
                double weight=contactPhase.Weight(input.Recovery);localTranslation=releasedCervicalTranslationPelvis*weight;cervicalRotationPelvis=releasedCervicalRotationPelvis*weight;
                cervicalVelocityPelvis=cervicalAngularVelocityPelvis=new Vec3();
            }
            Vec3 beforeLimit=localTranslation,rotationBeforeLimit=cervicalRotationPelvis;StylizedUterusMotion.Limit(input.CervixMaxDisplacement,groupRadius,ref localTranslation,ref cervicalRotationPelvis);
            if((beforeLimit-localTranslation).LengthSquared+(rotationBeforeLimit-cervicalRotationPelvis).LengthSquared>1e-20)cervicalVelocityPelvis=cervicalAngularVelocityPelvis=new Vec3();
            cervicalTranslationPelvis=localTranslation;cervicalPose=StylizedUterusMotion.Pose(currentPelvis,terminal.Centre,localTranslation,cervicalRotationPelvis);travel=cervicalPose.Translation;
            output.CervicalTipPressure=pressure;output.EffectiveCervicalTravel=maximum;output.UterusRotation=cervicalPose.Rotation;output.UterusRotationPelvis=cervicalRotationPelvis;output.UterusTranslationPelvis=localTranslation;
            // Predict the softer receiver first. Contact feedback against its
            // ACTUAL current geometry is solved after that prediction below.
        }
        public void Step(ReactiveRigInput input,ReactiveRigOutput output)
        {
            long started=Stopwatch.GetTimestamp();var frame=PrepareGuide(input,output);long stage=Stopwatch.GetTimestamp();
            Vec3 orientation=frame.Orientation;double dt=frame.Dt,alpha=frame.Alpha,sourceContactRadius=frame.Radius;bool entryContact=frame.EntryContact,admitted=frame.Admitted;
            tissueLoaded=input.HasSource&&(entryContact||admitted);contactPhase.Advance(tissueLoaded,Math.Max(0,input.Dt));
            if(contactPhase.BeganRelease){Array.Copy(previousLocal,releasedLocal,previousLocal.Length);cervicalReleasedLocal=cervicalLocal;releasedCervicalRotationPelvis=cervicalRotationPelvis;releasedCervicalTranslationPelvis=cervicalTranslationPelvis;}
            releaseElapsed=contactPhase.ReleasedSeconds;
            Vec3 depthAxis=StylizedMotion?currentPelvis.Y:AnatomicalDepthAxis();output.CervicalMotionAxis=depthAxis;
            Vec3 cervicalPush=input.Bones[root].Multiply(cervicalLocal);if(!StylizedMotion&&AnatomicalCervicalMotion)cervicalPush=depthAxis*Math.Max(0,Math.Min(input.CervixMaxDisplacement,Vec3.Dot(cervicalPush,depthAxis)));
            bool cervicalLoaded=false;Vec3 cervicalDirection=terminal.Available?terminal.Normal:path.AtDistance(path.Length).Tangent;
            bool tipDriven=TipDrivenCervix&&terminal.Available;
            output.EntryContact=entryContact;output.SourceAdmitted=admitted;
            bool localMode=LocalCervixOnly&&localCervix!=null,stylized=!localMode&&StylizedMotion&&terminal.Available;
            if(localMode){localCervix.Advance(baseFull,terminal.Centre,terminal.Normal,tissueLoaded?unloadedTerminalPressure:0,input.CervixMaxDisplacement,input.CervixFollowStrength,input.Dt,input.Response,contactPhase.Active,contactPhase.BeganRelease,contactPhase.Weight(input.Recovery));cervicalPush=terminal.Normal*localCervix.Amount;output.CervicalTipPressure=unloadedTerminalPressure;output.EffectiveCervicalTravel=localCervix.Maximum;output.UterusRotation=output.UterusRotationPelvis=output.UterusTranslationPelvis=new Vec3();UpdateGuidance(cervicalPush);if(input.HasSource&&input.SourceFollowsPath){rod.RefreshEndpoint(path,input.Source,Math.Max(sourceContactRadius,input.PosedSize*.001),input.MaxSource*output.SourceGuideWeight,true);rod.MapVertices(input.Source,sourceGuided);Array.Copy(rod.Solved,output.SourceGuidedCentres,rod.Count);}}
            else if(stylized)SolveStylizedCervix(input,output,orientation,sourceContactRadius,alpha,ref cervicalPush);
            else if(tipDriven)SolveTipDrivenCervix(input,output,depthAxis,orientation,sourceContactRadius,alpha,ref cervicalPush);
            else if(input.HasSource&&cervix.Available&&(entryContact||admitted))
            {
                cervix.Refit(baseFull);Landmarks.Update(sourceGuided,orientation);Vec3 outward;
                double depth=cervix.Contact(Landmarks,new Vec3(),targets[0],cervicalDirection,input.PosedSize*.001,input.PosedSize*.25,out outward);
                if(depth>0)
                {
                    cervicalLoaded=true;output.CervicalPenetration=depth;
                    double share=Math.Max(0,input.TissueStrength)/Math.Max(1e-9,input.TissueStrength+input.SourceCompliance);
                    Vec3 desired=AnatomicalCervicalMotion?CervicalFollowConstraint.AlongAxis(depth,outward,depthAxis,share*input.CervixFollowStrength,input.CervixMaxDisplacement,new Vec3()):CervicalFollowConstraint.Target(depth,outward,share*input.CervixFollowStrength,input.CervixMaxDisplacement);
                    cervicalPush+=(desired-cervicalPush)*alpha;
                    double remaining=cervix.Contact(Landmarks,cervicalPush,targets[0],cervicalDirection,input.PosedSize*.001,input.PosedSize*.25,out outward);
                    // Use the compliant organ mount's available travel before
                    // bending the stiffer root-attached source. The former order
                    // snapped the source back even while this budget was unused.
                    if(CervixTravelBeforeSource&&input.CervixFollowStrength>0)for(int iteration=0;iteration<4&&remaining>output.CervicalContactTolerance;iteration++)
                    {
                        Vec3 corrected=AnatomicalCervicalMotion?CervicalFollowConstraint.AlongAxis(remaining,outward,depthAxis,input.CervixFollowStrength,input.CervixMaxDisplacement,cervicalPush):Vec3.ClampLength(cervicalPush-outward*(remaining*input.CervixFollowStrength),input.CervixMaxDisplacement);
                        if((corrected-cervicalPush).LengthSquared<1e-18)break;cervicalPush=corrected;
                        remaining=cervix.Contact(Landmarks,cervicalPush,targets[0],cervicalDirection,input.PosedSize*.001,input.PosedSize*.25,out outward);
                    }
                    for(int iteration=0;iteration<3&&remaining>output.CervicalContactTolerance&&input.SourceCompliance>0&&SourceCervicalReaction;iteration++)
                    {
                        if(!ImproveCervicalSource(input,output,cervicalPush,cervicalDirection,orientation,Math.Max(sourceContactRadius,input.PosedSize*.025),ref remaining,ref outward))break;
                    }
                    // If the root-attached rod reaches a shape/budget limit,
                    // use remaining permitted organ travel before accepting a
                    // residual. This is unilateral contact, never a tip latch.
                    for(int iteration=0;iteration<2&&remaining>output.CervicalContactTolerance&&input.CervixFollowStrength>0;iteration++)
                    {
                        Vec3 corrected=AnatomicalCervicalMotion?CervicalFollowConstraint.AlongAxis(remaining,outward,depthAxis,input.CervixFollowStrength,input.CervixMaxDisplacement,cervicalPush):Vec3.ClampLength(cervicalPush-outward*(remaining*input.CervixFollowStrength),input.CervixMaxDisplacement);
                        if((corrected-cervicalPush).LengthSquared<1e-18)break;cervicalPush=corrected;
                        remaining=cervix.Contact(Landmarks,cervicalPush,targets[0],cervicalDirection,input.PosedSize*.001,input.PosedSize*.25,out outward);
                    }
                    output.CervicalResidual=remaining;
                }
            }
            if(!stylized&&!tipDriven&&!cervicalLoaded)
            {
                if(contactPhase.Active)cervicalPush*=Math.Exp(-Math.Max(0,input.Dt)*Math.Max(0,input.Response));
                else cervicalPush=input.Bones[root].Multiply(cervicalReleasedLocal)*contactPhase.Weight(input.Recovery);
            }

            cervicalPush=Vec3.ClampLength(cervicalPush,input.CervixMaxDisplacement);
            Linear3 carrierInverse;if(!input.Bones[root].TryInverse(out carrierInverse))throw new InvalidOperationException("Singular cervical carrier");cervicalLocal=carrierInverse.Multiply(cervicalPush);output.CervicalDisplacement=cervicalPush;output.CervicalQueries=cervix.Queries;
            double beforePushLength=0;for(int s=1;s<targets.Length;s++)beforePushLength+=(targets[s]-targets[s-1]).Length;
            for(int s=0;!localMode&&s<targets.Length;s++)
            {
                double weight=AnatomicalLengthConstraint.Profile(station[s]);
                if(stylized){var partial=new RigidOrganPose{Pivot=terminal.Centre,Translation=cervicalPush*weight,Rotation=cervicalPose.Rotation*weight};targets[s]=partial.Point(targets[s]);}
                else targets[s]+=cervicalPush*weight;
            }
            targetsContainCervicalPose=true;
            if(stylized&&input.HasSource&&input.SourceFollowsPath&&output.SourceGuideWeight>0)
            {
                UpdateGuidance(cervicalPush);rod.RefreshEndpoint(path,input.Source,Math.Max(sourceContactRadius,input.PosedSize*.001),input.MaxSource*output.SourceGuideWeight,terminal.Available);rod.MapVertices(input.Source,sourceGuided);
                Array.Copy(rod.Solved,output.SourceGuidedCentres,rod.Count);output.GuidanceCervicalDisplacement=cervicalPush;output.GuidanceCervicalRotationPelvis=cervicalRotationPelvis;
            }
            double afterPushLength=0;for(int s=1;s<targets.Length;s++)afterPushLength+=(targets[s]-targets[s-1]).Length;
            output.ContactCanalExtension=Math.Max(0,afterPushLength-beforePushLength);output.CanalExtension+=output.ContactCanalExtension;
            output.TerminalAvailable=terminal.Available;
            if(terminal.Available)
            {
                output.TerminalCentre=terminal.Centre+cervicalPush;output.TerminalNormal=stylized?cervicalPose.Direction(terminal.Normal):terminal.Normal;output.TerminalRadius=terminal.Radius;
                output.TerminalRootDistance=(output.TerminalCentre-input.SourceRoot).Length;
                output.TerminalWithinSourceLength=input.HasSource&&output.TerminalRootDistance<=output.SourceArcLength+1e-8;
                for(int k=0;k<5;k++)output.TerminalPoints[k]=stylized?cervicalPose.Point(terminal.Points[k]):terminal.Points[k]+cervicalPush;
                Vec3 delta=Landmarks.MaterialTip-output.TerminalCentre;output.TerminalTipAxial=Vec3.Dot(delta,output.TerminalNormal);output.TerminalTipLateral=(delta-output.TerminalNormal*output.TerminalTipAxial).Length;
            }

            Frames(targets,newTangents,newX,chart==null?orientation:basePhysical[guide.Boundaries[0][0]]-centres[0]);
            output.PathMs+=Ms(stage);stage=Stopwatch.GetTimestamp();Array.Clear(compressionTarget,0,compressionTarget.Length);
            if(input.HasSource)Landmarks.Update(sourceGuided,orientation);
            anySourceEntered=false;for(int i=0;i<sourceEntered.Length;i++){sourceEntered[i]=input.HasSource&&Vec3.Dot(sourceGuided[i]-targets[0],newTangents[0])>=-input.PosedSize*.001;anySourceEntered|=sourceEntered[i];}

            for(int s=0;s<centres.Length;s++){nominalRadius[s]=0;var contour=guide.Contours[s];for(int j=0;j<contour.Vertices.Length;j++){Vec3 delta=basePhysical[contour.Vertices[j]]-centres[s];nominalRadius[s]+=(delta-tangents[s]*Vec3.Dot(delta,tangents[s])).Length*contour.Weights[j];}nominalRadius[s]=Math.Max(.0005,nominalRadius[s]);sourceRadius[s]=input.HasSource&&(entryContact||admitted)?SourceRadius(s,sourceGuided,output):0;}
            for(int s=0;s<centres.Length;s++)
            {
                double gap=Math.Max(0,sourceRadius[s]+(sourceRadius[s]>0?.00035:0)-baseRadius[s]);double share=Math.Max(0,input.TissueStrength)/(Math.Max(0,input.TissueStrength)+Math.Max(0,input.SourceCompliance)+input.Compliance/Math.Max(1e-8,dt*dt));double target=1+gap*share/baseRadius[s];
                bool loaded=gap>1e-7;sectionReleaseAge[s]=loaded?0:Math.Min(input.Recovery,sectionReleaseAge[s]+Math.Max(0,input.Dt));
                stateScale[s]+=((input.HasSource?target:1)-stateScale[s])*alpha;scale[s]=Math.Max(1,stateScale[s]);if(gap>0){output.Contacts++;double nearest=double.PositiveInfinity;int node=0;for(int j=0;j<sourceRig.SectionCount;j++){double d=(sourceRig.AnimatedCentre(j)-targets[s]).LengthSquared;if(d<nearest){nearest=d;node=j;}}compressionTarget[node]=Math.Max(compressionTarget[node],Math.Min(.25,gap*(1-share)/Math.Max(.001,sourceRadius[s])));}
            }
            for(int i=0;i<input.Source.Length;i++){sourceCorrected[i]=sourceGuided[i];sourceOffset[i]=sourceCorrected[i]-input.Source[i];}
            output.SourceFrame=rod.GuidanceFrame;output.SourceBudgetBlend=rod.BudgetBlend;output.SourceMaximumTurn=rod.MaximumTurn;
            Array.Copy(rod.Animated,output.SourceAnimatedCentres,rod.Count);Array.Copy(rod.Solved,output.SourceSolvedCentres,rod.Count);
            // Reconcile the receiver against the accepted source correction.
            // Also bound vertices between cut planes; endpoint-only samples
            // would miss bulges of the real open shell.
            radialField.Begin(baseRadius);
            if(FiniteSurfaceCuts&&input.HasSource&&(entryContact||admitted))
            {
                sourceSectionContacts.Refit(sourceCorrected);
                for(int s=0;s<targets.Length;s++)sourceSectionContacts.Include(radialField,s,targets[s],newTangents[s],newX[s],Vec3.Cross(newTangents[s],newX[s]),input.PosedSize*.0025);
                output.TriangleTests+=sourceSectionContacts.TriangleTests;
            }
            for(int i=0;!FiniteSurfaceCuts&&i<sourceCorrected.Length&&input.HasSource;i++)
            {
                if(!sourceEntered[i]||!(entryContact||admitted))continue;Vec3 p=sourceCorrected[i];if(Vec3.Dot(p-targets[targets.Length-1],newTangents[newTangents.Length-1])>input.PosedSize*.002)continue;double best=double.PositiveInfinity;int span=0;
                for(int s=0;s<targets.Length-1;s++){double distance=(p-Segment(p,targets[s],targets[s+1])).LengthSquared;if(distance<best){best=distance;span=s;}}
                Vec3 spanNormal=(targets[span+1]-targets[span]).Normalized;Vec3 from=p-targets[span];double radius=(from-spanNormal*Vec3.Dot(from,spanNormal)).Length+input.PosedSize*.01;double left=Math.Max(.5,Math.Abs(Vec3.Dot(newTangents[span],spanNormal))),right=Math.Max(.5,Math.Abs(Vec3.Dot(newTangents[span+1],spanNormal)));scale[span]=Math.Max(scale[span],radius/(left*baseRadius[span]));scale[span+1]=Math.Max(scale[span+1],radius/(right*baseRadius[span+1]));
                for(int s=span;s<=span+1;s++)
                {
                    double den=Vec3.Dot(spanNormal,newTangents[s]);if(Math.Abs(den)<.5)continue;
                    Vec3 cut=p-spanNormal*(Vec3.Dot(p-targets[s],newTangents[s])/den)-targets[s];
                    radialField.Include(s,cut,newX[s],Vec3.Cross(newTangents[s],newX[s]),input.PosedSize*.0025);
                }
            }
            // Active contact fits the CURRENT shell. Retaining a slowly
            // decaying radius from another side/phase caused opposite-wall
            // dilation. Inactive recovery is owned by contactPhase/MapReceiver.
            radialField.Update(targets,dt,input.Response,CurrentContactProfile&&contactPhase.Active);
            output.EntryContact=entryContact;output.SourceAdmitted=admitted;output.ReceiverActive=contactPhase.Active;
            for(int s=0;s<fieldRadii.Length;s++)fieldRadii[s]=baseRadius[s]*scale[s];sectionField.Update(targets,fieldRadii);

            output.ContactMs=Ms(stage);stage=Stopwatch.GetTimestamp();MapReceiver(input);output.CervicalMaterialBlend=cervicalMaterialBlend;
            for(int e=0;e<edges.Length;e++){int a=edges[e].A,b=edges[e].B;referenceLengths[e]=ReferenceLength(a,b,input);}
            for(int i=0;i<materialBaseline.Length;i++)materialBaseline[i]=baseFull[i]+input.Bones[root].Multiply(previousLocal[i]);
            double coherentBlend=1;
            if(EnableMaterialProjection&&chart!=null&&contactPhase.Active)
            {
                Array.Copy(candidate,materialTargets,candidate.Length);Array.Copy(fieldRadii,savedFieldRadii,fieldRadii.Length);
                for(int i=0;i<fieldRadii.Length;i++)fieldRadii[i]=baseRadius[i];
                sectionField.Update(targets,fieldRadii);baselineMapping=true;MapReceiver(input);baselineMapping=false;Array.Copy(candidate,materialBaseline,candidate.Length);
                Array.Copy(savedFieldRadii,fieldRadii,fieldRadii.Length);sectionField.Update(targets,fieldRadii);Array.Copy(materialTargets,candidate,candidate.Length);
                for(int e=0;e<edges.Length;e++)referenceLengths[e]=Math.Max(referenceLengths[e],(materialBaseline[edges[e].A]-materialBaseline[edges[e].B]).Length);
                coherentBlend=CoherentSurfaceLimiter.Project(materialBaseline,candidate,edges,referenceLengths,materialFaces,input.MaxStrain,chart.SupportAxial);
            }
            else if(EnableMaterialProjection&&chart==null&&(tissueLoaded||releaseElapsed<input.Recovery))
            {
                // The external body stays on the native entrance rig. Project
                // local wall targets against all material edges and face areas;
                // four bounded sweeps replace the old 96-pass secondary loop.
                for(int i=0;i<materialBaseline.Length;i++){guardAnchors[i]=chart!=null&&guardMobility[i]==0?candidate[i]:baseFull[i];materialBaseline[i]=guardAnchors[i]+input.Bones[root].Multiply(previousLocal[i]);}
                for(int e=0;e<edges.Length;e++)referenceLengths[e]=Math.Max(referenceLengths[e],(guardAnchors[edges[e].A]-guardAnchors[edges[e].B]).Length);
                materialGuard.BeginStep(materialBaseline,guardAnchors,guardMobility,referenceLengths,input.MaxStrain,input.PosedSize*input.MaximumOffsetRatio);
                materialGuard.Project(candidate,4);
            }double materialBlend=1;output.LimitingEdge=-1;output.MaterialBlend=coherentBlend;output.MaximumPrimaryStrain=0;Linear3 inverse;if(!input.Bones[root].TryInverse(out inverse))throw new InvalidOperationException("Singular rig carrier");
            double recovery=contactPhase.Active?0:input.Recovery>0?Math.Min(1,releaseElapsed/input.Recovery):1,recoveryWeight=1-recovery*recovery*(3-2*recovery);output.RecoveryFraction=recovery;
            for(int i=0;i<candidate.Length;i++){Vec3 delta=(candidate[i]-baseFull[i])*materialBlend;if(chart!=null||tissueLoaded){previousLocal[i]=inverse.Multiply(delta);output.Offsets[i]=baseFull[i]-nativeBaseFull[i]+delta;}else output.Offsets[i]=baseFull[i]-nativeBaseFull[i]+input.Bones[root].Multiply(previousLocal[i])*recoveryWeight;}
            for(int i=0;i<baseFull.Length;i++)output.FullWorld[i]=nativeBaseFull[i]+output.Offsets[i];
            if(entranceRegularizer!=null){entranceRegularizer.Measure(output.FullWorld,baseFull);output.EntranceRegularizedVertices=entranceRegularizer.Prepared?entranceRegularizer.VertexCount:0;output.EntranceSymmetryPairs=entranceRegularizer.Prepared?entranceRegularizer.PairCount:0;output.EntranceSymmetryResidual=entranceRegularizer.MaximumResidual;output.EntranceMidlineVertices=entranceRegularizer.Prepared?entranceRegularizer.MidlineCount:0;output.EntranceMidlineResidual=entranceRegularizer.MidlineResidual;}
            for(int i=0;i<basePhysical.Length;i++)output.Physical[i]=output.FullWorld[transfer.PhysicalIndices[i]];
            for(int e=0;e<edges.Length;e++){int a=edges[e].A,b=edges[e].B;if(referenceLengths[e]>1e-12){double strain=(output.FullWorld[a]-output.FullWorld[b]).Length/referenceLengths[e]-1;if(strain>output.MaximumStrain){output.MaximumStrain=strain;output.LimitingEdge=e;}if(primarySupport[a]&&primarySupport[b])output.MaximumPrimaryStrain=Math.Max(output.MaximumPrimaryStrain,strain);}}
            for(int i=0;i<input.Source.Length;i++){output.SourceOffsets[i]=input.HasSource?sourceOffset[i]:new Vec3();output.SourceWorld[i]=input.Source[i]+output.SourceOffsets[i];output.MaxSourceMovement=Math.Max(output.MaxSourceMovement,output.SourceOffsets[i].Length);}
            if(!localMode&&stylized&&cervix.Available&&input.HasSource&&tissueLoaded)
            {
                cervix.Refit(output.FullWorld);Landmarks.Update(output.SourceWorld,orientation);Vec3 outward;
                double remaining=cervix.Contact(Landmarks,new Vec3(),targets[0],output.TerminalNormal,input.PosedSize*.001,input.PosedSize*.25,out outward);output.CervicalPenetration=remaining;
                for(int i=0;i<3&&remaining>output.CervicalContactTolerance&&input.SourceCompliance>0&&SourceCervicalReaction;i++)if(!ImproveCervicalSource(input,output,new Vec3(),output.TerminalNormal,orientation,Math.Max(sourceContactRadius,input.PosedSize*.025),ref remaining,ref outward))break;
                output.CervicalResidual=remaining;output.CervicalQueries=cervix.Queries;rod.MapVertices(input.Source,sourceCorrected);
                output.MaxSourceMovement=0;for(int i=0;i<input.Source.Length;i++){output.SourceWorld[i]=sourceCorrected[i];output.SourceOffsets[i]=sourceOffset[i]=sourceCorrected[i]-input.Source[i];output.MaxSourceMovement=Math.Max(output.MaxSourceMovement,output.SourceOffsets[i].Length);}
                Array.Copy(rod.Solved,output.SourceSolvedCentres,rod.Count);output.SourceBudgetBlend=rod.BudgetBlend;output.SourceMaximumTurn=rod.MaximumTurn;
            }
            // These metrics must describe the final accepted source, after
            // physical contact, rather than the earlier guidance prediction.
            if(localMode&&input.HasSource){cervix.Refit(output.FullWorld);Landmarks.Update(output.SourceWorld,orientation);Vec3 normal;output.CervicalResidual=cervix.Contact(Landmarks,new Vec3(),targets[0],terminal.Normal,input.PosedSize*.001,input.PosedSize*.25,out normal);output.CervicalPenetration=output.CervicalResidual;output.CervicalQueries=cervix.Queries;}
            if(terminal.Available&&input.HasSource){Landmarks.Update(sourceCorrected,orientation);Vec3 delta=Landmarks.MaterialTip-output.TerminalCentre;output.TerminalTipDistance=delta.Length;output.TerminalTipAxial=Vec3.Dot(delta,output.TerminalNormal);output.TerminalTipLateral=(delta-output.TerminalNormal*output.TerminalTipAxial).Length;}
            else output.TerminalTipDistance=output.TerminalTipAxial=output.TerminalTipLateral=0;
            for(int i=0;i<basePhysical.Length;i++)basePhysical[i]=output.Physical[i];
            for(int s=0;s<centres.Length;s++){var c=guide.Contours[s];Vec3 p=new Vec3();for(int j=0;j<c.Vertices.Length;j++)p+=output.Physical[c.Vertices[j]]*c.Weights[j];centres[s]=output.Path[s]=p;}
            Frames(centres,tangents,xAxis,orientation);for(int s=0;s<centres.Length;s++)baseRadius[s]=ReceiverRadius(s);
            output.EntranceRadius=baseRadius[0];output.RequiredEntranceRadius=sourceRadius[0];output.ContainedVertices=output.EngagedVertices=0;output.SourceVertices=input.Source.Length;
            for(int i=0;i<input.Source.Length&&input.HasSource;i++){Vec3 p=sourceCorrected[i];if(Vec3.Dot(p-output.Path[0],tangents[0])<0||Vec3.Dot(p-output.Path[centres.Length-1],tangents[centres.Length-1])>0)continue;output.EngagedVertices++;double best=double.PositiveInfinity,allowed=0;for(int s=0;s<centres.Length-1;s++){Vec3 e=output.Path[s+1]-output.Path[s];double t=e.LengthSquared<1e-20?0:Math.Max(0,Math.Min(1,Vec3.Dot(p-output.Path[s],e)/e.LengthSquared));double distance=(p-output.Path[s]-e*t).Length;if(distance<best){best=distance;allowed=baseRadius[s]*(1-t)+baseRadius[s+1]*t;}}if(best<=allowed+.0005)output.ContainedVertices++;else output.MaximumResidual=Math.Max(output.MaximumResidual,best-allowed);}
            output.LandmarkCount=0;if(input.HasSource){Landmarks.Update(sourceCorrected,orientation);output.LandmarkPoints[output.LandmarkCount++]=Landmarks.MaterialBase;output.LandmarkPoints[output.LandmarkCount++]=Landmarks.MaterialTip;foreach(var s in Landmarks.Sections){output.LandmarkPoints[output.LandmarkCount++]=s.Centre;foreach(var p in s.Radial)output.LandmarkPoints[output.LandmarkCount++]=p;}}
            for(int s=0;s<guide.Contours.Length;s++){output.LandmarkPoints[output.LandmarkCount++]=output.Path[s];var c=guide.Contours[s];for(int k=0;k<4;k++)output.LandmarkPoints[output.LandmarkCount++]=output.Physical[c.Vertices[Math.Min(c.Vertices.Length-1,k*c.Vertices.Length/4)]];}
            output.ReceiverLandmarkCount=0;
            if(ReceiverLandmarks!=null){ReceiverLandmarks.Update(output.FullWorld,output.Path,station,sourceSpacing,input.HasSource?rod.Solved:null);output.ReceiverLandmarkCount=ReceiverLandmarks.Count;output.ReceiverSpacing=ReceiverLandmarks.Spacing;Array.Copy(ReceiverLandmarks.Points,output.ReceiverLandmarkPoints,ReceiverLandmarks.Count*5);Array.Copy(ReceiverLandmarks.SourcePairs,output.ReceiverSourcePairs,ReceiverLandmarks.Count);Array.Copy(ReceiverLandmarks.ExternalPoints,output.ExternalLandmarkPoints,5);}
            output.TransferMs=Ms(stage);output.TotalMs=Ms(started);
        }
    }
}
