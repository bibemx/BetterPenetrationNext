using System;
using System.IO;
using System.Diagnostics;
using BetterPenetrationNext.Core;
using BepInEx.Configuration;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed partial class NextRuntime
    {
        ConfigEntry<bool> anatomicalReceiver;
        ConfigEntry<bool> reactiveRig,backgroundRig,wholeSource,debugLandmarks,debugReceiverLandmarks;
        ConfigEntry<float> cervixMaxDisplacement,cervixFollowStrength,canalLengthScale;
        ConfigEntry<float> sourceLengthScale,sourceWidthScale;
        static SimulationDefinition channelDefinition,supportDefinition;
        static ReceiverSupportTransfer channelTransfer;
        static ReceiverSupportTransfer reactiveTransfer;
        static ReactiveMaterialChart reactiveChart;
        static ReceiverGuideDefinition channelContours;
        static AnimatedPathProfile channelProfile;
        static T ReadChannelResource<T>(string name)
        {
            using(var stream=typeof(NextRuntime).Assembly.GetManifestResourceStream("BetterPenetrationNext.channel."+name))
            {
                if(stream==null)throw new InvalidOperationException("Missing channel resource: "+name);
                using(var reader=new StreamReader(stream))return JsonCodec.Deserialize<T>(reader.ReadToEnd());
            }
        }
        static void LoadChannelResources()
        {
            if(channelDefinition!=null)return;
            var physical=ReadChannelResource<SimulationDefinition>("physical.json");var support=ReadChannelResource<SimulationDefinition>("support.json");
            channelTransfer=ReadChannelResource<ReceiverSupportTransfer>("transfer.json");channelContours=ReadChannelResource<ReceiverGuideDefinition>("contours.json");channelProfile=ReadChannelResource<AnimatedPathProfile>("path.json");
            reactiveTransfer=ReadChannelResource<ReceiverSupportTransfer>("reactive-transfer.json");reactiveChart=ReadChannelResource<ReactiveMaterialChart>("chart.json");
            physical.Validate(56130);support.Validate(56130);channelProfile.Validate(physical);supportDefinition=support;channelDefinition=physical;
        }
        bool UseAnatomicalReceiver {get{return anatomicalReceiver!=null&&anatomicalReceiver.Value&&contactMode.Value==ContactMode.Bidirectional&&pathMode.Value!=PathMode.Disabled;}}
        sealed class NativeAnatomicalSource : IAnatomicalSourceBinding
        {
            readonly Observation observation;readonly Vec3 axis;
            public NativeAnatomicalSource(Observation o){observation=o;axis=o.Geometry.ReferenceAxis(o.Chain[0],o.Chain[o.Chain.Count-1]);}
            public Vec3[] AnimatedVertices {get{return observation.Geometry.AnimatedVertices;}}
            public Vec3[] ReferenceVertices {get{return observation.Geometry.ReferenceVertices;}}
            public int[] Triangles {get{return observation.Geometry.SelectedTriangles;}}
            public Vec3 Root {get{return AnimationAdapter.ToCore(observation.Chain[0].position);}}
            public Vec3 MaterialAxis {get{return axis;}}
            public SourceContactBody Body {get{return observation.ContactBody;}}
            public SourceSectionRig CreateSectionRig(){return observation.Geometry.CreateSectionRig(observation.Chain[0],observation.Chain[observation.Chain.Count-1]);}
            public TriangleSurface Refit(SourceSectionRig rig,Vec3[] initializationOverride)
            {
                observation.SectionRig=rig;
                if(initializationOverride!=null)observation.Geometry.RefitContactPositions(initializationOverride);else observation.Geometry.RefitDeformation(rig);
                return observation.Geometry.ActiveContactSurface;
            }
        }
        void UpdateAnatomicalBody(PreparedBody body,int normalInterval,ref FrameMeasurement measurement)
        {
            Observation selected=null;
            foreach(var o in observations.Values)if(o.Target==body.Character&&Time.unscaledTime-o.LastSeen<=1&&o.Chain.Count>=2&&GuideTargetMatches(o.Agent.m_referenceTarget,body.Character.transform,requireMatchedGuideTarget.Value)){selected=o;break;}
            if(reactiveRig.Value){UpdateReactiveBody(body,selected,normalInterval,ref measurement);return;}
            long started=Stopwatch.GetTimestamp();body.ReferenceAnimation.Capture(null);
            var animation=body.Animation;
            if(selected!=null)
            {
                if(selected.AnatomicalSource==null)selected.AnatomicalSource=new NativeAnatomicalSource(selected);
                if(body.Anatomical==null)
                {
                    body.Anatomical=new AnatomicalContactSession(channelDefinition,supportDefinition,channelTransfer,channelProfile,channelContours,selected.Geometry.AnimatedVertices.Length);
                    body.Solver=body.Anatomical.Solver;
                    log.LogInfo("NEXT_RECEIVER_KERNEL active; exact channel walls="+channelDefinition.ConfinementTriangles.Length/3+"; physicalControls="+channelDefinition.Rest.Length+"; supportControls="+supportDefinition.Rest.Length);
                }
                body.Anatomical.RecoverySeconds=Math.Max(0,contactSettings.RecoveryTime);
                body.Anatomical.Step(animation.CapturedLinear,animation.CapturedTranslation,animation.CapturedMorphs,selected.AnatomicalSource,contactSettings,Time.deltaTime,body.ReferenceAnimation.PosedSize,pathMode.Value==PathMode.SourceFollowsPath);
                Array.Copy(body.Anatomical.StableAnchors,animation.Anchors,animation.Anchors.Length);
            }
            else if(body.Anatomical!=null)body.Anatomical.Release(animation.CapturedLinear,animation.CapturedTranslation,animation.CapturedMorphs,Math.Max(0,Time.deltaTime));
            long apply=Stopwatch.GetTimestamp();var offsets=body.Anatomical==null?body.EmptyAnatomicalOffsets:body.Anatomical.WorldOffsets;
            body.Binding.ApplyControlOffsets(offsets,body.ReferenceAnimation.PosedSize*.6,normalInterval,transition.Blend);body.SuccessfulFrames++;
            measurement.Solve+=(apply-started)*1000.0/Stopwatch.Frequency;measurement.Apply+=(Stopwatch.GetTimestamp()-apply)*1000.0/Stopwatch.Frequency;
            measurement.Vertices+=body.Solver.Positions.Length;measurement.Contacts+=body.Solver.ContactCount;
            if(body.Anatomical!=null&&body.Anatomical.Ready&&body.SuccessfulFrames>=3&&transition.Blend>=.999&&!body.PoseVerified)
            {
                double error=body.Binding.VerifyAppliedControlOffsets(offsets);body.PoseVerified=error<Math.Max(1e-4,body.ReferenceAnimation.PosedSize*1e-4);
                log.LogInfo("NEXT_RECEIVER_NATIVE mappingError="+error+"; verified="+body.PoseVerified);
            }
            if(debugTrajectory.Value&&body.Anatomical!=null){var path=body.Anatomical.Path;if(path!=null)for(int i=1;i<path.Count;i++)debugOverlay.Line(AnimationAdapter.ToUnity(path[i-1]),AnimationAdapter.ToUnity(path[i]),Color.green);}
        }
        static Linear3 SourceFrame(Transform t)
        {return new Linear3(AnimationAdapter.ToCore(t.right),AnimationAdapter.ToCore(t.up),AnimationAdapter.ToCore(t.forward));}
        static Vec3 RigSkin(SimulationDefinition d,int i,AnimationAdapter animation)
        {
            Vec3 p=d.Rest[i];for(int m=0;m<d.Morphs.Length;m++)p+=d.Morphs[m].Deltas[i]*animation.CapturedMorphs[m];Vec3 q=new Vec3();for(int j=d.InfluenceOffsets[i];j<d.InfluenceOffsets[i+1];j++){int b=d.BoneIndices[j];q+=(animation.CapturedLinear[b].Multiply(p)+animation.CapturedTranslation[b])*d.BoneWeights[j];}return q;
        }
        void UpdateReactiveBody(PreparedBody body,Observation selected,int normals,ref FrameMeasurement measurement)
        {
            long start=Stopwatch.GetTimestamp();var animation=body.Animation;
            Vec3[] entrySource=selected==null?null:selected.SizedWorld;
            if(selected!=null)
            {
                body.SourceDimensions=selected.SourceDimensions;
            }
            if(selected!=null&&pathMode.Value==PathMode.SourceFollowsPath)
            {
                Vec3 entry=new Vec3();foreach(int i in channelContours.Boundaries[0])entry+=animation.Anchors[i]/channelContours.Boundaries[0].Length;
                var r1=channelContours.Contours[1];Vec3 next=new Vec3();for(int j=0;j<r1.Vertices.Length;j++)next+=animation.Anchors[r1.Vertices[j]]*r1.Weights[j];
                if(body.EntryFrame==null)body.EntryFrame=new OrientedEntryFrame(channelContours.Boundaries[0].Length);
                body.EntryFrame.Update(animation.Anchors,channelContours.Boundaries[0],entry,next,animation.Anchors[channelContours.Boundaries[0][0]]-entry);
                if(!selected.EntryTargetRig.Configured)selected.EntryTargetRig.Configure(selected.Geometry.CreateSectionRig(selected.Chain[0],selected.Chain[selected.Chain.Count-1]));
                if(selected.EntryTargetRig.Aim(selected.Chain[0],selected.Chain[1],entrySource,body.EntryFrame,Math.Max(0,Time.deltaTime),pathMaximumAngle.Value,pathAngularSpeed.Value,pathResponse.Value,contactSettings.MaxSourceDisplacement*.5))
                {selected.Geometry.Capture(true,true);PrepareSourceDimensions(selected);measurement.Capture+=selected.Geometry.CaptureMilliseconds;}
            }
            else if(selected!=null)selected.EntryTargetRig.Reset();

            if(selected!=null&&(body.ReactiveRig==null||body.RigOwner!=selected))
            {
                if(body.RigWorker!=null)body.RigWorker.Dispose();body.RigOwner=selected;body.RigHasOutput=false;
                var geometry=selected.Geometry;var sourceRig=geometry.CreateSectionRig(selected.Chain[0],selected.Chain[selected.Chain.Count-1]);
                body.ReactiveRig=new ReactiveReceiverRig(channelDefinition,supportDefinition,channelTransfer,channelContours,geometry.ReferenceVertices,geometry.SelectedTriangles,geometry.ReferenceAxis(selected.Chain[0],selected.Chain[selected.Chain.Count-1]),sourceRig,reactiveTransfer,reactiveChart);
                body.RigInput=new ReactiveRigInput(channelDefinition.BoneNames.Length,channelDefinition.Morphs.Length,geometry.AnimatedVertices.Length);body.RigOutput=new ReactiveRigOutput(supportDefinition.Rest.Length,channelDefinition.Rest.Length,geometry.AnimatedVertices.Length,body.ReactiveRig.SectionCount);body.RigOffsets=new Vec3[supportDefinition.Rest.Length];body.RigSourceOffsets=new Vec3[geometry.AnimatedVertices.Length];body.RigSourceDeformation=new SnapshotSourceDeformation(geometry.AnimatedVertices.Length);body.RigSourceRod=null;body.RigRebasedSource=new Vec3[geometry.AnimatedVertices.Length];if(body.EntranceContactRig!=null)body.EntranceContactRig.Dispose();body.EntranceContactRig=new NativeEntranceContactRig();
                if(backgroundRig.Value)body.RigWorker=new ReactiveRigWorker(body.ReactiveRig,channelDefinition.BoneNames.Length,channelDefinition.Morphs.Length,supportDefinition.Rest.Length,channelDefinition.Rest.Length,geometry.AnimatedVertices.Length);
                body.RigGuide=new ReactiveReceiverRig(channelDefinition,supportDefinition,channelTransfer,channelContours,geometry.ReferenceVertices,geometry.SelectedTriangles,geometry.ReferenceAxis(selected.Chain[0],selected.Chain[selected.Chain.Count-1]),geometry.CreateSectionRig(selected.Chain[0],selected.Chain[selected.Chain.Count-1]),reactiveTransfer,reactiveChart);
                body.RigGuideOutput=new ReactiveRigOutput(supportDefinition.Rest.Length,channelDefinition.Rest.Length,geometry.AnimatedVertices.Length,body.RigGuide.SectionCount);body.RigSourceRod=body.RigGuide.SourceRod;
                log.LogInfo("NEXT_REACTIVE_RIG sections="+body.ReactiveRig.SectionCount+" sourceLandmarks="+body.ReactiveRig.Landmarks.Sections.Length+" worker="+backgroundRig.Value+" original skin identities retained");
            }
            if(body.ReactiveRig==null)return;
            if(body.RigWorker!=null&&!backgroundRig.Value){body.RigWorker.Dispose();body.RigWorker=null;}
            else if(body.RigWorker==null&&backgroundRig.Value)body.RigWorker=new ReactiveRigWorker(body.ReactiveRig,channelDefinition.BoneNames.Length,channelDefinition.Morphs.Length,supportDefinition.Rest.Length,channelDefinition.Rest.Length,body.RigInput.Source.Length);
            var input=body.RigInput;Array.Copy(animation.CapturedLinear,input.Bones,input.Bones.Length);Array.Copy(animation.CapturedTranslation,input.Translation,input.Translation.Length);Array.Copy(animation.CapturedMorphs,input.Morph,input.Morph.Length);
            input.Frame=Time.frameCount;input.Dt=body.LastRigSubmitted>0?Math.Max(0,Time.unscaledTime-body.LastRigSubmitted):Math.Max(0,Time.unscaledDeltaTime);input.PosedSize=animation.PosedSize;input.MaxSource=Math.Max(0,contactSettings.MaxSourceDisplacement-(selected==null?0:selected.EntryTargetRig.MaximumDisplacement));input.MaxStrain=contactSettings.MaxTissueStrain;input.SourceCompliance=contactSettings.SourceCorrectionStrength;input.TissueStrength=contactSettings.TissueDeformationStrength;input.Compliance=contactSettings.TissueCompliance;input.Response=contactSettings.ContactDamping;input.Recovery=contactSettings.RecoveryTime;input.MaximumOffsetRatio=offset.Value;input.HasSource=selected!=null;input.SourceFollowsPath=pathMode.Value==PathMode.SourceFollowsPath;input.WholeSource=wholeSource.Value;input.CervixMaxDisplacement=Math.Max(0,Math.Min(.08,cervixMaxDisplacement.Value));input.CervixFollowStrength=Math.Max(0,Math.Min(1,cervixFollowStrength.Value));input.CanalLengthScale=Math.Max(1,Math.Min(1.5,canalLengthScale.Value));
            if(selected!=null){input.SourceRoot=AnimationAdapter.ToCore(selected.Chain[0].position);Array.Copy(selected.SizedWorld,input.Source,input.Source.Length);input.SourceFrame=SourceFrame(selected.Chain[0]);}
            bool updated=false;
            if(body.RigWorker!=null){if(body.RigWorker.Failure!=null)throw new InvalidOperationException(body.RigWorker.Failure);updated=body.RigWorker.TryRead(body.RigOutput);if(body.RigWorker.Submit(input))body.LastRigSubmitted=Time.unscaledTime;}else{body.ReactiveRig.Step(input,body.RigOutput);body.LastRigSubmitted=Time.unscaledTime;updated=true;}
            if(updated)body.RigHasOutput=true;if(!body.RigHasOutput)return;
            var result=body.RigOutput;int root=Array.IndexOf(channelDefinition.BoneNames,"MED_Pelvis");Linear3 oldInverse;if(!result.Carrier.TryInverse(out oldInverse))throw new InvalidOperationException("Singular published carrier");var current=animation.CapturedLinear[root];var currentTranslation=animation.CapturedTranslation[root];
            bool guideReady=selected!=null&&body.RigWorker!=null;
            if(guideReady)body.RigGuide.GuideSourceOnly(input,body.RigGuideOutput,result,Time.unscaledDeltaTime);
            if(selected==null&&result.RecoveryFraction>=1&&body.EntranceContactRig!=null)body.EntranceContactRig.Release();
            for(int i=0;i<body.RigOffsets.Length;i++)
            {
                // The procedural wall owns a coherent solved surface. Adding an
                // old offset to a newly folded, independently animated baseline
                // would reintroduce wrinkles that were absent in the worker.
                double ownership=result.ReceiverActive||result.RecoveryFraction<1&&result.Offsets[i].LengthSquared>1e-20?body.ReactiveRig.ReceiverOwnership[i]:0;
                if(ownership<=0&&result.Offsets[i].LengthSquared<1e-20){body.RigOffsets[i]=new Vec3();continue;}
                Vec3 native=ownership>0?(guideReady?body.RigGuide.NativeSupportPoint(i):RigSkin(supportDefinition,i,animation)):new Vec3();
                body.RigOffsets[i]=ReceiverPoseRebase.Offset(result.FullWorld[i],result.Offsets[i],native,oldInverse,result.CarrierTranslation,current,currentTranslation,ownership);
            }
            for(int i=0;i<result.Physical.Length;i++)body.Solver.Positions[i]=animation.Anchors[i]+body.RigOffsets[channelTransfer.PhysicalIndices[i]];
            body.PathWorld=result.Path;body.PathFrame=result.Frame;
            if(selected!=null)
            {
                if(body.RigWorker!=null)
                {
                    Array.Copy(body.RigGuideOutput.SourceWorld,body.RigRebasedSource,body.RigRebasedSource.Length);
                }
                else
                {
                    body.RigSourceRod.Rebase(input.Source,input.SourceRoot,result.SourceAnimatedCentres,result.SourceSolvedCentres,result.SourceFrame,input.SourceFrame,input.MaxSource);
                    body.RigSourceRod.MapVertices(input.Source,body.RigRebasedSource);
                }
                body.EntranceContactRig.Synchronize(selected.TargetAgent,selected.Agent,selected.Chain[0],body.RigSourceRod,body.RigRebasedSource,LegacyEntranceDynamicsForAudit);
                for(int i=0;i<body.RigSourceOffsets.Length;i++)body.RigSourceOffsets[i]=body.ReactiveRig.IsSourceAttachment(i)?new Vec3():body.RigRebasedSource[i]-selected.Geometry.AnimatedVertices[i];
                body.RigSourceDeformation.Capture(body.RigSourceOffsets,contactSettings.MaxSourceDisplacement+body.SourceDimensions.MaximumOffset);selected.ContactBody.Offset=new Vec3();selected.ContactBody.RotationVector=new Vec3();selected.ContactBody.PreserveContactAcrossDeformation=true;selected.ContactBody.ApplyDeformation(body.RigSourceDeformation);if(result.Contacts>0)selected.ContactBody.MarkContact();
                selected.CorrectionFrame=Time.frameCount;
            }
            long apply=Stopwatch.GetTimestamp();body.Binding.ApplyControlOffsets(body.RigOffsets,input.PosedSize*offset.Value,normals,transition.Blend);body.SuccessfulFrames++;measurement.Solve+=(apply-start)*1000.0/Stopwatch.Frequency;measurement.Apply+=(Stopwatch.GetTimestamp()-apply)*1000.0/Stopwatch.Frequency;measurement.Vertices+=result.Physical.Length;measurement.Contacts+=result.Contacts;
            if(!body.PoseVerified&&body.SuccessfulFrames>=3&&transition.Blend>=.999){double error=body.Binding.VerifyAppliedControlOffsets(body.RigOffsets);body.PoseVerified=error<1e-4;log.LogInfo("NEXT_REACTIVE_NATIVE mappingError="+error+" verified="+body.PoseVerified+" resultAge="+(Time.frameCount-result.Frame));}
            if(debugTrajectory.Value)for(int i=1;i<result.Path.Length;i++)debugOverlay.Line(AnimationAdapter.ToUnity(result.Path[i-1]),AnimationAdapter.ToUnity(result.Path[i]),Color.green);
            if(debugReceiverLandmarks.Value)
            {
                if(result.TerminalAvailable)
                {
                    if(debugOverlay==null)debugOverlay=new ContactDebugOverlay();
                    var rn=AnimationAdapter.ToUnity(result.TerminalCentre);var normal=AnimationAdapter.ToUnity(result.TerminalNormal);
                    debugOverlay.Line(rn,rn+normal*(float)(input.PosedSize*.06),Color.white);
                    debugOverlay.Line(rn,rn+AnimationAdapter.ToUnity(result.CervicalMotionAxis)*(float)(input.PosedSize*.09),Color.blue);
                    for(int k=1;k<5;k++)debugOverlay.Line(rn,AnimationAdapter.ToUnity(result.TerminalPoints[k]),Color.magenta);
                }
                if(debugOverlay==null)debugOverlay=new ContactDebugOverlay();float size=(float)(input.PosedSize*.004);
                for(int i=0;i<result.ReceiverLandmarkCount;i++)
                {
                    var c=AnimationAdapter.ToUnity(result.ReceiverLandmarkPoints[i*5]);
                    if(i>0)debugOverlay.Line(AnimationAdapter.ToUnity(result.ReceiverLandmarkPoints[(i-1)*5]),c,Color.yellow);
                    for(int k=1;k<=4;k++){var p=AnimationAdapter.ToUnity(result.ReceiverLandmarkPoints[i*5+k]);debugOverlay.Line(c,p,k<3?Color.magenta:Color.cyan);debugOverlay.Line(p-Vector3.up*size,p+Vector3.up*size,Color.white);}
                    debugOverlay.Line(c,AnimationAdapter.ToUnity(result.ReceiverSourcePairs[i]),new Color(.3f,1,.3f));
                }
                var e=AnimationAdapter.ToUnity(result.ExternalLandmarkPoints[0]);for(int k=1;k<=4;k++)debugOverlay.Line(e,AnimationAdapter.ToUnity(result.ExternalLandmarkPoints[k]),Color.blue);
            }
            if(debugLandmarks.Value){if(debugOverlay==null)debugOverlay=new ContactDebugOverlay();float size=(float)(input.PosedSize*.006);for(int i=0;i<result.LandmarkCount;i++){var p=AnimationAdapter.ToUnity(result.LandmarkPoints[i]);debugOverlay.Line(p-Vector3.right*size,p+Vector3.right*size,Color.yellow);debugOverlay.Line(p-Vector3.up*size,p+Vector3.up*size,Color.yellow);}}
        }
    }
}
