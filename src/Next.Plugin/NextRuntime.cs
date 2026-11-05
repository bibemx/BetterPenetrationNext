using System;
using System.Collections.Generic;
using System.Diagnostics;
using BepInEx.Configuration;
using BepInEx.Logging;
using BetterPenetrationNext.Core;
using Core_BetterPenetration;
using HarmonyLib;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace BetterPenetrationNext
{
    public enum NextMode { Original,Hybrid,SoftBody }
    public enum SourceGuideShape { Rigid,FlexibleRod,SectionRig }
    [DefaultExecutionOrder(32000)]
    public sealed partial class NextRuntime : MonoBehaviour
    {
        sealed class PreparedBody : IDisposable
        {
            public ChaControl Character;
            public MeshDeformationBinding Binding;
            public AnimationAdapter Animation;
            public SoftBodySolver Solver;
            public bool AnatomicalConfigured;
            public AnatomicalContactSession Anatomical;
            public ReactiveRigWorker RigWorker;
            public ReactiveReceiverRig ReactiveRig;
            public ReactiveRigInput RigInput;
            public ReactiveRigOutput RigOutput;
            public ReactiveReceiverRig RigGuide;public ReactiveRigOutput RigGuideOutput;
            public Vec3[] RigOffsets,RigSourceOffsets;
            public SnapshotSourceDeformation RigSourceDeformation;
            public NativeEntranceContactRig EntranceContactRig;
            public OrientedEntryFrame EntryFrame;
            public InextensibleSourceRig RigSourceRod; public Vec3[] RigRebasedSource;
            public SourceDimensions SourceDimensions;
            public Observation RigOwner;
            public bool RigHasOutput;
            public double LastRigSubmitted;
            public AnimationAdapter ReferenceAnimation;
            public Vec3[] EmptyAnatomicalOffsets;
            public PreparedMaterials Materials;
            public Func<Mesh> Snapshot;
            public int SuccessfulFrames;
            public bool PoseVerified;
            public AnatomicalPathConstraintSystem Path;
            public Matrix4x4 PathMatrix;
            public Vec3[] PathWorld;
            public PathProfile PathProfile;
            public PathProfile RejectedPathProfile;
            public int PathFrame=-1;
            public void Dispose() { if(RigWorker!=null)RigWorker.Dispose();if(EntranceContactRig!=null)EntranceContactRig.Dispose();Binding.Dispose(); if(Materials!=null) Materials.Dispose(); }
        }
        sealed class Observation : IDisposable
        {
            public readonly SourceEntryTargetRig EntryTargetRig=new SourceEntryTargetRig();
            public DanAgent Agent;public CollisionAgent TargetAgent;
            public ChaControl Source,Target;
            public ContactDetection Geometry;
            public double LastSeen;
            public Func<Mesh> Snapshot;
            public double NextVisibilityCheck;
            public List<Transform> Chain;
            public bool ReportedUnwelded;
            public SourceContactBody ContactBody;
            public double PathParameter=-1;
            public readonly PathGuidanceController Guidance=new PathGuidanceController();
            public readonly SourceRodGuide RodGuide=new SourceRodGuide();
            public SourceSectionRig SectionRig;
            public SourceDimensions SourceDimensions;
            public Vec3[] SizedWorld,DimensionOffsets;
            public SnapshotSourceDeformation DimensionDeformation;
            public int DimensionFrame=-1,CorrectionFrame=-1;
            public NativeAnatomicalSource AnatomicalSource;
            public void Dispose() { EntryTargetRig.Dispose();if(Geometry!=null) Geometry.Dispose(); }
        }
        public static NextRuntime Instance { get; private set; }
        ConfigEntry<NextMode> mode;
        ConfigEntry<PhysicsQuality> quality;
        ConfigEntry<int> iterations,substeps,normals;
        ConfigEntry<float> hz,stretch,bend,shape,damping,offset,recovery,maximumAdaptiveStep;
        ConfigEntry<float> transitionSeconds,speed,margin,query,pinCompliance;
        ConfigEntry<ContactMode> contactMode;
        ConfigEntry<PathMode> pathMode;
        ConfigEntry<bool> requireMatchedGuideTarget;
        ConfigEntry<float> tissueCompliance,sourceStrength,tissueStrength,maxSource,maxStrain,contactDamping,recoveryTime;
        ConfigEntry<bool> debugContacts,debugTrajectory;
        ConfigEntry<string> pathFile,regionFile;
        ConfigEntry<bool> seamProxy,activeContactList,triangleHints,releaseWhenSeparated;
        ConfigEntry<float> maximumSeamGap,pathResponse,pathMaximumAngle,pathAngularSpeed;
        ConfigEntry<SourceGuideShape> sourceGuideShape;
        ConfigEntry<float> rodBudgetFraction,pathContactTravel;
        readonly BidirectionalContactSettings contactSettings=new BidirectionalContactSettings();
        [System.Runtime.Serialization.DataContract] public sealed class PathProfile : AnimatedPathProfile { }
        PathProfile pathProfile;
        ContactDebugOverlay debugOverlay;
        string loadedPath,loadedRegions;
        bool contactConfigurationChanging;
        ModeTransition transition;
        ConfigEntry<string> packagePath;
        ConfigEntry<bool> display;
        ConfigEntry<bool> xrayEnabled,dots,includeHiddenSource;
        ConfigEntry<XRayTarget> xrayTarget;
        ConfigEntry<OcclusionMode> occlusion;
        ConfigEntry<Color> sourceColor,receiverColor;
        ConfigEntry<float> sourceOpacity,receiverOpacity,textureBlend,dotScale,dotDensity;
        readonly Dictionary<string,ConfigEntry<float>> morphSettings=new Dictionary<string,ConfigEntry<float>>();
        ConfigFile configuration;
        readonly ReleaseDefaults defaults=ReleaseDefaults.LoadEmbedded();
        internal ReleaseDefaults DefaultProfile { get { return defaults; } }
        XRayRenderer xray;
        bool xrayAttempted;
        readonly PerformanceWindow performance=new PerformanceWindow();
        double lastPerformanceLog;
        int previousCollections;
        ConfigEntry<bool> logPerformance;
        ConfigEntry<bool> autoSnapshot,exportSnapshot;
        bool snapshotPending;
        double snapshotAt;
        readonly Dictionary<object,double> registrationRetry=new Dictionary<object,double>();
        readonly Dictionary<object,double> registrationWarning=new Dictionary<object,double>();
        readonly Dictionary<int,PreparedBody> bodies=new Dictionary<int,PreparedBody>();
        readonly Dictionary<object,Observation> observations=new Dictionary<object,Observation>();
        readonly List<int> retireBodies=new List<int>(); readonly List<object> retireContacts=new List<object>();
        readonly CompositeSurface composite=new CompositeSurface();
        readonly List<LocalContactSurface> regions=new List<LocalContactSurface>();
        readonly SoftBodySettings settings=new SoftBodySettings();
        SimulationDefinition definition;
        ManualLogSource log;
        double nextDiscovery;
        bool dirty;
        Harmony hooks;
        static readonly System.Reflection.FieldInfo MaleField=AccessTools.Field(typeof(Lookat_dan),"male");
        public static ChaControl SourceCharacter(Lookat_dan value) { return value==null||MaleField==null?null:MaleField.GetValue(value) as ChaControl; }
        public NextMode Mode { get { return transition==null?NextMode.Original:(NextMode)transition.Current; } }
        public double LastCpuMilliseconds { get; private set; }
        public int RegisteredBodies { get { return bodies.Count; } }
        public string Status { get; private set; }="Initializing";
        internal int VerifyDefaultConfiguration() { return defaults.VerifyEffectiveConfiguration(configuration,false); }
        public void Initialize(ConfigFile config,ManualLogSource logger)
        {
            Instance=this; log=logger; configuration=config;
            log.LogInfo("Next defaults: "+defaults.Profile.Id+"; status="+defaults.Profile.Status+"; SHA256="+defaults.Sha256);
            mode=defaults.Bind<NextMode>(config,"Next","Mode","Original preserves upstream; Hybrid adds prepared surface; SoftBody additionally suspends audited target surface controllers only while physics is verified.");
            transition=new ModeTransition((int)mode.Value); transitionSeconds=defaults.Bind<float>(config,"Next","TransitionSeconds","Fade out physical displacement before mode changes, then fade in. Original source constraints remain active.");
            quality=defaults.Bind<PhysicsQuality>(config,"Next.Performance","Quality","Performance / Balanced / Quality / Custom.");
            iterations=defaults.Bind<int>(config,"Next.Performance","Iterations","Custom solver iterations, 1..32."); substeps=defaults.Bind<int>(config,"Next.Performance","MaximumSubsteps","Custom maximum catch-up substeps, 1..8.");
            hz=defaults.Bind<float>(config,"Next.Performance","FixedHz","Custom fixed simulation rate, 30..240 Hz."); normals=defaults.Bind<int>(config,"Next.Performance","NormalsInterval","Custom normal update interval.");
            maximumAdaptiveStep=defaults.Bind<float>(config,"Next.Performance","MaximumAdaptiveStep","Maximum longer XPBD step when the fixed-step budget overflows, in seconds (0 disables; maximum 0.1). Does not increase the substep or iteration budget.");
            stretch=defaults.Bind<float>(config,"Next.Physics","StretchCompliance","Stretch compliance."); bend=defaults.Bind<float>(config,"Next.Physics","BendCompliance","Opposite-vertex bending surrogate compliance.");
            shape=defaults.Bind<float>(config,"Next.Physics","ShapeCompliance","Animated rest-shape tether compliance."); damping=defaults.Bind<float>(config,"Next.Physics","Damping","Velocity damping per second."); offset=defaults.Bind<float>(config,"Next.Physics","MaximumOffsetRatio","Offset cap relative to current simulation size.");
            recovery=defaults.Bind<float>(config,"Next.Physics","RecoverySeconds","Nominal gradual recovery time after contact ends, in game seconds. Zero restores the original elastic response; this does not hold a frozen shape or delay new contacts.");
            contactMode=defaults.Bind<ContactMode>(config,"Next.Contact","ContactMode","Original / OneWay / Bidirectional. Changes retain the legacy Next Mode control and fade through zero.");
            pathMode=defaults.Bind<PathMode>(config,"Next.Contact","PathMode","Disabled / SourceFollowsPath / AnatomyConforms. Requires explicit path points for path guidance.");
            anatomicalReceiver=defaults.Bind<bool>(config,"Next.Contact","AnatomicalReceiver","Use the coupled material channel and surrounding-tissue rig for the supported prepared receiver.");
            if(GetComponent<SourceEntryPoseRestore>()==null)gameObject.AddComponent<SourceEntryPoseRestore>();
            reactiveRig=defaults.Bind<bool>(config,"Next.Contact","ReactiveRig","Use cached adaptive landmarks and reduced section rig. Disable to retain the reference backend.");
            backgroundRig=defaults.Bind<bool>(config,"Next.Performance","BackgroundRig","Portable numerical solve on one bounded worker; Unity capture/upload remain on main thread.");
            wholeSource=defaults.Bind<bool>(config,"Next.Contact","WholeSourceContainment","Separate bounded admission trajectory and whole-source envelope, including material behind the first bone.");
            cervixMaxDisplacement=defaults.Bind<float>(config,"Next.Receiver","CervixMaxDisplacement",new ConfigDescription("Maximum LOCAL cervical compression, additionally limited by the actual cervical depth. Uterus and contact-driven canal length stay fixed. Zero locks compression; this is a visual model, not a physiological coefficient.",new AcceptableValueRange<float>(0f,.08f)));
            cervixFollowStrength=defaults.Bind<float>(config,"Next.Receiver","CervixFollowStrength",new ConfigDescription("Local cervical response to tip pressure, 0..1, bounded by available cervical depth. No rigid organ motion or tip attachment. Inactive recovery uses RecoveryTime.",new AcceptableValueRange<float>(0f,1f)));
            sourceLengthScale=defaults.Bind<float>(config,"Next.Source","SourceLengthScale",new ConfigDescription("Manual length multiplier relative to the card's animated source, 0.5..1.5. 1 uses the card size. Applied before contact and landmarks; attached base stays fixed. Changes blend smoothly.",new AcceptableValueRange<float>(.5f,1.5f)));
            sourceWidthScale=defaults.Bind<float>(config,"Next.Source","SourceWidthScale",new ConfigDescription("Manual width multiplier relative to the card's animated source, 0.5..1.5. 1 uses the card size. Preserves the cross-section profile and original skin identities. Changes blend smoothly.",new AcceptableValueRange<float>(.5f,1.5f)));
            canalLengthScale=defaults.Bind<float>(config,"Next.Receiver","CanalLengthScale",new ConfigDescription("1..1.5: requested arc length relative to the current native centreline. Extension follows the anatomical cervical-to-cavity axis; the entry stays attached. This is a configured baseline length. Contact never adds extra canal length or rigid organ movement.",new AcceptableValueRange<float>(1f,1.5f)));
            debugReceiverLandmarks=defaults.Bind<bool>(config,"Next.Diagnostics","DebugReceiverLandmarks","Display real-surface R0-Rn section points, separate outer-entry group and spatial Source matches through the body.");
            debugLandmarks=defaults.Bind<bool>(config,"Next.Diagnostics","DebugLandmarks","Draw cached real-surface Source and Receiver landmarks independently of X-Ray materials.");
            reactiveRig.SettingChanged+=(s,e)=>dirty=true;backgroundRig.SettingChanged+=(s,e)=>dirty=true;wholeSource.SettingChanged+=(s,e)=>dirty=true;
            anatomicalReceiver.SettingChanged+=(s,e)=>dirty=true;pathMode.SettingChanged+=(s,e)=>dirty=true;
            contactMode.SettingChanged+=(s,e)=>dirty=true;
            requireMatchedGuideTarget=defaults.Bind<bool>(config,"Next.Contact","RequireMatchedGuideTarget","Apply the prepared channel guide only when the upstream animation target matches this receiver and channel. Keeps idle and unrelated targets from being pulled into the channel. Disable only for an explicitly controlled neutral guide fixture; geometric contact remains independent.");
            tissueCompliance=defaults.Bind<float>(config,"Next.Contact","TissueCompliance","XPBD contact compliance, numerical simulation units; not a physiological norm.");
            sourceStrength=defaults.Bind<float>(config,"Next.Contact","SourceCorrectionStrength","Source inverse mobility relative to tissue, zero pins animated source geometry.");
            tissueStrength=defaults.Bind<float>(config,"Next.Contact","TissueDeformationStrength","Tissue inverse mobility, zero prevents local contact displacement.");
            maxSource=defaults.Bind<float>(config,"Next.Contact","MaxSourceDisplacement","Maximum source correction in Unity world units, relative to this frame's animation.");
            maxStrain=defaults.Bind<float>(config,"Next.Contact","MaxTissueStrain","Maximum extension ratio of simulation edges relative to animated lengths.");
            contactDamping=defaults.Bind<float>(config,"Next.Contact","ContactDamping","Source correction release rate per second; tissue uses existing velocity damping.");
            recoveryTime=defaults.Bind<float>(config,"Next.Contact","RecoveryTime","Finite smooth visual recovery duration in seconds. Not a clinical measurement.");
            debugContacts=defaults.Bind<bool>(config,"Next.Contact","DebugContacts","Draw contact normals and computed source reaction as diagnostic lines.");
            debugTrajectory=defaults.Bind<bool>(config,"Next.Contact","DebugTrajectory","Draw explicit centreline guide; X-Ray is independent.");
            pathFile=defaults.Bind<string>(config,"Next.Contact","PathFile","Path JSON: legacy renderer-local Points or schema 2 bone/morph-bound controls. Empty uses the bundled guide only for the exact supported receiver signature. PathMode must still enable guidance.");
            regionFile=defaults.Bind<string>(config,"Next.Contact","RegionComplianceFile","Optional JSON array of nonnegative compliance per simulation particle, in metadata particle order.");
            seamProxy=defaults.Bind<bool>(config,"Next.Contact","AnimatedSeamProxy","Contact-only joining faces for uniquely matched rest-space cracks. Does not weld weights or alter rendered geometry; unmatched openings retain bounded shell queries.");
            maximumSeamGap=defaults.Bind<float>(config,"Next.Contact","MaximumSeamGapRatio","Maximum allowed animated crack gap / source reference size. Larger gaps safely fall back to the original shell.");
            activeContactList=defaults.Bind<bool>(config,"Next.Contact","ActiveContactList","Compact local candidates using the current contact representation bounds before expensive surface queries.");
            triangleHints=defaults.Bind<bool>(config,"Next.Contact","TriangleHints","Seed exact BVH queries with the previous triangle. Global BVH refinement remains active.");
            releaseWhenSeparated=defaults.Bind<bool>(config,"Next.Contact","ReleaseOnlyWhenSeparated","ContactDamping releases correction after contact ends; it does not erase active contact support on a slow frame. False restores 1.0.4 release semantics.");
            pathResponse=defaults.Bind<float>(config,"Next.Contact","PathResponse","Independent guide response rate per second.");
            pathMaximumAngle=defaults.Bind<float>(config,"Next.Contact","MaximumGuideAngle","Maximum guide angular correction, radians.");
            pathAngularSpeed=defaults.Bind<float>(config,"Next.Contact","GuideAngularSpeed","Maximum change in guide angle per second, radians.");
            sourceGuideShape=defaults.Bind<SourceGuideShape>(config,"Next.Contact","SourceGuideShape","SectionRig samples material sections from the actual evaluated animation. FlexibleRod is the legacy straight-axis approximation; Rigid preserves six-DOF motion. Original skin weights remain unchanged.");
            rodBudgetFraction=defaults.Bind<float>(config,"Next.Contact","RodGuideBudgetFraction","Fraction of MaxSourceDisplacement reserved for rod mapping, 0..1. The remainder is available for contact translation. Numerical visualization control, not a physiological constant.");
            pathContactTravel=defaults.Bind<float>(config,"Next.Contact","PathContactTravel","Maximum contact translation away from the guided pose, world units. Keeps contact from ejecting Source from the route; infeasible contacts remain visible in residual diagnostics.");
            contactMode.SettingChanged+=(s,e)=> {
                if(contactConfigurationChanging) return; contactConfigurationChanging=true;
                mode.Value=contactMode.Value==ContactMode.Original?NextMode.Original:mode.Value==NextMode.Original?NextMode.Hybrid:mode.Value;
                transition.Request((int)mode.Value); contactConfigurationChanging=false;
            };
            speed=defaults.Bind<float>(config,"Next.Physics","MaximumSpeedRatio","Maximum particle speed relative to posed simulation size."); margin=defaults.Bind<float>(config,"Next.Physics","ContactMarginRatio","Contact clearance relative to simulation size."); query=defaults.Bind<float>(config,"Next.Physics","ContactQueryRatio","BVH query reach relative to simulation size."); pinCompliance=defaults.Bind<float>(config,"Next.Physics","PinCompliance","Partial attachment compliance; exact hard pins remain fixed.");
            packagePath=defaults.Bind<string>(config,"Next.Assets","PreparedZipmod","Package containing the user-prepared visual mesh and simulation metadata."); display=defaults.Bind<bool>(config,"Next.Diagnostics","ShowStatus","Runtime status panel.");
            xrayEnabled=defaults.Bind<bool>(config,"Next.XRay","XRayEnabled","Independent hidden working-geometry pass. The external body submesh is excluded.");
            includeHiddenSource=defaults.Bind<bool>(config,"Next.XRay","IncludeHiddenSource","Draw the already selected Source even when its Renderer or hierarchy is hidden by the game. Camera layer masks still apply; other hidden surfaces remain hidden.");
            xrayTarget=defaults.Bind<XRayTarget>(config,"Next.XRay","XRayTarget","Source / Receiver / Both."); occlusion=defaults.Bind<OcclusionMode>(config,"Next.XRay","OcclusionMode","Only occluded fragments by default.");
            sourceColor=defaults.Bind<Color>(config,"Next.XRay","SourceColor","Source tint."); receiverColor=defaults.Bind<Color>(config,"Next.XRay","ReceiverColor","Internal prepared-region tint.");
            sourceOpacity=defaults.Bind<float>(config,"Next.XRay","SourceOpacity","Source opacity."); receiverOpacity=defaults.Bind<float>(config,"Next.XRay","ReceiverOpacity","Receiver opacity."); textureBlend=defaults.Bind<float>(config,"Next.XRay","OriginalTextureBlend","Preserve original albedo texture in the independent pass.");
            dots=defaults.Bind<bool>(config,"Next.XRay","DotPatternEnabled","Screen-space dot mask."); dotScale=defaults.Bind<float>(config,"Next.XRay","DotScale","Cell size in pixels."); dotDensity=defaults.Bind<float>(config,"Next.XRay","DotDensity","Fraction of populated cells.");
            xrayEnabled.SettingChanged+=(s,e)=>xrayAttempted=false;
            logPerformance=defaults.Bind<bool>(config,"Next.Diagnostics","LogPerformance","Log rolling mean/P95 CPU times and counts every ten seconds. GC collection counts are not allocated bytes.");
            autoSnapshot=defaults.Bind<bool>(config,"Next.Diagnostics","AutoSnapshot","Write read-only renderer/camera/solver diagnostics after source binding or a binding failure. No images or transform changes.");
            exportSnapshot=defaults.Bind<bool>(config,"Next.Diagnostics","ExportSnapshot","Set true to write a technical snapshot; automatically resets false."); exportSnapshot.SettingChanged+=(s,e)=> { if(exportSnapshot.Value) { snapshotPending=true; snapshotAt=Time.unscaledTime; } };
            mode.SettingChanged+=(s,e)=> { transition.Request((int)mode.Value); if(!contactConfigurationChanging) { contactConfigurationChanging=true; contactMode.Value=mode.Value==NextMode.Original?ContactMode.Original:contactMode.Value==ContactMode.Original?ContactMode.OneWay:contactMode.Value; contactConfigurationChanging=false; } }; packagePath.SettingChanged+=(s,e)=>dirty=true;
            if(contactMode.Value==ContactMode.Original&&mode.Value!=NextMode.Original) mode.Value=NextMode.Original;
            else if(contactMode.Value!=ContactMode.Original&&mode.Value==NextMode.Original) { mode.Value=NextMode.Hybrid; transition.Request((int)mode.Value); }
            try { UncensorIntegration.Initialize(); definition=UncensorIntegration.LoadDefinition(packagePath.Value); Status=definition==null?"Awaiting user-prepared model package":"Prepared model metadata loaded"; }
            catch(Exception e) { Status="Prepared model rejected: "+e.Message; log.LogWarning(Status); }
            SceneManager.sceneUnloaded+=SceneUnloaded;
            hooks=new Harmony("org.betterpenetration.next.simulation");
            var setTarget=AccessTools.DeclaredMethod(typeof(DanAgent),"SetDanTarget",new[]{typeof(CollisionAgent),typeof(bool)});
            if(setTarget==null) throw new MissingMethodException("Forked animation contract differs.");
            hooks.Patch(setTarget,postfix:new HarmonyMethod(typeof(NextRuntime),nameof(TargetPostfix)));
            hooks.Patch(AccessTools.DeclaredMethod(typeof(DanAgent),"ConstrainDan"),prefix:new HarmonyMethod(typeof(NextRuntime),nameof(SourceContactPrefix)));
            hooks.Patch(AccessTools.DeclaredMethod(typeof(DanAgent),"GetSquishedDanLength"),prefix:new HarmonyMethod(typeof(NextRuntime),nameof(SourceLengthPrefix)));
            hooks.Patch(AccessTools.DeclaredMethod(typeof(DanAgent),"GetSquishedDanGirth"),prefix:new HarmonyMethod(typeof(NextRuntime),nameof(SourceGirthPrefix)));
            var controller=Type.GetType("KK_Plugins.UncensorSelector+UncensorSelectorController, KKS_UncensorSelector",false);
            var reload=AccessTools.DeclaredMethod(controller,"ReloadCharacterBody");
            if(reload!=null) hooks.Patch(reload,prefix:new HarmonyMethod(typeof(NextRuntime),nameof(ReloadPrefix)) { priority=Priority.First });
            hooks.Patch(AccessTools.DeclaredMethod(typeof(CoreGame),"InitializeDanAgents"),postfix:new HarmonyMethod(typeof(NextRuntime),nameof(SourceAgentsReadyPostfix)));
            RegisterExternalMeshCompatibility();
            hooks.Patch(AccessTools.DeclaredMethod(typeof(CollisionAgent),"PullKokanBones"),prefix:new HarmonyMethod(typeof(NextRuntime),nameof(PullSurfacePrefix)));
            hooks.Patch(AccessTools.DeclaredMethod(typeof(CollisionAgent),"ReturnKokanBones"),prefix:new HarmonyMethod(typeof(NextRuntime),nameof(EntranceReturnPrefix)));
            hooks.Patch(AccessTools.DeclaredMethod(typeof(CollisionAgent),"PullAnaBone"),prefix:new HarmonyMethod(typeof(NextRuntime),nameof(PullBackSurfacePrefix)));
            InitializeControlWindow();
            log.LogInfo("Next runtime: "+Status+"; mode="+Mode+". Ready reactive bindings own source contact; Original/unbound sources retain upstream constraints.");
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-selftest")>=0) gameObject.AddComponent<NextNativeTests>();
        }
        static void TargetPostfix(DanAgent __instance,CollisionAgent __0) { Observe(__instance,__0); }
        static void SourceAgentsReadyPostfix(){if(Instance!=null)Instance.DiscoverSourceAgents();}
        static void ReloadPrefix(object __instance) { BeforeReload(__instance); }
        static bool PullSurfacePrefix(CollisionAgent __instance)
        { if(!ReactiveEntrancePrefix(__instance))return false; if(!BoneConstraintController.SuppressLegacySurface(__instance.m_collisionCharacter)) return true; __instance.ReturnKokanBones(); return false; }
        static bool PullBackSurfacePrefix(CollisionAgent __instance)
        { if(!BoneConstraintController.SuppressLegacySurface(__instance.m_collisionCharacter)) return true; __instance.ReturnAnaBones(); return false; }
        public bool UsesNativeEntrance {get{return reactiveRig!=null&&reactiveRig.Value&&UseAnatomicalReceiver;}}
        public bool PhysicsReady(ChaControl character)
        {
            PreparedBody body; return character!=null&&bodies.TryGetValue(character.GetInstanceID(),out body)&&body.Binding.Alive&&body.Binding.Visible&&body.PoseVerified&&body.SuccessfulFrames>=3&&body.Solver.SaturatedVertices==0&&body.Solver.LastFrameDroppedTime<0.05;
        }
        internal static void Observe(DanAgent source,CollisionAgent target)
        {
            var runtime=Instance; if(runtime==null||!runtime.isActiveAndEnabled||(runtime.Mode==NextMode.Original&&!runtime.xrayEnabled.Value)||source==null||target==null||source.m_danCharacter==null||target.m_collisionCharacter==null) return;
            double retry; if(runtime.registrationRetry.TryGetValue(source,out retry)&&Time.unscaledTime<retry) return;
            try { runtime.Record(source,target); runtime.registrationRetry.Remove(source); }
            catch(Exception e)
            {
                runtime.registrationRetry[source]=Time.unscaledTime+0.5; double warning;
                if(!runtime.registrationWarning.TryGetValue(source,out warning)||Time.unscaledTime>=warning)
                { runtime.registrationWarning[source]=Time.unscaledTime+5; runtime.log.LogWarning("Contact registration rejected; upstream continues: "+e.Message); if(runtime.autoSnapshot.Value) { runtime.snapshotPending=true; runtime.snapshotAt=Time.unscaledTime+0.2; } }
            }
        }
        void Record(DanAgent source,CollisionAgent target)
        {
            if(definition==null||UncensorIntegration.EffectiveGuid(target.m_collisionCharacter)!=definition.BodyGuid) return;
            var item=EnsureSource(source);if(item==null)return;
            item.Target=target.m_collisionCharacter;item.TargetAgent=target;item.LastSeen=Time.unscaledTime;BoneConstraintController.UpdateDynamicBones(target);
        }
        Observation EnsureSource(DanAgent source)
        {
            if(source==null||source.m_danCharacter==null||!source.m_danPointsFound||source.m_danPoints==null)return null;
            Observation item;
            observations.TryGetValue(source,out item); bool visibilityChanged=false;
            if(item!=null&&item.Geometry!=null&&!item.Geometry.Visible&&Time.unscaledTime>=item.NextVisibilityCheck)
            { item.NextVisibilityCheck=Time.unscaledTime+0.5; visibilityChanged=ContactDetection.HasVisibleCandidate(source.m_danCharacter,item.Chain,item.Geometry.Renderer); }
            if(item==null||item.Geometry==null||!item.Geometry.Alive||visibilityChanged)
            {
                var oldTarget=item==null?null:item.Target;var oldTargetAgent=item==null?null:item.TargetAgent;double oldSeen=item==null?-1000000:item.LastSeen;
                if(item!=null) { if(xray!=null) xray.Unregister(item); item.Dispose(); observations.Remove(source); }
                var chain=new List<Transform>(); foreach(var p in source.m_danPoints.danPoints) if(p.transform!=null) chain.Add(p.transform);
                var geometry=ContactDetection.FromChain(source.m_danCharacter,chain); if(geometry==null) return null;
                item=new Observation { Agent=source,TargetAgent=oldTargetAgent,Source=source.m_danCharacter,Target=oldTarget,LastSeen=oldSeen,Geometry=geometry,ContactBody=new SourceContactBody { Surface=geometry,PreserveContactAcrossDeformation=true },Snapshot=geometry.XRaySnapshot,Chain=chain };
                item.SourceDimensions=new SourceDimensions(geometry.CreateSectionRig(chain[0],chain[chain.Count-1]),sourceLengthScale.Value,sourceWidthScale.Value);
                item.SizedWorld=new Vec3[geometry.AnimatedVertices.Length];item.DimensionOffsets=new Vec3[item.SizedWorld.Length];item.DimensionDeformation=new SnapshotSourceDeformation(item.SizedWorld.Length);observations.Add(source,item);
                log.LogInfo("NEXT_SOURCE renderer="+geometry.Renderer.name+" mesh="+geometry.Renderer.sharedMesh.name+" kind="+geometry.Kind+" visible="+geometry.Visible+" triangles="+geometry.TriangleCount+" deformationGroups="+geometry.DeformationGroups+" unweldedFallback="+geometry.UsesUnweldedFallback);
                if(autoSnapshot.Value) { snapshotPending=true; snapshotAt=Time.unscaledTime+0.5; }
            }
            return item;
        }
        void DiscoverSourceAgents()
        {
            if(mode==null||Mode==NextMode.Original||CoreGame.danAgents==null)return;
            foreach(var source in CoreGame.danAgents)
            {
                double retry;if(source==null||registrationRetry.TryGetValue(source,out retry)&&Time.unscaledTime<retry)continue;
                try{EnsureSource(source);registrationRetry.Remove(source);}catch(Exception e){registrationRetry[source]=Time.unscaledTime+.5;log.LogWarning("NEXT_SOURCE_PRELOAD "+e.Message);}
            }
        }
        void PrepareSourceDimensions(Observation source)
        {
            double dt=source.DimensionFrame==Time.frameCount?0:Time.unscaledDeltaTime;
            source.SourceDimensions.Apply(source.Geometry.AnimatedVertices,AnimationAdapter.ToCore(source.Chain[0].position),sourceLengthScale.Value,sourceWidthScale.Value,dt,pathResponse.Value,source.SizedWorld);
            for(int i=0;i<source.DimensionOffsets.Length;i++)source.DimensionOffsets[i]=source.SizedWorld[i]-source.Geometry.AnimatedVertices[i];
            source.DimensionDeformation.Capture(source.DimensionOffsets,source.SourceDimensions.MaximumOffset);source.DimensionFrame=Time.frameCount;
        }
        internal static void BeforeReload(object controller)
        { if(Instance==null) return; var character=UncensorIntegration.Character(controller); if(character!=null) Instance.Retire(character.GetInstanceID()); }
        void Discover()
        {
            if(definition==null) return;
            retireBodies.Clear(); foreach(var entry in bodies) if(entry.Value.Character==null||!entry.Value.Binding.Alive||UncensorIntegration.EffectiveGuid(entry.Value.Character)!=definition.BodyGuid) retireBodies.Add(entry.Key);
            foreach(int key in retireBodies) Retire(key);
            foreach(var character in Resources.FindObjectsOfTypeAll<ChaControl>())
            {
                if(character==null||!character.gameObject.scene.IsValid()||UncensorIntegration.EffectiveGuid(character)!=definition.BodyGuid) continue;
                int id=character.GetInstanceID(); if(bodies.ContainsKey(id)) continue;
                SkinnedMeshRenderer visual=character.rendBody as SkinnedMeshRenderer; if(visual==null||visual.sharedMesh==null) continue;
                MeshDeformationBinding binding=null;
                PreparedMaterials materials=null;
                try
                {
                    materials=new PreparedMaterials(visual,packagePath.Value);
                    bool anatomical=UseAnatomicalReceiver;if(anatomical)LoadChannelResources();var physical=anatomical?channelDefinition:definition;
                    if(anatomical){int nativeInfluences=NativeSkinContract.Verify(physical,visual);nativeInfluences=Math.Max(nativeInfluences,NativeSkinContract.Verify(supportDefinition,visual));log.LogInfo("NEXT_NATIVE_SKIN verified complete influences; maximum="+nativeInfluences);}
                    binding=new MeshDeformationBinding(visual,anatomical?supportDefinition:definition); var animation=new AnimationAdapter(physical,visual); animation.Capture(null);
                    var solver=new SoftBodySolver(new SimulationTopology(physical.Rest,physical.Triangles,physical.AllowJunctions),physical.PinWeights); solver.Reset(animation.Anchors);
                    var body=new PreparedBody { Character=character,Binding=binding,Animation=animation,Solver=solver,Materials=materials,Snapshot=binding.XRaySnapshot,AnatomicalConfigured=anatomical,ReferenceAnimation=anatomical?new AnimationAdapter(definition,visual):null,EmptyAnatomicalOffsets=anatomical?new Vec3[supportDefinition.Rest.Length]:null }; bodies.Add(id,body); ApplyMorphSettings(body);
                    log.LogInfo("NEXT_BOUND "+visual.name+" simulationParticles="+physical.Rest.Length+" anatomical="+anatomical);
                }
                catch(Exception e) { if(binding!=null) binding.Dispose(); if(materials!=null) materials.Dispose(); log.LogWarning("Prepared binding rejected: "+e.Message); }
            }
        }
        void ApplyMorphSettings(PreparedBody body)
        {
            if(definition.Morphs==null) return;
            foreach(var morph in definition.Morphs)
            {
                ConfigEntry<float> setting;
                if(!morphSettings.TryGetValue(morph.Name,out setting))
                { setting=defaults.BindAssetDefault(configuration,"Next.PreparedMorphs",morph.Name,morph.DefaultWeight,"User-authored shape weight, 0..100. Blender drivers are replaced by this explicit control."); morphSettings.Add(morph.Name,setting); setting.SettingChanged+=(s,e)=> { foreach(var b in bodies.Values) ApplyMorphSettings(b); }; }
                int index=body.Binding.Original.GetBlendShapeIndex(morph.Name); if(index>=0) body.Binding.Renderer.SetBlendShapeWeight(index,Mathf.Clamp(setting.Value,0,100));
            }
        }
        void UpdateXray()
        {
            if(xray==null&&xrayEnabled.Value&&!xrayAttempted) { xrayAttempted=true; try { xray=new XRayRenderer(); } catch(Exception e) { log.LogWarning("X-Ray unavailable; physics continues: "+e.Message); } }
            if(xray==null) return;
            xray.Enabled=xrayEnabled.Value; xray.IncludeHiddenSource=includeHiddenSource.Value; xray.Target=xrayTarget.Value; xray.Occlusion=occlusion.Value; xray.SourceColor=sourceColor.Value; xray.ReceiverColor=receiverColor.Value;
            xray.SourceOpacity=sourceOpacity.Value; xray.ReceiverOpacity=receiverOpacity.Value; xray.OriginalTextureBlend=textureBlend.Value; xray.DotPatternEnabled=dots.Value; xray.DotScale=dotScale.Value; xray.DotDensity=dotDensity.Value;
            if(xray.Enabled) { foreach(var body in bodies.Values) xray.Register(body,body.Binding.Renderer,false,body.Snapshot); foreach(var o in observations.Values) xray.Register(o,o.Geometry.Renderer,true,o.Snapshot); }
            xray.Tick();
        }
        void LateUpdate()
        {
            if(mode==null) return; long started=Stopwatch.GetTimestamp(); var measurement=new FrameMeasurement();
            if(debugOverlay!=null) debugOverlay.Clear();
            if(debugOverlay==null&&(debugContacts.Value||debugTrajectory.Value)) debugOverlay=new ContactDebugOverlay();
            try
            {
                if(transition.Advance(Math.Max(0,Time.unscaledDeltaTime),Math.Max(0,transitionSeconds.Value))) { Clear(); nextDiscovery=0; }
                if(dirty)
                { Clear(); dirty=false; definition=UncensorIntegration.LoadDefinition(packagePath.Value);loadedPath=null;pathProfile=null; }
                if(Time.unscaledTime>=nextDiscovery) { nextDiscovery=Time.unscaledTime+0.5; DiscoverSourceAgents();Discover(); }
                UpdateXray();
                if(Mode==NextMode.Original) return;
                var policy=PhysicsPolicy.Resolve(quality.Value,iterations.Value,substeps.Value,hz.Value,normals.Value);
                double alpha=1-Math.Exp(-Math.Max(0,Time.deltaTime)*15);
                settings.StretchCompliance+=(stretch.Value-settings.StretchCompliance)*alpha; settings.BendCompliance+=(bend.Value-settings.BendCompliance)*alpha; settings.ShapeCompliance+=(shape.Value-settings.ShapeCompliance)*alpha; settings.Damping+=(damping.Value-settings.Damping)*alpha; settings.MaximumOffsetRatio=offset.Value;
                settings.MaximumSpeedRatio=speed.Value; settings.ContactMarginRatio=margin.Value; settings.ContactQueryRatio=query.Value; settings.PinCompliance=pinCompliance.Value;
                settings.RecoverySeconds=recovery.Value;
                settings.MaximumAdaptiveStep=maximumAdaptiveStep.Value;
                settings.PreserveVolume=definition!=null&&definition.PreserveVolume;
                contactSettings.Mode=contactMode.Value; contactSettings.TissueCompliance=tissueCompliance.Value; contactSettings.SourceCorrectionStrength=sourceStrength.Value;
                contactSettings.TissueDeformationStrength=tissueStrength.Value; contactSettings.MaxSourceDisplacement=maxSource.Value; contactSettings.MaxTissueStrain=maxStrain.Value;
                contactSettings.ContactDamping=contactDamping.Value; contactSettings.RecoveryTime=recoveryTime.Value;
                contactSettings.UseActiveContactList=activeContactList.Value; contactSettings.UseTriangleHints=triangleHints.Value;
                contactSettings.ProfileStages=logPerformance.Value;
                if(loadedRegions!=regionFile.Value) { var regionsValue=string.IsNullOrWhiteSpace(regionFile.Value)?null:JsonCodec.Deserialize<double[]>(System.IO.File.ReadAllText(regionFile.Value)); contactSettings.RegionCompliance=regionsValue; loadedRegions=regionFile.Value; }
                if(loadedPath!=pathFile.Value) {
                    PathProfile profile=null;
                    try {
                    if(!string.IsNullOrWhiteSpace(pathFile.Value)) {
                        var file=new System.IO.FileInfo(pathFile.Value); if(file.Length>4*1024*1024) throw new ArgumentException("PathFile exceeds 4 MiB.");
                        profile=JsonCodec.Deserialize<PathProfile>(System.IO.File.ReadAllText(file.FullName));
                    } else {
                        using(var stream=typeof(NextRuntime).Assembly.GetManifestResourceStream("BetterPenetrationNext.receiver-path.json"))
                            if(stream!=null) using(var reader=new System.IO.StreamReader(stream)) {
                                var candidate=JsonCodec.Deserialize<PathProfile>(reader.ReadToEnd());
                                if(definition!=null&&candidate.VisualMeshSignature==definition.VisualMeshSignature) profile=candidate;
                            }
                    }
                    } catch(Exception e) { log.LogWarning("NEXT_PATH file disabled; contact remains active: "+e.Message); }
                    pathProfile=profile; loadedPath=pathFile.Value;
                }
                retireBodies.Clear(); retireContacts.Clear();
                foreach(var item in observations)
                {
                    var o=item.Value;
                    if(o.Source==null||o.Agent.m_danCharacter!=o.Source||!o.Geometry.Alive) { retireContacts.Add(item.Key); continue; }
                    bool fresh=o.Target!=null&&Time.unscaledTime-o.LastSeen<=1;
                    if(!fresh){o.Target=null;o.TargetAgent=null;o.EntryTargetRig.Reset();}
                    bool sizing=sourceLengthScale.Value!=1||sourceWidthScale.Value!=1||o.SourceDimensions.MaximumOffset>1e-10;
                    if(fresh||sizing) try
                    {
                        o.Geometry.EnableSeamProxy=seamProxy.Value; o.Geometry.MaximumSeamGapRatio=maximumSeamGap.Value;
                        o.Geometry.Capture(false,true); measurement.Capture+=o.Geometry.CaptureMilliseconds;
                        PrepareSourceDimensions(o);
                        o.ContactBody.ReleaseOnlyWhenSeparated=releaseWhenSeparated.Value;
                        o.ContactBody.BeginFrame(contactSettings,Time.deltaTime,pathMode.Value==PathMode.AnatomyConforms);
                        if(o.Geometry.UsesUnweldedFallback&&!o.ReportedUnwelded) { o.ReportedUnwelded=true; log.LogWarning("NEXT_SOURCE uses actual unwelded triangles after seam separation; source binding and X-Ray retained."); if(autoSnapshot.Value) { snapshotPending=true; snapshotAt=Time.unscaledTime+0.2; } }
                    } catch(Exception e) { log.LogWarning(e.Message); retireContacts.Add(item.Key); }
                }
                foreach(var key in retireContacts) { if(xray!=null) xray.Unregister(observations[key]); observations[key].Dispose(); observations.Remove(key); }
                foreach(var item in bodies)
                {
                    var body=item.Value;
                    try
                    {
                        if(body.Character==null||!body.Binding.Alive||UncensorIntegration.EffectiveGuid(body.Character)!=definition.BodyGuid) { retireBodies.Add(item.Key); continue; }
                        long captureStart=Stopwatch.GetTimestamp(); body.Animation.Capture(body.AnatomicalConfigured?null:body.Solver); measurement.Capture+=(Stopwatch.GetTimestamp()-captureStart)*1000.0/Stopwatch.Frequency;
                        if(body.AnatomicalConfigured){UpdateAnatomicalBody(body,policy.NormalsInterval,ref measurement);continue;}
                        SoftBodySolver.LocalGuardEnabled=false;SoftBodySolver.IncrementalMaterialGuard=false;SoftBodySolver.ContinuousMaterialSelfContact=false;SoftBodySolver.TransferTissueSelfReaction=false;
                        composite.Surfaces.Clear(); int index=0;
                        foreach(var observation in observations.Values) if(observation.Target==body.Character&&Time.unscaledTime-observation.LastSeen<=1)
                        {
                            if(index==regions.Count) regions.Add(new LocalContactSurface()); var region=regions[index++]; region.Surface=contactMode.Value==ContactMode.Bidirectional?(IContactSurface)observation.ContactBody:observation.Geometry; region.Enabled=true;
                            region.Center=observation.Agent.m_referenceTarget!=null?AnimationAdapter.ToCore(observation.Agent.m_referenceTarget.position):body.Animation.Anchors[0];
                            region.Radius=body.Animation.PosedSize; region.FullStrengthRadiusFraction=0.9; composite.Surfaces.Add(region);
                            bool rod=false,sectionRig=false,refitted=false;
                            if(contactMode.Value==ContactMode.Bidirectional&&pathMode.Value==PathMode.SourceFollowsPath&&GuideTargetMatches(observation.Agent.m_referenceTarget,body.Character.transform,requireMatchedGuideTarget.Value)) {
                                var path=ResolvePath(body);
                                if(path!=null&&observation.Chain.Count>=2) {
                                    Vec3 point=AnimationAdapter.ToCore(observation.Chain[observation.Chain.Count-1].position),forward=point-AnimationAdapter.ToCore(observation.Chain[0].position);
                                    if(sourceGuideShape.Value==SourceGuideShape.SectionRig) {
                                        Vec3 root=AnimationAdapter.ToCore(observation.Chain[0].position);
                                        if(observation.SectionRig==null)observation.SectionRig=observation.Geometry.CreateSectionRig(observation.Chain[0],observation.Chain[observation.Chain.Count-1]);
                                        observation.SectionRig.Capture(path,observation.Geometry.AnimatedVertices,root,Time.deltaTime,contactSettings.MaxSourceDisplacement*Math.Max(0,Math.Min(1,rodBudgetFraction.Value)),pathResponse.Value);
                                        observation.Geometry.RefitDeformation(observation.SectionRig);refitted=true;
                                        observation.ContactBody.ApplyDeformation(observation.SectionRig);observation.ContactBody.TranslationAxis=path.Closest(observation.SectionRig.CorrectedCentre(observation.SectionRig.SectionCount-1)).Tangent;observation.PathParameter=observation.SectionRig.Parameter;observation.Guidance.Reset();sectionRig=true;
                                    } else if(sourceGuideShape.Value==SourceGuideShape.FlexibleRod) {
                                        Vec3 root=point-forward;point=observation.Geometry.LeadingPoint(root,point);
                                        observation.RodGuide.Capture(path,root,point,Time.deltaTime,contactSettings.MaxSourceDisplacement*Math.Max(0,Math.Min(1,rodBudgetFraction.Value)),pathResponse.Value);
                                        observation.Geometry.RefitDeformation(observation.RodGuide);refitted=true;
                                        observation.ContactBody.ApplyDeformation(observation.RodGuide);observation.ContactBody.TranslationAxis=path.Closest(observation.RodGuide.Map(point)).Tangent;observation.PathParameter=observation.RodGuide.Parameter;observation.Guidance.Reset();rod=true;
                                    } else {
                                        observation.Geometry.RefitDeformation(null);
                                        refitted=true;
                                        var pose=observation.Guidance.Step(path,point,forward,Time.deltaTime,contactSettings.MaxSourceDisplacement,pathMaximumAngle.Value,pathResponse.Value,pathAngularSpeed.Value);
                                        PathGuidanceController.Apply(observation.ContactBody,pose); observation.PathParameter=pose.Parameter;
                                    }
                                    observation.ContactBody.MaximumContactTravel=Math.Max(0,pathContactTravel.Value);
                                }
                                else observation.Guidance.Reset();
                            } else { observation.Guidance.Reset();observation.PathParameter=-1; }
                            if(!sectionRig&&observation.SectionRig!=null)observation.SectionRig.Reset();
                            if(sectionRig){if(!refitted)observation.Geometry.RefitDeformation(observation.SectionRig);}
                            else if(rod){if(!refitted)observation.Geometry.RefitDeformation(observation.RodGuide);}
                            else { observation.RodGuide.Reset();if(!refitted)observation.Geometry.RefitDeformation(null); }
                            measurement.Refit+=observation.Geometry.RefitMilliseconds;
                            observation.CorrectionFrame=Time.frameCount;
                        }
                        if(!body.Binding.Visible&&composite.Surfaces.Count==0) { body.Solver.Reset(body.Animation.Anchors); continue; }
                        long solveStart=Stopwatch.GetTimestamp(); body.Solver.Advance(body.Animation.Anchors,composite,settings,policy,Time.deltaTime,body.Animation.PosedSize,contactSettings); long applyStart=Stopwatch.GetTimestamp();
                        body.Binding.Apply(body.Solver,settings.MaximumOffsetRatio*body.Animation.PosedSize,policy.NormalsInterval,transition.Blend); long applyEnd=Stopwatch.GetTimestamp(); body.SuccessfulFrames++;
                        measurement.Solve+=(applyStart-solveStart)*1000.0/Stopwatch.Frequency; measurement.Apply+=(applyEnd-applyStart)*1000.0/Stopwatch.Frequency; measurement.Vertices+=body.Solver.Positions.Length; measurement.Contacts+=body.Solver.ContactCount;
                        if(!body.PoseVerified&&body.SuccessfulFrames>=3&&transition.Blend>=0.999) body.PoseVerified=body.Binding.VerifyAppliedPose(body.Solver)<Math.Max(1e-4,body.Animation.PosedSize*1e-4);
                        if(debugContacts.Value) for(int c=0;c<body.Solver.ContactCount;c++) { var contact=body.Solver.Contacts[c]; debugOverlay.Line(AnimationAdapter.ToUnity(contact.Point),AnimationAdapter.ToUnity(contact.Point+contact.Normal*.02),Color.yellow); }
                        if(debugTrajectory.Value) { var path=ResolvePath(body); if(path!=null) for(int p=1;p<path.Count;p++) debugOverlay.Line(AnimationAdapter.ToUnity(path[p-1]),AnimationAdapter.ToUnity(path[p]),Color.green); }
                    }
                    catch(Exception e) { log.LogWarning("NEXT_FALLBACK "+e.Message); retireBodies.Add(item.Key); }
                }
                foreach(int id in retireBodies) Retire(id);
                foreach(var o in observations.Values) {
                    bool solved=contactMode.Value==ContactMode.Bidirectional&&o.CorrectionFrame==Time.frameCount;
                    bool sized=o.DimensionFrame==Time.frameCount&&o.SourceDimensions.MaximumOffset>1e-10;
                    if(!solved&&sized){o.ContactBody.Offset=o.ContactBody.RotationVector=new Vec3();o.ContactBody.ApplyDeformation(o.DimensionDeformation);}
                    if(solved||sized) {
                        o.Geometry.ApplyCorrection(o.ContactBody,transition.Blend);
                        if(debugContacts.Value) { var point=o.Chain.Count>0?o.Chain[o.Chain.Count-1].position:o.Geometry.Renderer.transform.position; debugOverlay.Line(point,point+AnimationAdapter.ToUnity(o.ContactBody.LastContactReaction),Color.magenta); }
                    } else o.Geometry.ReleaseCorrection();
                }
                BoneConstraintController.RestoreUnready();
                int activeContacts=0,visibleSources=0; foreach(var b in bodies.Values) activeContacts+=b.Solver.ContactCount; foreach(var o in observations.Values) if(o.Geometry.Visible) visibleSources++;
                Status=definition==null?"Awaiting user-prepared model package":"Bodies="+bodies.Count+" pairs="+observations.Count+" activeContacts="+activeContacts+" visibleSources="+visibleSources;
            }
            catch(Exception e) { Clear(); Status="Physics stopped safely: "+e.Message; log.LogWarning(Status); }
            finally
            {
                if(snapshotPending&&Time.unscaledTime>=snapshotAt)
                {
                    snapshotPending=false; exportSnapshot.Value=false;
                    try { string path=System.IO.Path.Combine(BepInEx.Paths.GameRootPath,"BepInEx","BetterPenetrationNext","Diagnostics","runtime-"+DateTime.UtcNow.ToString("yyyyMMdd-HHmmss-fff")+".txt"); log.LogInfo(global::Next.RuntimeProbe.Dump(path)); }
                    catch(Exception e) { log.LogWarning("Read-only snapshot failed: "+e.Message); }
                }
                LastCpuMilliseconds=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency; measurement.Total=LastCpuMilliseconds; measurement.Pairs=observations.Count;
                measurement.XRay=xray!=null&&xray.Enabled&&xray.LastRenderedFrame>=Time.frameCount-1?xray.CameraCpuMilliseconds:0; measurement.Total+=measurement.XRay;
                int collections=GC.CollectionCount(0); measurement.Collections=collections-previousCollections; previousCollections=collections; performance.Add(measurement);
                if(logPerformance.Value&&Time.unscaledTime>=lastPerformanceLog+10)
                {
                    lastPerformanceLog=Time.unscaledTime; var total=performance.Time(5); var cap=performance.Time(0); var solve=performance.Time(2); var apply=performance.Time(3); var xr=performance.Time(4); var refit=performance.Time(1);
                    log.LogInfo("NEXT_PROFILE n="+performance.Count+" CPU mean/P95="+total.Mean.ToString("F3")+"/"+total.P95.ToString("F3")+"ms capture="+cap.Mean.ToString("F3")+" refit="+refit.Mean.ToString("F3")+" solve="+solve.Mean.ToString("F3")+" apply="+apply.Mean.ToString("F3")+" xray="+xr.Mean.ToString("F3")+" particles="+measurement.Vertices+" contacts="+measurement.Contacts+" pairs="+measurement.Pairs+" GC0collections="+measurement.Collections+" monoBytes="+UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong());
                }
            }
        }
        AnatomicalPathConstraintSystem ResolvePath(PreparedBody body)
        {
            var guide=body.Character.GetComponentInChildren<AnatomicalPathGuide>(); if(guide!=null&&guide.isActiveAndEnabled) return guide.Capture();
            if(pathProfile==null) return null;
            if(body.RejectedPathProfile==pathProfile) return null;
            if(body.PathProfile!=pathProfile) {
                try { pathProfile.Validate(definition); }
                catch(ArgumentException e) { body.RejectedPathProfile=pathProfile; log.LogWarning("NEXT_PATH disabled: "+e.Message); return null; }
            }
            var matrix=body.Binding.Renderer.transform.localToWorldMatrix;
            if(body.Path==null||matrix!=body.PathMatrix||body.PathProfile!=pathProfile||(pathProfile.Animated&&body.PathFrame!=Time.frameCount)) {
                if(body.PathWorld==null||body.PathWorld.Length!=pathProfile.ControlCount) body.PathWorld=new Vec3[pathProfile.ControlCount];
                if(pathProfile.Animated) body.Animation.CapturePath(pathProfile,body.PathWorld);
                else for(int i=0;i<body.PathWorld.Length;i++) body.PathWorld[i]=AnimationAdapter.ToCore(matrix.MultiplyPoint3x4(AnimationAdapter.ToUnity(pathProfile.Points[i])));
                if(body.Path==null||body.PathProfile!=pathProfile)body.Path=new AnatomicalPathConstraintSystem(body.PathWorld,pathProfile.Spline);
                else body.Path.UpdateControls(body.PathWorld);
                body.PathMatrix=matrix; body.PathProfile=pathProfile; body.PathFrame=Time.frameCount;
            }
            return body.Path;
        }
        void Retire(int id) { PreparedBody body; if(bodies.TryGetValue(id,out body)) { if(xray!=null) xray.Unregister(body); body.Dispose(); bodies.Remove(id); } BoneConstraintController.Restore(); }
        void Clear() { foreach(var body in bodies.Values) { if(xray!=null) xray.Unregister(body); body.Dispose(); } bodies.Clear(); foreach(var o in observations.Values) { if(xray!=null) xray.Unregister(o); o.Dispose(); } observations.Clear(); registrationRetry.Clear(); registrationWarning.Clear(); BoneConstraintController.Restore(); }
        void SceneUnloaded(Scene scene) { Clear(); }
        void OnDisable() { Clear(); if(debugOverlay!=null) debugOverlay.Clear(); if(xray!=null) { xray.Enabled=false; xray.Tick(); } }
        void OnDestroy() { SceneManager.sceneUnloaded-=SceneUnloaded; Clear(); if(debugOverlay!=null) debugOverlay.Dispose(); if(xray!=null) xray.Dispose(); if(hooks!=null) hooks.UnpatchSelf(); if(Instance==this) Instance=null; }
        void OnGUI() { if(display!=null&&display.Value) GUI.Label(new Rect(20,20,1000,30),"Next "+Mode+": "+Status+" CPU="+LastCpuMilliseconds.ToString("F3")+"ms");DrawControlWindow(); }
    }
}
