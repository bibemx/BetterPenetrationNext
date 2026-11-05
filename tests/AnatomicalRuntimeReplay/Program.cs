using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text.Json;
using System.Text.Json.Serialization;
using BetterPenetrationNext.Core;

static partial class Program
{
    sealed class VectorConverter : JsonConverter<Vec3>
    {
        public override Vec3 Read(ref Utf8JsonReader reader,Type type,JsonSerializerOptions options){throw new NotSupportedException();}
        public override void Write(Utf8JsonWriter writer,Vec3 value,JsonSerializerOptions options){writer.WriteStartObject();writer.WriteNumber("X",value.X);writer.WriteNumber("Y",value.Y);writer.WriteNumber("Z",value.Z);writer.WriteEndObject();}
    }
    static readonly JsonSerializerOptions ExportOptions=MakeExportOptions();
    static JsonSerializerOptions MakeExportOptions(){var result=new JsonSerializerOptions{IncludeFields=true};result.Converters.Add(new VectorConverter());return result;}
    static Vec3 V(JsonElement p){return p.ValueKind==JsonValueKind.Array?new Vec3(p[0].GetDouble(),p[1].GetDouble(),p[2].GetDouble()):new Vec3(p.GetProperty("X").GetDouble(),p.GetProperty("Y").GetDouble(),p.GetProperty("Z").GetDouble());}
    static Vec3[] Vertices(JsonElement p){return p.EnumerateArray().Select(V).ToArray();}
    static T Read<T>(string path){return JsonCodec.Deserialize<T>(File.ReadAllText(path));}
    sealed class Source : IAnatomicalSourceBinding
    {
        public Vec3[] AnimatedVertices {get;set;}
        public Vec3[] ReferenceVertices {get;set;}
        public int[] Triangles {get;set;}
        public Vec3 Root {get;set;}
        public Vec3 ReferenceRoot;
        public Vec3 MaterialAxis {get;set;}
        public SourceContactBody Body {get;set;}
        public WeldedContactMesh Weld;public AnimatedSeamContactProxy Proxy;public TriangleSurface Surface;
        public SourceSectionRig CreateSectionRig(){return new SourceSectionRig(ReferenceVertices,Triangles,ReferenceRoot,MaterialAxis);}
        public TriangleSurface Refit(SourceSectionRig rig,Vec3[] initializationOverride)
        {
            if(initializationOverride==null){rig.MapVertices(AnimatedVertices,mapped);initializationOverride=mapped;}
            Weld.Update(initializationOverride,.00006);if(!Proxy.Update(Weld.Positions))throw new Exception("Recorded source seam proxy separated");Surface.Refit(Proxy.Positions);return Surface;
        }
        public Vec3[] mapped;
    }
    sealed class Frame {public int Id;public Vec3[] Source,Solved;public Vec3 Root,Offset;public Linear3[] Linear;public Vec3[] Translation;public double[] Morph;public double Dt,Size,Time;public Linear3 SourceFrame;public double SourceLengthScale=1,SourceWidthScale=1;public JsonElement Settings;}
    static double FrameSetting(Frame f,string name,double fallback){JsonElement value;return f.Settings.ValueKind==JsonValueKind.Object&&f.Settings.TryGetProperty(name,out value)&&value.ValueKind==JsonValueKind.Number?value.GetDouble():fallback;}
    static double EnvironmentSetting(string name,double fallback){double value;return double.TryParse(Environment.GetEnvironmentVariable(name),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out value)?value:fallback;}
    static double CaptureSetting(JsonElement capture,string name,double fallback){JsonElement settings,value;return capture.TryGetProperty("settings",out settings)&&settings.TryGetProperty(name,out value)&&value.ValueKind==JsonValueKind.Number?value.GetDouble():fallback;}
    static Linear3 SourceRotation(JsonElement state)
    {
        JsonElement chain,q;if(!state.TryGetProperty("chain",out chain)||chain.GetArrayLength()==0||!chain[0].TryGetProperty("worldRotation",out q))return new Linear3();
        double x=q[0].GetDouble(),y=q[1].GetDouble(),z=q[2].GetDouble(),w=q[3].GetDouble(),s=2/(x*x+y*y+z*z+w*w);
        return new Linear3(new Vec3(1-s*(y*y+z*z),s*(x*y+z*w),s*(x*z-y*w)),new Vec3(s*(x*y-z*w),1-s*(x*x+z*z),s*(y*z+x*w)),new Vec3(s*(x*z+y*w),s*(y*z-x*w),1-s*(x*x+y*y)));
    }
    static Vec3 BindOrigin(JsonElement p){var m=p.EnumerateArray().Select(value=>value.GetDouble()).ToArray();var linear=new Linear3(new Vec3(m[0],m[4],m[8]),new Vec3(m[1],m[5],m[9]),new Vec3(m[2],m[6],m[10]));Vec3 v;if(!linear.TryInverseMultiply(-new Vec3(m[3],m[7],m[11]),out v))throw new Exception("Singular source reference");return v;}
    static int Main(string[] args)
    {
        try
        {
            string work=Path.GetFullPath(args[0]),resources=Path.GetFullPath(args[1]),output=Path.GetFullPath(args[2]);Directory.CreateDirectory(output);
            var physical=Read<SimulationDefinition>(Path.Combine(resources,"physical.json"));var support=Read<SimulationDefinition>(Path.Combine(resources,"support.json"));var transfer=Read<ReceiverSupportTransfer>(Path.Combine(resources,"transfer.json"));var profile=Read<AnimatedPathProfile>(Path.Combine(resources,"path.json"));var contours=Read<ReceiverGuideDefinition>(Path.Combine(resources,"contours.json"));
            var frames=new List<Frame>();var roots=new Dictionary<int,Vec3>();bool live=args.Length>3;
            if(live)
            {
                using(var document=JsonDocument.Parse(File.ReadAllText(args[3])))foreach(var g in document.RootElement.GetProperty("captures").EnumerateArray())
                {
                    var r=g.GetProperty("Receivers")[0];var s=g.GetProperty("Sources")[0];var state=s.GetProperty("state");
                    // AnimationAdapter.CapturedMorphs are already normalized 0..1.
                    frames.Add(new Frame{Id=g.GetProperty("CapturedFrame").GetInt32(),Settings=g.GetProperty("settings").Clone(),Time=g.GetProperty("time").GetProperty("unscaledTime").GetDouble(),Source=Vertices(s.GetProperty("AnimatedWorld")),Solved=Vertices(r.GetProperty("SolvedWorld")),Root=V(state.GetProperty("animationRoot")),SourceFrame=SourceRotation(state),SourceLengthScale=CaptureSetting(g,"sourceLengthScale",1),SourceWidthScale=CaptureSetting(g,"sourceWidthScale",1),Offset=V(state.GetProperty("offset")),Linear=r.GetProperty("pathLinearColumns").EnumerateArray().Select(p=>new Linear3(V(p[0]),V(p[1]),V(p[2]))).ToArray(),Translation=Vertices(r.GetProperty("pathTranslation")),Morph=r.GetProperty("morphWeights").EnumerateArray().Select(p=>p.GetDouble()).ToArray(),Dt=g.GetProperty("time").GetProperty("deltaTime").GetDouble(),Size=r.GetProperty("state").GetProperty("posedSize").GetDouble()});
                }
            }
            else {
            using(var geometry=JsonDocument.Parse(File.ReadAllText(Path.Combine(work,"receiver-sequence-replay/Profiles/geometry.json"))))foreach(var p in geometry.RootElement.GetProperty("poses").EnumerateArray())roots.Add(p.GetProperty("frame").GetInt32(),V(p.GetProperty("root")));
            foreach(string file in Directory.GetFiles(Path.Combine(work,"receiver-sequence-20261102"),"capture-*.json").OrderBy(f=>f))
            {
                using(var raw=JsonDocument.Parse(File.ReadAllText(file)))
                {
                    var g=raw.RootElement;int id=g.GetProperty("CapturedFrame").GetInt32();var r=g.GetProperty("Receivers")[0];var columns=r.GetProperty("pathLinearColumns");
                    var linear=columns.EnumerateArray().Select(p=>new Linear3(V(p[0]),V(p[1]),V(p[2]))).ToArray();
                    using(var trimmed=JsonDocument.Parse(File.ReadAllText(Path.Combine(work,"receiver-sequence-replay/Data/frame-"+id+".json"))))
                        frames.Add(new Frame{Id=id,Source=Vertices(g.GetProperty("Sources")[0].GetProperty("AnimatedWorld")),Root=roots[id],Linear=linear,Translation=Vertices(r.GetProperty("pathTranslation")),Morph=r.GetProperty("morphWeights").EnumerateArray().Select(p=>p.GetDouble()*.01).ToArray(),Dt=g.GetProperty("time").GetProperty("deltaTime").GetDouble(),Size=trimmed.RootElement.GetProperty("Receivers")[0].GetProperty("PosedSize").GetDouble()});
                }
            }
            }
            var source=new Source{Body=new SourceContactBody{PreserveContactAcrossDeformation=true},AnimatedVertices=frames[0].Source,Root=frames[0].Root};
            using(var raw=JsonDocument.Parse(File.ReadAllText(Path.Combine(work,"receiver-sequence-replay/Data/source-asset-data.json"))))
            {
                var mesh=raw.RootElement.GetProperty("meshes").EnumerateArray().First(m=>m.GetProperty("name").GetString()=="o_dankon");source.ReferenceVertices=Vertices(mesh.GetProperty("positions"));
                using(var first=JsonDocument.Parse(File.ReadAllText(Path.Combine(work,"receiver-sequence-replay/Data/frame-35460.json"))))source.Triangles=first.RootElement.GetProperty("Sources")[0].GetProperty("TriangleIndices").EnumerateArray().Select(p=>p.GetInt32()).ToArray();
                var groups=new int[source.ReferenceVertices.Length];var keys=new Dictionary<string,int>();var bi=mesh.GetProperty("boneIndices");var bw=mesh.GetProperty("boneWeights");for(int i=0;i<groups.Length;i++){string key=bi[i].GetRawText()+bw[i].GetRawText();int group;if(!keys.TryGetValue(key,out group)){group=keys.Count;keys.Add(key,group);}groups[i]=group;}
                source.Weld=new WeldedContactMesh(source.ReferenceVertices,source.Triangles,1e-6,true,groups);source.Proxy=new AnimatedSeamContactProxy(source.Weld.RestPositions,source.Weld.Triangles);source.Weld.Update(source.AnimatedVertices,.00006);source.Proxy.Update(source.Weld.Positions);source.Surface=new TriangleSurface(source.Proxy.Positions,source.Proxy.Triangles,SurfaceKind.ClosedVolume){RobustSign=false};source.Body.Surface=source.Surface;source.mapped=new Vec3[source.ReferenceVertices.Length];
                var bind=mesh.GetProperty("bindposes");source.ReferenceRoot=BindOrigin(bind[0]);source.MaterialAxis=BindOrigin(bind[bind.GetArrayLength()-1])-source.ReferenceRoot;
            }
            if(Environment.GetEnvironmentVariable("BPN_CAPTURE_CADENCE")=="1")for(int i=1;i<frames.Count;i++)frames[i].Dt=Math.Max(1e-6,frames[i].Time-frames[i-1].Time);
            if(Environment.GetEnvironmentVariable("BPN_DENSE_REPLAY")=="1")frames=DenseFrames(frames);
            if(args.Length>4&&args[4]=="--reactive")return Reactive(frames,physical,support,transfer,contours,source,output,Read<ReceiverSupportTransfer>(Path.Combine(work,"receiver-support-transfer.json")),File.Exists(Path.Combine(resources,"chart.json"))?Read<ReactiveMaterialChart>(Path.Combine(resources,"chart.json")):null);
            var session=new AnatomicalContactSession(physical,support,transfer,profile,contours,source.AnimatedVertices.Length);var contact=new BidirectionalContactSettings{MaxTissueStrain=4,SourceCorrectionStrength=.35,MaxSourceDisplacement=.05,ContactDamping=12,ProfileStages=true};
            double total=0,maxMovement=0;long allocated=0;var rows=new List<object>();
            for(int n=live?-40:-240;n<frames.Count;n++)
            {
                var f=frames[n<0?0:n];double dt=live?f.Dt:n<0&&n< -10?1.0/60:f.Dt;source.AnimatedVertices=f.Source;source.Root=f.Root;source.Body.BeginFrame(contact,dt,true);
                if(live&&n>=0){Array.Copy(f.Solved,session.Solver.Positions,f.Solved.Length);Array.Clear(session.Solver.Velocities,0,session.Solver.Velocities.Length);source.Body.Offset=f.Offset;}
                long bytes=GC.GetAllocatedBytesForCurrentThread(),start=Stopwatch.GetTimestamp();session.Step(f.Linear,f.Translation,f.Morph,source,contact,dt,f.Size,false);double ms=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency;
                if(n<0){if(n%40==0)Console.WriteLine("INITIALIZING "+n+" ms="+ms);continue;}
                allocated+=GC.GetAllocatedBytesForCurrentThread()-bytes;total+=ms;maxMovement=Math.Max(maxMovement,session.MaximumSourceMovement);
                var points=session.Solver.Positions.Take(539).ToArray();var src=new Vec3[source.AnimatedVertices.Length];for(int i=0;i<src.Length;i++)src[i]=source.Body.ToCorrectedVertex(i,source.AnimatedVertices[i]);
                int[][] faces=new int[physical.ConfinementTriangles.Length/3][];for(int i=0;i<faces.Length;i++)faces[i]=new[]{physical.ConfinementTriangles[i*3],physical.ConfinementTriangles[i*3+1],physical.ConfinementTriangles[i*3+2]};
                var export=new {Frame=f.Id,SourceWorld=src,SourceTriangles=source.Triangles,InnerWorld=points,InnerTriangles=faces,Boundaries=contours.Boundaries,PathWorld=Enumerable.Range(0,25).Select(i=>session.Path.AtDistance(session.Path.Length*i/24).Point).ToArray()};
                File.WriteAllText(Path.Combine(output,"solved-"+f.Id+".json"),JsonSerializer.Serialize(export,ExportOptions));
                rows.Add(new{Frame=f.Id,StepMs=ms,session.Solver.ContactQueries,session.Solver.ResidualQueries,session.Solver.MaterialBaselineResets,session.Solver.BroadphaseMilliseconds,session.Solver.SearchMilliseconds,session.Solver.CorrectionMilliseconds,session.Solver.ResidualMilliseconds,session.Solver.AdvanceMilliseconds,session.Solver.MaterialGuardMilliseconds,session.Solver.MaterialGuardSweeps,session.Solver.LocalGuardPasses,CavityBuildMs=session.Solver.Cavity.BuildMilliseconds,CavityProjectMs=session.Solver.Cavity.ProjectionMilliseconds,CavityResidualMs=session.Solver.Cavity.ResidualMilliseconds,CavityQueries=session.Solver.Cavity.Queries,CavityClassifierQueries=session.Solver.Cavity.ClassifierQueries,DirectionMs=session.Solver.DirectionalLimit.Milliseconds,MeshRefineMs=session.Solver.MeshContacts.Milliseconds,ContinuousMs=session.Solver.ContinuousSelfContact.Milliseconds,TissueReactionMs=session.Solver.ContinuousSelfContact.ReactionMilliseconds,CachedBlockedMoves=session.Solver.ContinuousSelfContact.CachedBlockedMoves,ContinuousPairs=session.Solver.ContinuousSelfContact.PairTests,SourceMovement=session.MaximumSourceMovement});
            }
            var report=new{Scope=live?"Diagnostic replay of current 1.0.8 MCP poses and seeded live solved positions/Source offset; uncaptured velocities initialized to zero and guide histories warmed separately. Stage attribution on .NET6; excludes Unity capture/upload/render and is not measured game FPS.":"Exact native-kernel C# session on recorded poses; excludes Unity capture, upload, XRay and game rendering; no new KKS scene verification",Frames=frames.Count,MeanStepMs=total/frames.Count,AllocatedBytes=allocated,MaximumSourceMovement=maxMovement,SourceLimit=.05,ContactSamples=session.ContactSamples,RefinementPasses=session.RefinementPasses,Rows=rows};File.WriteAllText(Path.Combine(output,"acceptance.json"),JsonSerializer.Serialize(report,new JsonSerializerOptions{WriteIndented=true}));Console.WriteLine("REPLAY_COMPLETED frames="+frames.Count+" meanMs="+total/frames.Count+" allocated="+allocated);return 0;
        }
        catch(Exception e){Console.Error.WriteLine(e);return 1;}
    }
    static int Reactive(System.Collections.Generic.List<Frame> frames,SimulationDefinition physical,SimulationDefinition support,ReceiverSupportTransfer transfer,ReceiverGuideDefinition contours,Source source,string output,ReceiverSupportTransfer mucosal,ReactiveMaterialChart chart)
    {
        if(Environment.GetEnvironmentVariable("BPN_OPENING_AUDIT")=="1"){
            var rn=new CervicalOpeningLandmark(support.Rest,support.Triangles,chart.TriangleRegions);rn.Update(support.Rest,new Vec3(0,1,1),new Vec3(1,0,0));
            var faces=Enumerable.Range(0,chart.TriangleRegions.Length).Where(i=>chart.TriangleRegions[i]==3).SelectMany(i=>support.Triangles.Skip(i*3).Take(3)).ToArray();var mesh=new TriangleSurface(support.Rest,faces,SurfaceKind.OrientedShell);SurfaceSample sample;
            foreach(double offset in new[]{-.005,0,.005}){Vec3 point=rn.Centre+rn.Normal*offset;if(mesh.TrySample(point,.03,out sample))Console.WriteLine("REAL_OS_AUDIT offset="+offset+" signed="+sample.SignedDistance+" nearest="+(point-sample.Point).Length+" radius="+rn.Radius);}
            return 0;
        }
        if(Environment.GetEnvironmentVariable("BPN_ENTRY_JUNCTION_TESTS")=="1")return EntryJunctionChecks(frames,physical,support,transfer,contours,source,mucosal,chart,output);
        if(Environment.GetEnvironmentVariable("BPN_STYLIZED_TESTS")=="1")return StylizedReceiverChecks(frames[0],physical,support,transfer,contours,source,mucosal,chart,output);
        if(Environment.GetEnvironmentVariable("BPN_CONTROL_TESTS")=="1")return ReactiveControlChecks(frames[0],physical,support,transfer,contours,source,mucosal,chart,output);
        var rig=new ReactiveReceiverRig(physical,support,transfer,contours,source.ReferenceVertices,source.Triangles,source.MaterialAxis,source.CreateSectionRig(),mucosal,chart);rig.EnableMaterialProjection=Environment.GetEnvironmentVariable("BPN_RIG_RAW")==null;var input=new ReactiveRigInput(physical.BoneNames.Length,physical.Morphs.Length,source.ReferenceVertices.Length);var result=new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,source.ReferenceVertices.Length,rig.SectionCount);var rows=new System.Collections.Generic.List<object>();
        rig.FollowLoadedTerminal=Environment.GetEnvironmentVariable("BPN_UNLOADED_TERMINAL")!="1";rig.SourceCervicalReaction=Environment.GetEnvironmentVariable("BPN_NO_CERVICAL_REACTION")!="1";
        rig.RegularizeEntrance=Environment.GetEnvironmentVariable("BPN_LEGACY_ENTRANCE")==null;rig.PreserveEntranceProfile=Environment.GetEnvironmentVariable("BPN_EXPERIMENTAL_ENTRY_PROFILE")!=null;
        rig.LocalCervixOnly=Environment.GetEnvironmentVariable("BPN_LEGACY_CERVIX")==null;
        rig.ExactSourceCuts=Environment.GetEnvironmentVariable("BPN_EXACT_SOURCE_CUTS")=="1";
        rig.SharedCanalGuide=Environment.GetEnvironmentVariable("BPN_SHARED_GUIDE")=="1";
        rig.SourceRod.ContinuousTerminalStop=Environment.GetEnvironmentVariable("BPN_LEGACY_TERMINAL_STOP")!="1";rig.ConsistentCervicalGuide=Environment.GetEnvironmentVariable("BPN_LEGACY_CERVICAL_GUIDE")!="1";rig.UseContactPrecision=Environment.GetEnvironmentVariable("BPN_LEGACY_CONTACT_PRECISION")!="1";rig.SourceRod.ContinuousTerminalOrientation=Environment.GetEnvironmentVariable("BPN_LEGACY_TERMINAL_ORIENTATION")!="1";
        rig.SourceRod.StablePathGuidance=Environment.GetEnvironmentVariable("BPN_LEGACY_GUIDE")!="1";
        rig.AnatomicalCervicalMotion=Environment.GetEnvironmentVariable("BPN_LEGACY_CERVICAL_AXIS")!="1";
        rig.CervixTravelBeforeSource=Environment.GetEnvironmentVariable("BPN_LEGACY_CERVIX_ORDER")!="1";
        bool replayGuide=Environment.GetEnvironmentVariable("BPN_REPLAY_CURRENT_GUIDE")=="1";
        var mainRig=replayGuide?new ReactiveReceiverRig(physical,support,transfer,contours,source.ReferenceVertices,source.Triangles,source.MaterialAxis,source.CreateSectionRig(),mucosal,chart):null;
        var mainResult=replayGuide?new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,source.ReferenceVertices.Length,rig.SectionCount):null;
        rig.FiniteSurfaceCuts=Environment.GetEnvironmentVariable("BPN_LEGACY_FINITE_CUTS")!="1";
        rig.CurrentContactProfile=Environment.GetEnvironmentVariable("BPN_LEGACY_RADIAL_HISTORY")!="1";
        rig.SourceRod.DistributedTerminalApproach=Environment.GetEnvironmentVariable("BPN_LEGACY_NARROW_APPROACH")!="1";
        if(mainRig!=null){mainRig.ConsistentCervicalGuide=rig.ConsistentCervicalGuide;mainRig.UseContactPrecision=rig.UseContactPrecision;mainRig.LocalCervixOnly=rig.LocalCervixOnly;}
        if(mainRig!=null)mainRig.SourceRod.DistributedTerminalApproach=rig.SourceRod.DistributedTerminalApproach;
        var dimensionOverride=new SourceDimensions(source.CreateSectionRig(),EnvironmentSetting("BPN_SOURCE_LENGTH_SCALE",frames[0].SourceLengthScale),EnvironmentSetting("BPN_SOURCE_WIDTH_SCALE",frames[0].SourceWidthScale));
        long bytes=0;double total=0;for(int n=-30;n<frames.Count;n++)
        {
            var f=frames[Math.Max(0,n)];Array.Copy(f.Linear,input.Bones,f.Linear.Length);Array.Copy(f.Translation,input.Translation,f.Translation.Length);Array.Copy(f.Morph,input.Morph,f.Morph.Length);Array.Copy(f.Source,input.Source,f.Source.Length);input.SourceRoot=f.Root;input.SourceFrame=f.SourceFrame;input.Dt=n<0?1.0/60:f.Dt;input.Frame=f.Id;input.PosedSize=f.Size;input.HasSource=true;input.SourceFollowsPath=true;double control;
            input.CanalLengthScale=FrameSetting(f,"canalLengthScale",1);input.CervixMaxDisplacement=FrameSetting(f,"cervixMaxDisplacement",.025);input.CervixFollowStrength=FrameSetting(f,"cervixFollowStrength",1);input.MaxSource=FrameSetting(f,"maxSource",.12);input.MaxStrain=FrameSetting(f,"maxStrain",4);input.SourceCompliance=FrameSetting(f,"sourceStrength",.35);input.TissueStrength=FrameSetting(f,"tissueStrength",1);input.Compliance=FrameSetting(f,"tissueCompliance",1e-7);input.Response=FrameSetting(f,"contactDamping",12);input.Recovery=FrameSetting(f,"recoveryTime",4.5);input.MaximumOffsetRatio=FrameSetting(f,"offset",1.5);
            if(double.TryParse(Environment.GetEnvironmentVariable("BPN_CANAL_LENGTH"),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out control))input.CanalLengthScale=control;
            if(double.TryParse(Environment.GetEnvironmentVariable("BPN_MAX_SOURCE"),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out control))input.MaxSource=control;
            if(double.TryParse(Environment.GetEnvironmentVariable("BPN_RECOVERY"),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out control))input.Recovery=control;
            if(double.TryParse(Environment.GetEnvironmentVariable("BPN_CERVIX_TRAVEL"),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out control))input.CervixMaxDisplacement=control;
            double dimensionLength=f.SourceLengthScale,dimensionWidth=f.SourceWidthScale;
            if(double.TryParse(Environment.GetEnvironmentVariable("BPN_SOURCE_LENGTH_SCALE"),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out control))dimensionLength=control;
            if(double.TryParse(Environment.GetEnvironmentVariable("BPN_SOURCE_WIDTH_SCALE"),System.Globalization.NumberStyles.Float,System.Globalization.CultureInfo.InvariantCulture,out control))dimensionWidth=control;
            long startBytes=GC.GetAllocatedBytesForCurrentThread(),dimensionStart=Stopwatch.GetTimestamp();dimensionOverride.Apply(f.Source,input.SourceRoot,dimensionLength,dimensionWidth,input.Dt,12,input.Source);double dimensionMs=(Stopwatch.GetTimestamp()-dimensionStart)*1000.0/Stopwatch.Frequency;rig.Step(input,result);result.TotalMs+=dimensionMs;double mainGuideMs=0,guidePathError=0,sourcePassDifference=0;
            if(mainRig!=null){mainRig.GuideSourceOnly(input,mainResult,result,input.Dt);mainGuideMs=mainResult.TotalMs;for(int i=0;i<rig.SourcePath.Count;i++)guidePathError=Math.Max(guidePathError,(rig.SourcePath[i]-mainRig.SourcePath[i]).Length);for(int i=0;i<input.Source.Length;i++)sourcePassDifference=Math.Max(sourcePassDifference,(result.SourceWorld[i]-mainResult.SourceWorld[i]).Length);}
            var rendered=mainResult??result;var renderedRig=mainRig??rig;long alloc=GC.GetAllocatedBytesForCurrentThread()-startBytes;if(n<0)continue;bytes+=alloc;total+=result.TotalMs+mainGuideMs;
            if(Environment.GetEnvironmentVariable("BPN_TRACE")=="1"&&(n==0||Environment.GetEnvironmentVariable("BPN_TRACE_FIRST_ONLY")!="1")){
                var trace=new System.Collections.Generic.Dictionary<string,object>();
                foreach(string name in new[]{"cervicalCollarWeight","cervicalPose","terminal","edges","referenceLengths","materialTargets","baseFull","nativeBaseFull","guardAnchors","materialBaseline","candidate","previousLocal","targets","centres","fieldRadii","baseRadius","scale","station","newTangents","newX","sourceCorrected"})trace[name]=typeof(ReactiveReceiverRig).GetField(name,System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance).GetValue(rig);
                var radial=typeof(ReactiveReceiverRig).GetField("radialField",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance).GetValue(rig);
                foreach(string name in new[]{"required","history","values"})trace["radial_"+name]=radial.GetType().GetField(name,System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance).GetValue(radial);
                trace["SourceSections"]=rig.Landmarks.Sections;
                var guard=typeof(ReactiveReceiverRig).GetField("materialGuard",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance).GetValue(rig);trace["BaselineResets"]=guard.GetType().GetField("BaselineResets").GetValue(guard);trace["LimitedVertices"]=guard.GetType().GetField("LimitedVertices").GetValue(guard);
                File.WriteAllText(Path.Combine(output,"trace-"+f.Id+".json"),JsonSerializer.Serialize(trace,ExportOptions));
            }
            int attachmentVertices=0;double maximumAttachmentMovement=0,maximumProtectedOrganMovement=0;
            for(int i=0;i<source.ReferenceVertices.Length;i++)if(Vec3.Dot(source.ReferenceVertices[i]-source.ReferenceRoot,source.MaterialAxis.Normalized)<=0){attachmentVertices++;maximumAttachmentMovement=Math.Max(maximumAttachmentMovement,result.SourceOffsets[i].Length);}
            if(chart!=null)for(int i=0;i<result.Offsets.Length;i++)if(chart.Regions[i]==4||chart.Regions[i]==5||chart.Regions[i]==9)maximumProtectedOrganMovement=Math.Max(maximumProtectedOrganMovement,result.Offsets[i].Length);
            if(maximumAttachmentMovement>1e-9)throw new Exception("Source attachment moved");
            if(chart!=null){var pose=rig.LocalCervixOnly?new RigidOrganPose():new RigidOrganPose{Pivot=result.TerminalCentre-result.CervicalDisplacement,Translation=result.CervicalDisplacement,Rotation=result.UterusRotation};for(int i=0;i<result.Offsets.Length;i++){if(chart.Regions[i]==9&&result.Offsets[i].Length>1e-9)throw new Exception("Unrelated protected organ moved");if(rig.IsRigidOrganControl(i)&&(result.FullWorld[i]-pose.Point(rig.NativeSupportPoint(i)+result.CanalLengthDisplacement)).Length>1e-8)throw new Exception("Actual organ vertices did not follow the required organ pose");}if(rig.LocalCervixOnly&&result.ContactCanalExtension>1e-12)throw new Exception("Local cervical contact extended the channel");}
            rows.Add(new{Frame=f.Id,Time=f.Time,Dt=input.Dt,SourceTip=new[]{renderedRig.Landmarks.MaterialTip.X,renderedRig.Landmarks.MaterialTip.Y,renderedRig.Landmarks.MaterialTip.Z},MainGuideMs=mainGuideMs,GuidePathError=guidePathError,SourcePassDifference=sourcePassDifference,RenderedTipGap=(renderedRig.Landmarks.MaterialTip-result.TerminalCentre).Length,SourceRoot=new[]{f.Root.X,f.Root.Y,f.Root.Z},CervicalVector=new[]{result.CervicalDisplacement.X,result.CervicalDisplacement.Y,result.CervicalDisplacement.Z},CervicalMotionAxis=new[]{result.CervicalMotionAxis.X,result.CervicalMotionAxis.Y,result.CervicalMotionAxis.Z},result.SourceMaximumTurn,result.SourceBudgetBlend,RootGoalAngle=Math.Acos(Math.Clamp(Vec3.Dot((rig.SourceRod.Animated[1]-rig.SourceRod.Animated[0]).Normalized,(result.TerminalCentre-f.Root).Normalized),-1,1)),NativeRootCorner=Math.Acos(Math.Clamp(Vec3.Dot((rig.SourceRod.Animated[1]-rig.SourceRod.Animated[0]).Normalized,(rig.SourceRod.Animated[2]-rig.SourceRod.Animated[1]).Normalized),-1,1)),IntendedDistance=rig.SourceRod.IntendedDistance,GuideLength=rig.SourceRod.GuideLength,EndpointError=rig.SourceRod.EndpointError,TerminalPressure=rig.SourceRod.TerminalPressure,result.MaterialEdgeCount,result.MaterialFaceCount,result.EntranceRegularizedVertices,result.EntranceSymmetryPairs,result.EntranceSymmetryResidual,result.EntranceMidlineVertices,result.EntranceMidlineResidual,UterusRotation=new[]{result.UterusRotation.X,result.UterusRotation.Y,result.UterusRotation.Z},UterusPelvisUp=new[]{result.UterusPelvisUp.X,result.UterusPelvisUp.Y,result.UterusPelvisUp.Z},result.NativeCanalArc,result.ConfiguredCanalArc,result.CervicalTipPressure,result.EffectiveCervicalTravel,result.SourceArcLength,result.TerminalRootDistance,result.TerminalWithinSourceLength,result.MaterialTipVertex,result.CervicalReactionAttempts,result.CervicalReactionRejected,StepMs=result.TotalMs,attachmentVertices,maximumAttachmentMovement,maximumProtectedOrganMovement,result.CaptureMs,result.PathMs,result.ContactMs,result.TransferMs,result.SectionTests,result.TriangleTests,result.ActiveSections,result.MaterialBlend,result.LimitingEdge,result.MaximumStrain,result.MaximumPrimaryStrain,result.MaxSourceMovement,result.EntranceRadius,result.RequiredEntranceRadius,result.ReceiverActive,result.ReceiverLandmarkCount,result.ReceiverSpacing,result.CanalExtension,result.ContactCanalExtension,result.CervicalPenetration,result.CervicalResidual,CervicalTravel=result.CervicalDisplacement.Length,result.CervicalQueries,result.TerminalAvailable,result.TerminalRadius,result.TerminalTipAxial,result.TerminalTipLateral,result.TerminalTipDistance,result.MaximumResidual,result.ContainedVertices,result.EngagedVertices,result.SourceVertices});
            if(Environment.GetEnvironmentVariable("BPN_DENSE_REPLAY")=="1"&&n!=frames.Count-1)continue;
            var src=new Vec3[input.Source.Length];for(int i=0;i<src.Length;i++)src[i]=rendered.SourceWorld[i];int[][] faces=new int[physical.ConfinementTriangles.Length/3][];for(int i=0;i<faces.Length;i++)faces[i]=new[]{physical.ConfinementTriangles[i*3],physical.ConfinementTriangles[i*3+1],physical.ConfinementTriangles[i*3+2]};
            File.WriteAllText(Path.Combine(output,"solved-"+f.Id+".json"),JsonSerializer.Serialize(new{Frame=f.Id,SourceWorld=src,SourceTriangles=source.Triangles,InnerWorld=result.Physical.Take(539).ToArray(),InnerTriangles=faces,Boundaries=contours.Boundaries,PathWorld=result.Path},ExportOptions));
            if(n==frames.Count-1||Environment.GetEnvironmentVariable("BPN_FULL_SEQUENCE")=="1"){var full=new Vec3[support.Rest.Length];for(int i=0;i<full.Length;i++){Vec3 p=support.Rest[i];for(int m=0;m<support.Morphs.Length;m++)p+=support.Morphs[m].Deltas[i]*input.Morph[m];for(int j=support.InfluenceOffsets[i];j<support.InfluenceOffsets[i+1];j++){int b=support.BoneIndices[j];full[i]+=(input.Bones[b].Multiply(p)+input.Translation[b])*support.BoneWeights[j];}full[i]+=result.Offsets[i];}File.WriteAllText(Path.Combine(output,"full-"+f.Id+".json"),JsonSerializer.Serialize(new{Frame=f.Id,SourceWorld=src,SourceTriangles=source.Triangles,InnerWorld=full,InnerTriangles=Enumerable.Range(0,support.Triangles.Length/3).Select(i=>new[]{support.Triangles[i*3],support.Triangles[i*3+1],support.Triangles[i*3+2]}).ToArray(),Boundaries=contours.Boundaries,PathWorld=result.Path},ExportOptions));}
        }
        File.WriteAllText(Path.Combine(output,"reactive.json"),JsonSerializer.Serialize(new{Scope="Reactive reduced-rig numerical replay; proxy statistics are not independent native-mesh containment or live frame times",IncludesCurrentGuide=replayGuide,Frames=frames.Count,MeanStepMs=total/frames.Count,WarmAllocatedBytes=bytes,Rows=rows},new JsonSerializerOptions{WriteIndented=true}));Console.WriteLine("REACTIVE_COMPLETED meanMs="+total/frames.Count+" allocated="+bytes);return 0;
    }
}
