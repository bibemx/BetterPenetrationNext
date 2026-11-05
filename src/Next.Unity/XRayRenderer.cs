using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    public enum XRayTarget { Source,Receiver,Both }
    public enum OcclusionMode { HiddenOnly,AllSelected }
    public sealed class XRayRenderer : IDisposable
    {
        sealed class Surface
        { public SkinnedMeshRenderer Renderer; public bool Source; public Func<Mesh> Snapshot; public Material[] Materials; public Mesh Mesh; public Matrix4x4 Matrix; }
        sealed class CameraState
        { public Camera Camera; public CommandBuffer Buffer; public RenderTexture SourceDepth,ReceiverDepth,ExtraDepth; public int Width,Height; public DepthTextureMode Previous; }
        readonly Dictionary<object,Surface> surfaces=new Dictionary<object,Surface>();
        readonly Dictionary<Camera,CameraState> cameras=new Dictionary<Camera,CameraState>();
        readonly List<Camera> dead=new List<Camera>(); readonly List<SkinnedMeshRenderer> occluders=new List<SkinnedMeshRenderer>();
        readonly List<Material> materials=new List<Material>(); readonly MaterialPropertyBlock block=new MaterialPropertyBlock();
        readonly Material depthMaterial;
        readonly HashSet<Renderer> drawnRenderers=new HashSet<Renderer>();
        readonly Dictionary<Renderer,Surface> posed=new Dictionary<Renderer,Surface>();
        readonly AssetBundle bundle;
        double nextOccluderScan;
        public bool Enabled,DotPatternEnabled=true,IncludeHiddenSource=true;
        public XRayTarget Target=XRayTarget.Source;
        public OcclusionMode Occlusion=OcclusionMode.HiddenOnly;
        public Color SourceColor=Color.cyan,ReceiverColor=new Color(1,0.55f,0.2f,1);
        public float SourceOpacity=0.45f,ReceiverOpacity=0.35f,OriginalTextureBlend=1,DotScale=8,DotDensity=0.5f;
        public readonly List<Renderer> AdditionalOccluders=new List<Renderer>();
        public double CameraCpuMilliseconds { get; private set; }
        public double LastCameraCpuMilliseconds { get; private set; }
        public long CameraCallbacks { get; private set; }
        public int LastRenderedFrame { get { return timedFrame; } }
        int timedFrame=-1;
        Shader shader;
        public Shader DiagnosticShader { get { return shader; } }
        public XRayRenderer(Shader diagnosticShader=null)
        {
            if(diagnosticShader==null) {
            using(var stream=typeof(XRayRenderer).Assembly.GetManifestResourceStream("BetterPenetrationNext.xray.bundle"))
            {
                if(stream==null) throw new FileNotFoundException("Next shader bundle was not embedded in this development DLL.");
                var bytes=new byte[stream.Length]; int done=0; while(done<bytes.Length) { int n=stream.Read(bytes,done,bytes.Length-done); if(n==0) throw new EndOfStreamException(); done+=n; }
                bundle=AssetBundle.LoadFromMemory(bytes);
            }
            if(bundle==null) throw new InvalidOperationException("Cannot load Next X-Ray bundle.");
            shader=bundle.LoadAsset<Shader>("assets/shaders/nextxray.shader");
            } else shader=diagnosticShader;
            if(shader==null||!shader.isSupported) { if(bundle!=null) bundle.Unload(true); throw new NotSupportedException("Next X-Ray shader unavailable on this device."); }
            depthMaterial=new Material(shader) { name="BPN private depth",hideFlags=HideFlags.HideAndDontSave }; Camera.onPreCull+=OnCamera;
        }
        public void Register(object key,SkinnedMeshRenderer renderer,bool source,Func<Mesh> snapshot)
        {
            if(surfaces.ContainsKey(key)) return;
            var slots=new Material[renderer.sharedMesh.subMeshCount]; for(int i=0;i<slots.Length;i++) slots[i]=new Material(shader) { name="BPN diagnostic surface "+i,hideFlags=HideFlags.HideAndDontSave };
            surfaces.Add(key,new Surface { Renderer=renderer,Source=source,Snapshot=snapshot,Materials=slots });
        }
        public void Unregister(object key) { Surface s; if(surfaces.TryGetValue(key,out s)) { foreach(var m in s.Materials) Object.Destroy(m); surfaces.Remove(key); } }
        bool Visible(Camera c,Renderer r) { return r!=null&&r.enabled&&r.gameObject.activeInHierarchy&&(c.cullingMask&(1<<r.gameObject.layer))!=0; }
        bool SelectedVisible(Camera c,Surface s) { return s.Renderer!=null&&(c.cullingMask&(1<<s.Renderer.gameObject.layer))!=0&&((s.Source&&IncludeHiddenSource)||(s.Renderer.enabled&&s.Renderer.gameObject.activeInHierarchy)); }
        static RenderTexture Depth(int width,int height)
        { var rt=new RenderTexture(width,height,24,RenderTextureFormat.RFloat,RenderTextureReadWrite.Linear) { name="BPN private occlusion depth",filterMode=FilterMode.Point }; rt.Create(); return rt; }
        void OnCamera(Camera camera)
        {
            if(camera==null) return;
            bool selectedVisible=false;
            if(Enabled&&camera.enabled&&!camera.stereoEnabled&&camera.cameraType!=CameraType.Reflection) foreach(var surface in surfaces.Values) if(SelectedVisible(camera,surface)&&(Target==XRayTarget.Both||(Target==XRayTarget.Source)==surface.Source)) { selectedVisible=true; break; }
            if(!selectedVisible) { CameraState previous; if(cameras.TryGetValue(camera,out previous)) previous.Buffer.Clear(); return; }
            long start=Stopwatch.GetTimestamp();
            CameraCallbacks++;
            try
            {
                if(timedFrame!=Time.frameCount) { timedFrame=Time.frameCount; CameraCpuMilliseconds=0; }
                CameraState state;
                if(!cameras.TryGetValue(camera,out state))
                { state=new CameraState { Camera=camera,Previous=camera.depthTextureMode,Buffer=new CommandBuffer { name="BPN textured hidden-region pass" } }; camera.depthTextureMode|=DepthTextureMode.Depth; camera.AddCommandBuffer(CameraEvent.BeforeImageEffects,state.Buffer); cameras.Add(camera,state); }
                if(state.Width!=camera.pixelWidth||state.Height!=camera.pixelHeight)
                {
                    ReleaseTextures(state); state.Width=Math.Max(1,camera.pixelWidth); state.Height=Math.Max(1,camera.pixelHeight);
                    state.SourceDepth=Depth(state.Width,state.Height); state.ReceiverDepth=Depth(state.Width,state.Height); state.ExtraDepth=Depth(state.Width,state.Height);
                }
                var buffer=state.Buffer; buffer.Clear(); var view=camera.worldToCameraMatrix; var projection=GL.GetGPUProjectionMatrix(camera.projectionMatrix,camera.targetTexture!=null); var rtProjection=GL.GetGPUProjectionMatrix(camera.projectionMatrix,true);
                posed.Clear();
                foreach(var surface in surfaces.Values) if(SelectedVisible(camera,surface))
                {
                    Surface existing;
                    if(posed.TryGetValue(surface.Renderer,out existing)) { surface.Mesh=existing.Mesh; surface.Matrix=existing.Matrix; }
                    else { surface.Mesh=surface.Snapshot(); surface.Matrix=AnimationAdapter.BakedToWorld(surface.Renderer); posed.Add(surface.Renderer,surface); }
                    for(int slot=0;slot<surface.Materials.Length;slot++) UpdateMaterial(surface,slot);
                }
                buffer.SetGlobalFloat("_BPNFar",camera.farClipPlane); buffer.SetGlobalFloat("_BPNFlip",Mathf.Sign(rtProjection.m11*projection.m11));
                DrawDepth(buffer,camera,state.SourceDepth,true,view,rtProjection); DrawDepth(buffer,camera,state.ReceiverDepth,false,view,rtProjection);
                buffer.SetRenderTarget(state.ExtraDepth); buffer.ClearRenderTarget(true,true,Color.white); buffer.SetViewProjectionMatrices(view,rtProjection);
                if(Time.unscaledTime>=nextOccluderScan) { nextOccluderScan=Time.unscaledTime+1; occluders.Clear(); foreach(var renderer in Resources.FindObjectsOfTypeAll<SkinnedMeshRenderer>()) if(renderer!=null&&renderer.gameObject.scene.IsValid()) occluders.Add(renderer); }
                // Opaque scene geometry is already covered by camera depth. This
                // private pass includes actual other-character skins and explicitly
                // supplied additional renderers without changing their materials.
                int drawn=0; foreach(var renderer in occluders) if(Visible(camera,renderer)&&!SelectedSource(renderer)&&drawn++<128) for(int slot=0;slot<renderer.sharedMesh.subMeshCount;slot++) buffer.DrawRenderer(renderer,depthMaterial,slot,0);
                foreach(var renderer in AdditionalOccluders) if(Visible(camera,renderer)&&!Selected(renderer)) buffer.DrawRenderer(renderer,depthMaterial,0,0);
                buffer.SetRenderTarget(BuiltinRenderTextureType.CameraTarget); buffer.SetViewProjectionMatrices(view,projection); buffer.SetGlobalTexture("_BPNExtraDepth",state.ExtraDepth);
                drawnRenderers.Clear();
                // Receiver first, Source last: registration order must not hide
                // an internal Source under the Receiver's translucent overlay.
                for(int pass=0;pass<2;pass++)foreach(var surface in surfaces.Values)
                {
                    if(surface.Source!=(pass==1))continue;
                    if(!SelectedVisible(camera,surface)||surface.Mesh==null||(Target==XRayTarget.Source&&!surface.Source)||(Target==XRayTarget.Receiver&&surface.Source)) continue;
                    if(!drawnRenderers.Add(surface.Renderer)) continue;
                    buffer.SetGlobalTexture("_BPNSelfDepth",surface.Source?state.SourceDepth:state.ReceiverDepth); buffer.SetGlobalTexture("_BPNOtherDepth",surface.Source?state.ReceiverDepth:state.SourceDepth);
                    for(int slot=0;slot<surface.Mesh.subMeshCount;slot++)
                    {
                        if(surface.Mesh.GetIndexCount(slot)==0) continue;
                        if(Occlusion==OcclusionMode.AllSelected) buffer.DrawMesh(surface.Mesh,surface.Matrix,surface.Materials[slot],slot,3);
                        else { buffer.DrawMesh(surface.Mesh,surface.Matrix,surface.Materials[slot],slot,1); buffer.DrawMesh(surface.Mesh,surface.Matrix,surface.Materials[slot],slot,2); }
                    }
                }
            }
            catch(Exception e) { Enabled=false; ClearCameras(); UnityEngine.Debug.LogWarning("Next X-Ray stopped; physics continues: "+e.Message); }
            finally { LastCameraCpuMilliseconds=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency; CameraCpuMilliseconds+=LastCameraCpuMilliseconds; }
        }
        bool Selected(Renderer renderer) { foreach(var surface in surfaces.Values) if(surface.Renderer==renderer) return true; return false; }
        bool SelectedSource(Renderer renderer) { foreach(var surface in surfaces.Values) if(surface.Renderer==renderer&&surface.Source) return true; return false; }
        void UpdateMaterial(Surface surface,int slot)
        {
            var mat=surface.Materials[slot]; surface.Renderer.GetSharedMaterials(materials); var original=materials.Count>0?materials[Math.Min(slot,materials.Count-1)]:null;
            block.Clear(); surface.Renderer.GetPropertyBlock(block,slot); Texture texture=block.GetTexture("_MainTex"); if(texture==null&&original!=null&&original.HasProperty("_MainTex")) texture=original.GetTexture("_MainTex");
            mat.SetTexture("_MainTex",texture??Texture2D.whiteTexture); if(original!=null&&original.HasProperty("_MainTex")) { mat.SetTextureScale("_MainTex",original.GetTextureScale("_MainTex")); mat.SetTextureOffset("_MainTex",original.GetTextureOffset("_MainTex")); }
            mat.SetColor("_Tint",surface.Source?SourceColor:ReceiverColor); mat.SetFloat("_Opacity",Mathf.Clamp01(surface.Source?SourceOpacity:ReceiverOpacity)); mat.SetFloat("_TextureBlend",Mathf.Clamp01(OriginalTextureBlend));
            mat.SetFloat("_Dots",DotPatternEnabled?1:0); mat.SetFloat("_DotScale",Mathf.Max(2,DotScale)); mat.SetFloat("_DotDensity",Mathf.Clamp01(DotDensity));
        }
        void DrawDepth(CommandBuffer buffer,Camera camera,RenderTexture target,bool source,Matrix4x4 view,Matrix4x4 projection)
        { buffer.SetRenderTarget(target); buffer.ClearRenderTarget(true,true,Color.white); buffer.SetViewProjectionMatrices(view,projection); drawnRenderers.Clear(); foreach(var s in surfaces.Values) if(s.Source==source&&SelectedVisible(camera,s)&&s.Mesh!=null&&drawnRenderers.Add(s.Renderer)) for(int slot=0;slot<s.Mesh.subMeshCount;slot++) if(s.Mesh.GetIndexCount(slot)>0) buffer.DrawMesh(s.Mesh,s.Matrix,depthMaterial,slot,0); }
        static void ReleaseTextures(CameraState s) { foreach(var t in new[]{s.SourceDepth,s.ReceiverDepth,s.ExtraDepth}) if(t!=null) { t.Release(); Object.Destroy(t); } s.SourceDepth=s.ReceiverDepth=s.ExtraDepth=null; }
        public void ClearCameras()
        {
            foreach(var state in cameras.Values) { if(state.Camera!=null) { state.Camera.RemoveCommandBuffer(CameraEvent.BeforeImageEffects,state.Buffer); if(state.Camera.depthTextureMode==(state.Previous|DepthTextureMode.Depth)) state.Camera.depthTextureMode=state.Previous; } state.Buffer.Release(); ReleaseTextures(state); } cameras.Clear();
        }
        public void Tick()
        { if(!Enabled) { if(cameras.Count>0) ClearCameras(); return; } dead.Clear(); foreach(var item in cameras) if(item.Key==null) dead.Add(item.Key); foreach(var camera in dead) { var s=cameras[camera]; s.Buffer.Release(); ReleaseTextures(s); cameras.Remove(camera); } }
        public void Dispose() { Camera.onPreCull-=OnCamera; ClearCameras(); foreach(var s in surfaces.Values) foreach(var m in s.Materials) Object.Destroy(m); surfaces.Clear(); Object.Destroy(depthMaterial); if(bundle!=null) bundle.Unload(true); }
    }
}
