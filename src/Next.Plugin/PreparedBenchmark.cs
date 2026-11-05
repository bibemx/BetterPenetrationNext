using System;
using System.Collections;
using System.Diagnostics;
using BetterPenetrationNext.Core;
using UnityEngine;
using UnityEngine.Rendering;
using Object=UnityEngine.Object;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        [Serializable] public sealed class PerformanceRow
        {
            public string scope="actual KKS, prepared user mesh, neutral animated sphere; not gameplay FPS",gpu,cpu,quality="Balanced";
            public int vertices,particles,samples=120,width=1280,height=720,activeContacts,gen0Collections,skinningGroups,normalVertices,contactQueries;
            public bool xray;
            public string contactMode;
            public double maximumResidualPenetration,maximumEdgeStrain,maximumSourceCorrection;
            public double captureMean,captureP95,refitMean,refitP95,solveMean,solveP95,applyMean,applyP95,xrayMean,xrayP95,totalMean,totalP95,cameraRenderMean,cameraRenderP95;
            public double gpuFrameMean=-1,allocatedBytesMean=-1,maximumAppliedPoseError,probeRadius=0.025;
            public long monoBytes,unityAllocatedBytes,cameraCallbacks;
            public float[] probeCenter;
        }
        IEnumerator BenchmarkPrepared(SkinnedMeshRenderer receiver,SimulationDefinition definition,SoftBodySolver solver,AnimationAdapter animation,MeshDeformationBinding binding)
        {
            bool graphics=SystemInfo.graphicsDeviceType!=GraphicsDeviceType.Null;
            var stats=new PerformanceWindow(120); var renderStats=new PerformanceWindow(120); root.transform.localScale=Vector3.one; receiver.gameObject.layer=29; receiver.localBounds=receiver.sharedMesh.bounds;
            animation.Capture(null); solver.Reset(animation.Anchors);
            int anchor=Array.IndexOf(definition.BoneNames,"MED_Vaginal_Mid"); if(anchor<0) throw new InvalidOperationException("Common authored benchmark anchor absent.");
            var center=AnimationAdapter.ToCore(receiver.bones[anchor].position);
            var probe=Skin("BPN neutral animated benchmark sphere",PrimitiveMeshes.Sphere(24,12,0.025),Vector3.zero); probe.transform.position=AnimationAdapter.ToUnity(center); probe.enabled=false;
            var weights=new double[probe.sharedMesh.vertexCount]; for(int i=0;i<weights.Length;i++) weights[i]=1;
            var contact=new ContactDetection(probe,weights); Camera camera=null; RenderTexture target=null; XRayRenderer effect=null; GameObject cameraObject=null;
            var settings=new SoftBodySettings { PreserveVolume=definition.PreserveVolume };
            var coupledSettings=new BidirectionalContactSettings(); var contactBody=new SourceContactBody { Surface=contact };
            var timing=new FrameTiming[1]; double gpuSum=0; int gpuCount=0;
            Func<long> allocated=null; var method=typeof(GC).GetMethod("GetAllocatedBytesForCurrentThread",Type.EmptyTypes); if(method!=null) allocated=(Func<long>)Delegate.CreateDelegate(typeof(Func<long>),method);
            try
            {
                if(graphics)
                {
                    cameraObject=new GameObject("BPN prepared comparison camera"); cameraObject.transform.SetParent(root.transform,false); cameraObject.transform.localPosition=receiver.sharedMesh.bounds.center+new Vector3(0,0,-3);
                    camera=cameraObject.AddComponent<Camera>(); camera.orthographic=true; camera.orthographicSize=0.9f; camera.nearClipPlane=0.1f; camera.farClipPlane=20; camera.cullingMask=1<<29; camera.clearFlags=CameraClearFlags.SolidColor; camera.backgroundColor=Color.gray;
                    camera.allowHDR=false; camera.allowMSAA=false; target=new RenderTexture(1280,720,24,RenderTextureFormat.ARGB32); target.Create(); camera.targetTexture=target;
                    effect=new XRayRenderer { Target=XRayTarget.Receiver,ReceiverOpacity=0.35f,DotDensity=0.5f,OriginalTextureBlend=1 }; effect.Register(binding,receiver,false,binding.XRaySnapshot);
                }
                foreach(bool coupled in new[]{false,true}) for(int state=0;state<(graphics?2:1);state++)
                {
                    contact.ReleaseCorrection(); contactBody.Offset=new Vec3();
                    bool enabled=state==1; if(effect!=null) { effect.Enabled=enabled; effect.Tick(); }
                    stats=new PerformanceWindow(120); renderStats=new PerformanceWindow(120); solver.Reset(animation.Anchors); int contacts=0,collections=GC.CollectionCount(0); long allocationSum=0,callbacks=effect==null?0:effect.CameraCallbacks; gpuSum=0; gpuCount=0;
                    for(int frame=0;frame<150;frame++)
                    {
                        probe.transform.position=AnimationAdapter.ToUnity(center)+new Vector3(0.002f*Mathf.Sin(frame*0.04f),0,0.002f*Mathf.Cos(frame*0.04f));
                        long bytesBefore=allocated==null?0:allocated(); long start=Stopwatch.GetTimestamp(); contact.Capture(); animation.Capture(solver); long captured=Stopwatch.GetTimestamp();
                        if(coupled) contactBody.BeginFrame(coupledSettings,1.0/60);
                        solver.Advance(animation.Anchors,coupled?(IContactSurface)contactBody:contact,settings,PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,animation.PosedSize,coupled?coupledSettings:null); long solved=Stopwatch.GetTimestamp();
                        binding.Apply(solver,animation.PosedSize*0.15,3); if(coupled) contact.ApplyCorrection(contactBody); long applied=Stopwatch.GetTimestamp(); long cameraStart=Stopwatch.GetTimestamp(); if(camera!=null) camera.Render(); long cameraEnd=Stopwatch.GetTimestamp();
                        if(frame>=30)
                        {
                            double xr=enabled?effect.LastCameraCpuMilliseconds:0;
                            stats.Add(new FrameMeasurement { Capture=Milliseconds(start,captured)-contact.RefitMilliseconds,Refit=contact.RefitMilliseconds,Solve=Milliseconds(captured,solved),Apply=Milliseconds(solved,applied),XRay=xr,Total=Milliseconds(start,applied)+xr });
                            renderStats.Add(new FrameMeasurement { Total=Milliseconds(cameraStart,cameraEnd) }); contacts=Math.Max(contacts,solver.ContactCount);
                            if(allocated!=null) allocationSum+=allocated()-bytesBefore;
                            if(graphics) { FrameTimingManager.CaptureFrameTimings(); if(FrameTimingManager.GetLatestTimings(1,timing)>0&&timing[0].gpuFrameTime>0) { gpuSum+=timing[0].gpuFrameTime; gpuCount++; } }
                        }
                        yield return null;
                    }
                    var c=stats.Time(0); var r=stats.Time(1); var s=stats.Time(2); var a=stats.Time(3); var x=stats.Time(4); var t=stats.Time(5); var cameraTime=renderStats.Time(5);
                    var row=new PerformanceRow { contactMode=coupled?"Bidirectional":"OneWay",maximumResidualPenetration=solver.MaximumResidualPenetration,maximumEdgeStrain=solver.MaximumEdgeStrain,maximumSourceCorrection=contactBody.Offset.Length,vertices=receiver.sharedMesh.vertexCount,particles=solver.Positions.Length,xray=enabled,gpu=SystemInfo.graphicsDeviceName,cpu=SystemInfo.processorType,activeContacts=contacts,skinningGroups=binding.SkinningGroupCount,normalVertices=binding.NormalVertexCount,contactQueries=solver.ContactQueries,
                        captureMean=c.Mean,captureP95=c.P95,refitMean=r.Mean,refitP95=r.P95,solveMean=s.Mean,solveP95=s.P95,applyMean=a.Mean,applyP95=a.P95,xrayMean=x.Mean,xrayP95=x.P95,totalMean=t.Mean,totalP95=t.P95,
                        cameraRenderMean=cameraTime.Mean,cameraRenderP95=cameraTime.P95,gpuFrameMean=gpuCount==0?-1:gpuSum/gpuCount,allocatedBytesMean=allocated==null?-1:(double)allocationSum/120,gen0Collections=GC.CollectionCount(0)-collections,
                        monoBytes=UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong(),unityAllocatedBytes=UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong(),cameraCallbacks=effect==null?0:effect.CameraCallbacks-callbacks,
                        maximumAppliedPoseError=binding.VerifyAppliedPose(solver),probeCenter=new[]{(float)center.X,(float)center.Y,(float)center.Z} };
                    report.performance.Add(row); Require(contacts>0,"prepared comparison contains active neutral contacts; X-Ray="+enabled,"contacts="+contacts);
                    Require(!enabled||row.cameraCallbacks>=150,"X-Ray camera callbacks are observed for every comparison frame");
                    Require(row.maximumAppliedPoseError<0.0003,"prepared comparison verifies actual skinned output; X-Ray="+enabled,"error="+row.maximumAppliedPoseError);
                }
            }
            finally { if(effect!=null) effect.Dispose(); contact.Dispose(); if(cameraObject!=null) Object.Destroy(cameraObject); if(target!=null) { target.Release(); Object.Destroy(target); } probe.enabled=false; }
        }
        static double Milliseconds(long from,long to) { return (to-from)*1000.0/Stopwatch.Frequency; }
    }
}
