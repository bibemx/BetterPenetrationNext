using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        [Serializable] public sealed class LandmarkNativeTiming
        {
            public string scope="Neutral two-bone open pipe; actual Unity BakeMesh, not a user-scene acceptance test";
            public string unity,gpu;public int vertices,triangles,sections,framesPerCase,submitted,dropped,completed,workerThread,mainThread;
            public double maxNativeSkinError,maxBindingError,maxSeamGap,maxWorkerSnapshotError;
            public double[] fullFrameMeanMs,fullFrameP95Ms,captureMeanMs,mainLandmarkMeanMs;
            public double workerMeanMs;public string[] cases=new[]{"animated capture only","synchronous landmarks","background landmarks"};
        }

        IEnumerator RunLandmarkNative()
        {
            float readyDeadline=Time.realtimeSinceStartup+30;
            while(UnityEngine.SceneManagement.SceneManager.GetActiveScene().name!="Title"&&Time.realtimeSinceStartup<readyDeadline)yield return null;
            Require(UnityEngine.SceneManagement.SceneManager.GetActiveScene().name=="Title","neutral native probe waits for the loaded engine scene");
            for(int settle=0;settle<30;settle++)yield return null;
            const int rings=37,sides=120,stride=sides+1,count=rings*stride,frames=120;
            var rest=new Vec3[count];var faces=new int[(rings-1)*sides*6];int f=0;
            for(int r=0;r<rings;r++)for(int s=0;s<stride;s++)rest[r*stride+s]=new Vec3(.2*Math.Cos(s*2*Math.PI/sides),.1*Math.Sin(s*2*Math.PI/sides),2.0*r/(rings-1));
            for(int r=0;r<rings-1;r++)for(int s=0;s<sides;s++){int a=r*stride+s,b=a+1,c=a+stride,d=b+stride;faces[f++]=a;faces[f++]=b;faces[f++]=c;faces[f++]=b;faces[f++]=d;faces[f++]=c;}
            var renderer=Skin("Neutral engineering pipe",new MeshData{Vertices=rest,Triangles=faces},Vector3.zero);renderer.enabled=false;
            Mesh mesh=renderer.sharedMesh;Material material=renderer.sharedMaterial;var first=renderer.bones[0];var second=new GameObject("Neutral second carrier").transform;second.SetParent(renderer.transform,false);
            renderer.bones=new[]{first,second};mesh.bindposes=new[]{first.worldToLocalMatrix*renderer.transform.localToWorldMatrix,second.worldToLocalMatrix*renderer.transform.localToWorldMatrix};
            var weights=new BoneWeight[count];var morph=new Vector3[count];
            for(int i=0;i<count;i++){float w=(float)(rest[i].Z*.5);weights[i]=new BoneWeight{boneIndex0=0,weight0=1-w,boneIndex1=1,weight1=w};morph[i]=new Vector3((float)rest[i].X*.25f,(float)rest[i].Y*.25f,0);}
            int seamA=(rings/2)*stride,seamB=seamA+sides;weights[seamA]=new BoneWeight{boneIndex0=0,weight0=1};weights[seamB]=new BoneWeight{boneIndex0=1,weight0=1};mesh.boneWeights=weights;
            morph[count-7].z=.4f;mesh.AddBlendShapeFrame("Neutral radial and terminal morph",100,morph,new Vector3[count],new Vector3[count]);
            var baked=new Mesh{name="Neutral animated bake buffer"};assets.Add(baked);var bakedVertices=new List<Vector3>(count);var pose=new Vec3[count];
            var sync=new AdaptiveMeshLandmarks(rest,faces,new Vec3(0,0,1));var workerPoints=new AdaptiveMeshLandmarks(rest,faces,new Vec3(0,0,1));int triangleCuts=sync.ReferenceTriangleTests;
            var workerInput=new ReactiveRigInput(0,0,count);var workerOutput=new ReactiveRigOutput(0,0,count,2);
            double error=0,bindingError=0,seamGap=0,snapshotError=0,workerTotal=0;int workerSamples=0,workerThread=0;
            var interval=new[]{new double[frames],new double[frames],new double[frames]};var capture=new double[3];var numeric=new double[3];
            int oldRate=Application.targetFrameRate,oldVsync=QualitySettings.vSyncCount;Application.targetFrameRate=60;QualitySettings.vSyncCount=0;
            var worker=new ReactiveRigWorker((input,result)=>{
                long start=Stopwatch.GetTimestamp();workerPoints.Update(input.Source,new Vec3(1,0,0));
                Array.Copy(input.Source,result.SourceWorld,input.Source.Length);result.Frame=input.Frame;result.WorkerThread=System.Threading.Thread.CurrentThread.ManagedThreadId;
                result.TotalMs=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency;
            },2,0,0,0,0,count);
            var submittedPoints=new Vec3[frames+30];bool workerFrameValid=true;
            try
            {
                for(int mode=0;mode<3;mode++)
                {
                    long previous=0;
                    for(int sample=-30;sample<frames;sample++)
                    {
                        if(worker.TryRead(workerOutput)){
                            if(workerOutput.Frame<0||workerOutput.Frame>=submittedPoints.Length)workerFrameValid=false;
                            else snapshotError=Math.Max(snapshotError,(workerOutput.SourceWorld[0]-submittedPoints[workerOutput.Frame]).Length);
                            workerTotal+=workerOutput.TotalMs;workerSamples++;workerThread=workerOutput.WorkerThread;
                        }
                        int n=sample+30;float a=Mathf.Sin(n*.07f);second.localRotation=Quaternion.Euler(a*12,a*6,a*8);second.localPosition=new Vector3(0,.05f+a*.02f,0);float blend=45+25*Mathf.Sin(n*.043f);renderer.SetBlendShapeWeight(0,blend);
                        long stamp=Stopwatch.GetTimestamp();renderer.BakeMesh(baked);baked.GetVertices(bakedVertices);
                        for(int i=0;i<count;i++)pose[i]=AnimationAdapter.ToCore(bakedVertices[i]);
                        double captureMs=(Stopwatch.GetTimestamp()-stamp)*1000.0/Stopwatch.Frequency;stamp=Stopwatch.GetTimestamp();
                        if(mode==1)sync.Update(pose,new Vec3(1,0,0));
                        else if(mode==2){Array.Copy(pose,workerInput.Source,count);workerInput.Frame=n;workerInput.Dt=1.0/60;if(worker.Submit(workerInput))submittedPoints[n]=pose[0];}
                        double numericMs=(Stopwatch.GetTimestamp()-stamp)*1000.0/Stopwatch.Frequency;
                        if(sample>=0){capture[mode]+=captureMs/frames;numeric[mode]+=numericMs/frames;}
                        if(sample==frames/2)
                        {
                            Matrix4x4 m0=renderer.transform.worldToLocalMatrix*first.localToWorldMatrix*mesh.bindposes[0],m1=renderer.transform.worldToLocalMatrix*second.localToWorldMatrix*mesh.bindposes[1];
                            for(int i=0;i<count;i++){Vector3 p=AnimationAdapter.ToUnity(rest[i])+morph[i]*(blend*.01f);var w=weights[i];Vector3 expected=(w.boneIndex0==0?m0:m1).MultiplyPoint3x4(p)*w.weight0+(w.boneIndex1==0?m0:m1).MultiplyPoint3x4(p)*w.weight1;error=Math.Max(error,(expected-bakedVertices[i]).magnitude);}
                            seamGap=Math.Max(seamGap,(pose[seamA]-pose[seamB]).Length);
                            if(mode==1)foreach(var section in sync.Sections)for(int i=0;i<section.Bindings.Length;i++)bindingError=Math.Max(bindingError,(section.Surface[i]-section.Bindings[i].Evaluate(pose)).Length);
                        }
                        yield return null;
                        long now=Stopwatch.GetTimestamp();if(sample>=0&&previous!=0)interval[mode][sample]=(now-previous)*1000.0/Stopwatch.Frequency;previous=now;
                    }
                }
                var result=new LandmarkNativeTiming{unity=Application.unityVersion,gpu=SystemInfo.graphicsDeviceName,vertices=count,triangles=faces.Length/3,sections=sync.Sections.Length,framesPerCase=frames,submitted=worker.Submitted,dropped=worker.Dropped,completed=worker.Completed,workerThread=workerThread,mainThread=System.Threading.Thread.CurrentThread.ManagedThreadId,maxNativeSkinError=error,maxBindingError=bindingError,maxSeamGap=seamGap,maxWorkerSnapshotError=snapshotError,fullFrameMeanMs=new double[3],fullFrameP95Ms=new double[3],captureMeanMs=capture,mainLandmarkMeanMs=numeric,workerMeanMs=workerSamples==0?0:workerTotal/workerSamples};
                for(int mode=0;mode<3;mode++){Array.Sort(interval[mode]);foreach(double ms in interval[mode])result.fullFrameMeanMs[mode]+=ms/frames;result.fullFrameP95Ms[mode]=interval[mode][(int)Math.Ceiling(frames*.95)-1];}
                File.WriteAllText(output+".landmarks.json",JsonUtility.ToJson(result,true));
                Require(error<1e-5,"actual two-bone skinning and BlendShapes match analytic geometry","maximum error="+error);
                Require(bindingError<1e-12,"native surface points retain cached edge provenance","maximum error="+bindingError);
                Require(seamGap>.001,"coincident native seam vertices retain different bone weights","maximum gap="+seamGap);
                Require(sync.ReferenceTriangleTests==triangleCuts,"native animation does not rebuild triangle cuts");
                Require(workerSamples>0&&workerThread!=System.Threading.Thread.CurrentThread.ManagedThreadId,"numerical landmarks execute off the Unity thread","worker="+workerThread);
                Require(workerFrameValid&&snapshotError<1e-12,"native snapshots retain frame identity without aliasing mutable buffers","error="+snapshotError);
                Require(renderer.sharedMesh==mesh&&renderer.sharedMaterial==material,"neutral checks preserve renderer mesh and material references");
            }
            finally{worker.Dispose();Application.targetFrameRate=oldRate;QualitySettings.vSyncCount=oldVsync;}
        }
    }
}
