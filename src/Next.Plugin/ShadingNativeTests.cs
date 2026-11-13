using System;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        void RunShadingScaleNative()
        {
            foreach(double scale in new[]{.0001,.001,1.0})
            {
                var data=new MeshData{Vertices=new[]{new Vec3(),new Vec3(scale,0,0),new Vec3(0,scale,0)},Triangles=new[]{0,1,2}};
                var renderer=Skin("Scale-independent normal probe",data,Vector3.zero);var mesh=renderer.sharedMesh;mesh.normals=new[]{Vector3.forward,Vector3.forward,Vector3.forward};mesh.tangents=new[]{new Vector4(1,0,0,1),new Vector4(1,0,0,1),new Vector4(1,0,0,1)};
                var definition=Definition(mesh,renderer,data);var motion=new Vec3[3];var rotation=Quaternion.Euler(67,0,0);
                for(int i=0;i<3;i++)motion[i]=AnimationAdapter.ToCore(rotation*AnimationAdapter.ToUnity(data.Vertices[i]))-data.Vertices[i];
                using(var mapping=new MeshDeformationBinding(renderer,definition))
                {
                    mapping.ApplyControlOffsets(motion,scale*3,1);var normals=renderer.sharedMesh.normals;var tangents=renderer.sharedMesh.tangents;Vector3 expected=rotation*Vector3.forward;double error=0,orthogonality=0;
                    for(int i=0;i<3;i++){error=Math.Max(error,(normals[i]-expected).magnitude);orthogonality=Math.Max(orthogonality,Math.Abs(Vector3.Dot(normals[i],new Vector3(tangents[i].x,tangents[i].y,tangents[i].z))));}
                    Require(error<1e-5&&orthogonality<1e-5,"native normal transport follows deformation at scale "+scale,"normalError="+error+" tangentDot="+orthogonality);
                }
                renderer.gameObject.SetActive(false);
            }
        }
        void RunShadingSeamNative()
        {
            foreach(int mode in new[]{0,1,2,3})
            {
                var controls=new MeshData{Vertices=new[]{new Vec3(),new Vec3(.02,0,0),new Vec3(.02,.02,0),new Vec3(0,.02,0)},Triangles=new[]{0,1,2,0,2,3}};
                var visual=new MeshData{Vertices=new[]{controls.Vertices[0],controls.Vertices[1],controls.Vertices[2],controls.Vertices[0],controls.Vertices[2],controls.Vertices[3]},Triangles=mode==3?new[]{0,1,2,3,5,4}:new[]{0,1,2,3,4,5}};
                var renderer=Skin("Independent normal seam probe "+mode,visual,Vector3.zero);var mesh=renderer.sharedMesh;mesh.normals=new[]{Vector3.forward,Vector3.forward,Vector3.forward,Vector3.forward,Vector3.forward,Vector3.forward};var tangent=new Vector4[6];for(int i=0;i<6;i++)tangent[i]=new Vector4(i<3?1:-1,0,0,i<3?1:-1);mesh.tangents=tangent;
                if(mode==1)
                {
                    var other=new GameObject("Independent coincident skin").transform;other.SetParent(renderer.transform,false);renderer.bones=new[]{renderer.bones[0],other};mesh.bindposes=new[]{mesh.bindposes[0],other.worldToLocalMatrix*renderer.transform.localToWorldMatrix};var skin=mesh.boneWeights;for(int i=3;i<skin.Length;i++)skin[i]=new BoneWeight{boneIndex0=1,weight0=1};mesh.boneWeights=skin;
                }
                if(mode==2){var delta=new Vector3[6];delta[3]=delta[4]=new Vector3(0,0,.001f);mesh.AddBlendShapeFrame("Independent seam morph",100,delta,new Vector3[6],new Vector3[6]);}
                var definition=Definition(mesh,renderer,controls);var offsets=new Vec3[4];offsets[1]=new Vec3(0,0,.01);
                using(var mapping=new MeshDeformationBinding(renderer,definition))
                {
                    mapping.ApplyControlOffsets(offsets,.02,1);var normals=renderer.sharedMesh.normals;double difference=Math.Max((normals[0]-normals[3]).magnitude,(normals[2]-normals[4]).magnitude);
                    if(mode==0)Require(mapping.NormalSeamGroupCount==2&&difference<1e-6,"UV copies share a smooth native shading frame under non-affine deformation","normalGap="+difference);
                    else Require(mapping.NormalSeamGroupCount==0&&difference>.01,"normal seams preserve "+(mode==1?"independent skin weights":mode==2?"independent morphs":"opposite material faces"),"groups="+mapping.NormalSeamGroupCount+" normalGap="+difference);
                }
                renderer.gameObject.SetActive(false);
            }
        }
    }
}
