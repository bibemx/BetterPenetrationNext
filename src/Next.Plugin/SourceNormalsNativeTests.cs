using System;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        void RunSourceNormals()
        {
            var grid=PrimitiveMeshes.Grid(2,2,.1,.1);var r=Skin("Neutral reversible source normals",grid,Vector3.zero);var original=r.sharedMesh;
            var authored=new Vector3[original.vertexCount];var delta=new Vector3[authored.Length];var deltaNormals=new Vector3[authored.Length];var zero=new Vector3[authored.Length];
            for(int i=0;i<authored.Length;i++){authored[i]=new Vector3(.1f,.2f,1).normalized;delta[i]=new Vector3(0,0,.002f*i);deltaNormals[i]=new Vector3(.03f,-.02f,.01f);}
            original.normals=authored;original.AddBlendShapeFrame("Source normal morph",100,delta,deltaNormals,zero);r.SetBlendShapeWeight(0,37);r.bones[0].localRotation=Quaternion.Euler(7,11,0);
            var before=new Mesh();var after=new Mesh();assets.Add(before);assets.Add(after);r.BakeMesh(before);var world=before.vertices;var matrix=AnimationAdapter.BakedToWorld(r);var animated=new Vec3[world.Length];for(int i=0;i<world.Length;i++)animated[i]=AnimationAdapter.ToCore(matrix.MultiplyPoint3x4(world[i]));
            var body=new SourceContactBody{MaximumDisplacement=.1,Pivot=animated[0],RotationVector=new Vec3(.15,0,0)};
            using(var motion=new SourceMotionBinding(r,original.triangles)){
                motion.Apply(body,animated);var deformed=motion.Working.normals;double changed=0;for(int i=0;i<authored.Length;i++)changed=Math.Max(changed,(deformed[i]-authored[i]).magnitude);
                Require(changed>.01,"1.0.6 source deformation changes actual private mesh normals");
                motion.RestoreBaseline();var restored=motion.Working.normals;double error=0;for(int i=0;i<authored.Length;i++)error=Math.Max(error,(restored[i]-authored[i]).magnitude);
                Require(error<1e-7,"1.0.6 source baseline restores exact authored normals");
                r.BakeMesh(after);var beforeNormals=before.normals;var afterNormals=after.normals;error=0;for(int i=0;i<beforeNormals.Length;i++)error=Math.Max(error,(beforeNormals[i]-afterNormals[i]).magnitude);
                Require(error<.00003&&Math.Abs(r.GetBlendShapeWeight(0)-37)<1e-6,"1.0.6 restored normals preserve native bone and morph normal contributions","maximumError="+error);
                motion.Apply(body,animated);body.RotationVector=new Vec3();motion.Apply(body,animated);restored=motion.Working.normals;error=0;for(int i=0;i<authored.Length;i++)error=Math.Max(error,(restored[i]-authored[i]).magnitude);
                Require(error<1e-7,"1.0.6 zero correction restores normals without requiring a separate baseline call");
            }
            Require(r.sharedMesh==original&&Math.Abs(r.GetBlendShapeWeight(0)-37)<1e-6,"1.0.6 source normals dispose preserves original mesh and morph weight");r.enabled=false;
        }
    }
}
