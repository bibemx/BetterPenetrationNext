using System;
using System.Diagnostics;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    // Evaluated after the scoped entrance-bone controller and before the same
    // MeshDeformationBinding upload as the internal Receiver. Captures only the
    // cached SAC material subset; no BakeMesh, scene search or topology rebuild.
    public sealed class NativeClosedEntranceBinding
    {
        readonly SkinnedMeshRenderer renderer;
        readonly SimulationDefinition support;
        readonly ClosedEntranceDefinition data;
        readonly ReceiverGuideDefinition guide;
        readonly Transform[] bones;
        readonly Matrix4x4[] bindposes,matrices;
        readonly Vec3[] native;
        readonly double[] radii=new double[ReceiverRadialField.Directions];
        readonly EntranceContactProxy proxy=new EntranceContactProxy();
        public readonly double ContactDepth;
        readonly int root;
        public readonly ClosedEntranceConstraint Constraint;
        public double LastMilliseconds;
        public NativeClosedEntranceBinding(SkinnedMeshRenderer renderer,SimulationDefinition support,SimulationDefinition physical,ReceiverGuideDefinition guide,ClosedEntranceDefinition data)
        {
            this.renderer=renderer;this.support=support;this.data=data;this.guide=guide;Constraint=new ClosedEntranceConstraint(data,support,physical,guide);
            bones=renderer.bones;bindposes=renderer.sharedMesh.bindposes;matrices=new Matrix4x4[bones.Length];native=new Vec3[data.Controls.Length];root=Array.IndexOf(support.BoneNames,"MED_Pelvis");
            if(root<0||bones.Length!=bindposes.Length)throw new ArgumentException("Closed entrance native carrier differs.");
            ContactDepth=Constraint.ContactDepth;
        }
        public void Apply(Vec3[] physical,ReactiveRigInput input,ReactiveRigOutput result,Vec3[] offsets,double dt)
        {
            long started=Stopwatch.GetTimestamp();
            for(int i=0;i<bones.Length;i++){if(bones[i]==null)throw new InvalidOperationException("SAC native bone was destroyed.");matrices[i]=bones[i].localToWorldMatrix*bindposes[i];}
            for(int j=0;j<native.Length;j++)
            {
                int i=data.Controls[j];Vec3 q=new Vec3();Vector3 rest=AnimationAdapter.ToUnity(support.Rest[i]);
                for(int k=support.InfluenceOffsets[i];k<support.InfluenceOffsets[i+1];k++)q+=AnimationAdapter.ToCore(matrices[support.BoneIndices[k]].MultiplyPoint3x4(rest))*support.BoneWeights[k];native[j]=q;
            }
            Vec3 centre=new Vec3(),next=new Vec3();foreach(int i in guide.Boundaries[0])centre+=physical[i]/guide.Boundaries[0].Length;var section=guide.Contours[1];for(int j=0;j<section.Vertices.Length;j++)next+=physical[section.Vertices[j]]*section.Weights[j];
            var carrier=AnimationAdapter.Linear(matrices[root]);double materialScale=(carrier.X.Length+carrier.Y.Length+carrier.Z.Length)/3;
            Vec3 axis=next-centre,side=carrier.X;double[] profile=null;double rotation=0;Vec3? contactCentre=null;
            if(result.EntranceProfileAvailable)
            {
                axis=carrier.Multiply(result.EntranceNormalLocal).Normalized;side=(side-axis*Vec3.Dot(side,axis)).Normalized;Vec3 profileSide=carrier.Multiply(result.EntranceSideLocal);rotation=Math.Atan2(Vec3.Dot(profileSide,Vec3.Cross(axis,side)),Vec3.Dot(profileSide,side));
                contactCentre=AnimationAdapter.ToCore(matrices[root].MultiplyPoint3x4(AnimationAdapter.ToUnity(result.EntranceCentreLocal)));
                double previousScale=(result.Carrier.X.Length+result.Carrier.Y.Length+result.Carrier.Z.Length)/3,ratio=previousScale>1e-9?materialScale/previousScale:1;
                for(int i=0;i<radii.Length;i++)radii[i]=result.EntranceContactRadii[i]*ratio;profile=radii;
                proxy.Active=result.EntranceContactDepth>0;proxy.Depth=result.EntranceContactDepth*ratio;proxy.Centre=contactCentre.Value;proxy.Axis=axis;proxy.Side=(profileSide-axis*Vec3.Dot(profileSide,axis)).Normalized;for(int i=0;i<proxy.Radii.Length;i++)proxy.Radii[i]=result.EntranceProxyRadii[i]*ratio;
            }
            else proxy.Active=false;
            Constraint.Apply(native,physical,guide.Boundaries[0],axis,side,input.HasSource&&(result.EntryContact||result.SourceAdmitted),dt,input.Response,input.Recovery,input.MaxStrain,materialScale,profile,rotation,contactCentre,proxy);
            for(int j=0;j<data.Controls.Length;j++)offsets[data.Controls[j]]=Constraint.Offsets[j];
            LastMilliseconds=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;
        }
    }
}
