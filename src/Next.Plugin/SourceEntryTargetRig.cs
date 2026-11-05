using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    [DefaultExecutionOrder(-32000)]
    internal sealed class SourceEntryPoseRestore : MonoBehaviour
    {void Update(){SourceEntryTargetRig.RestoreAll();}}

    internal sealed class SourceEntryTargetRig : IDisposable
    {
        static readonly List<SourceEntryTargetRig> owned=new List<SourceEntryTargetRig>();
        readonly EntryTargetGuidance guidance=new EntryTargetGuidance();Transform root;Quaternion native,applied;bool hasPose;SourceSectionRig shape;
        public void Configure(SourceSectionRig value){if(shape==null)shape=value;}
        public bool Configured {get{return shape!=null;}}
        public double EntryAxisError {get;private set;}
        public double EntryLateralError {get;private set;}
        public double RequestedAngle {get{return guidance.RequestedAngle;}}
        public double AppliedAngle {get{return guidance.RotationVector.Length;}}
        public double MaximumDisplacement {get;private set;}
        public double RootTranslationError {get;private set;}
        public Vec3 Target {get;private set;}
        public bool AppliedThisFrame {get;private set;}
        public SourceEntryTargetRig(){owned.Add(this);}
        public static void RestoreAll(){foreach(var item in owned)item.Restore();}
        void Restore()
        {
            if(hasPose&&root!=null&&Quaternion.Angle(root.localRotation,applied)<.001f)root.localRotation=native;
            hasPose=false;AppliedThisFrame=false;
        }
        public bool Aim(Transform joint,Transform next,Vec3[] animated,OrientedEntryFrame entry,double dt,double maximumAngle,double angularSpeed,double response,double displacementBudget)
        {
            Restore();root=joint;Target=entry.Centre;MaximumDisplacement=RootTranslationError=0;
            if(joint==null||next==null||!next.IsChildOf(joint))return false;
            Vector3 pivot=joint.position;shape.CaptureAnimation(animated,AnimationAdapter.ToCore(pivot));
            Vector3 forward=AnimationAdapter.ToUnity(shape.AnimatedCentre(shape.SectionCount-1))-pivot,direction=AnimationAdapter.ToUnity(entry.DesiredDirection(AnimationAdapter.ToCore(pivot)));
            if(direction.sqrMagnitude<1e-8||Vector3.Dot(forward,direction)<=0){guidance.Reset();return false;}
            double radius=0;foreach(var p in animated)radius=Math.Max(radius,(p-AnimationAdapter.ToCore(pivot)).Length);
            double angle=Math.Min(Math.Max(0,maximumAngle),2*Math.Asin(Math.Min(1,Math.Max(0,displacementBudget)/Math.Max(1e-9,2*radius))));
            var parent=joint.parent;Vector3 f=parent==null?forward:parent.InverseTransformDirection(forward),d=parent==null?direction:parent.InverseTransformDirection(direction);
            guidance.Advance(AnimationAdapter.ToCore(f),AnimationAdapter.ToCore(d),dt,angle,angularSpeed,response);native=joint.localRotation;
            Vec3 rotation=guidance.RotationVector;double length=rotation.Length;if(length<1e-9)return false;
            applied=Quaternion.AngleAxis((float)(length*Mathf.Rad2Deg),AnimationAdapter.ToUnity(rotation/length))*native;joint.localRotation=applied;hasPose=AppliedThisFrame=true;
            MaximumDisplacement=2*radius*Math.Sin(length*.5);RootTranslationError=Vector3.Distance(pivot,joint.position);
            Vec3 worldForward=AnimationAdapter.ToCore(parent==null?AnimationAdapter.ToUnity(EntryTargetGuidance.Rotate(AnimationAdapter.ToCore(f),rotation)):parent.TransformDirection(AnimationAdapter.ToUnity(EntryTargetGuidance.Rotate(AnimationAdapter.ToCore(f),rotation)))).Normalized;
            EntryAxisError=Math.Acos(Math.Max(-1,Math.Min(1,Vec3.Dot(worldForward,entry.Normal))));
            double den=Vec3.Dot(worldForward,entry.Normal);Vec3 hit=AnimationAdapter.ToCore(pivot)+worldForward*(Vec3.Dot(entry.Centre-AnimationAdapter.ToCore(pivot),entry.Normal)/Math.Max(1e-8,den));
            EntryLateralError=(hit-entry.Centre).Length;return true;
        }
        public void Reset(){Restore();guidance.Reset();}
        public void Dispose(){Reset();owned.Remove(this);}
    }
}
