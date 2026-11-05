using System.Collections.Generic;
using Core_BetterPenetration;
using UnityEngine;

namespace BetterPenetrationNext
{
    // Never bypasses source length/aim/IK constraints. Only explicitly prepared
    // targets can relinquish an audited surface-deformation controller.
    internal static class BoneConstraintController
    {
        sealed class Saved { public bool Enabled; public ChaControl Character; }
        static readonly Dictionary<DynamicBone,Saved> owned=new Dictionary<DynamicBone,Saved>();
        static readonly List<DynamicBone> release=new List<DynamicBone>();
        public static bool SuppressLegacySurface(ChaControl character)
        { return NextRuntime.Instance!=null&&NextRuntime.Instance.Mode==NextMode.SoftBody&&!NextRuntime.Instance.UsesNativeEntrance&&NextRuntime.Instance.PhysicsReady(character); }
        public static void UpdateDynamicBones(CollisionAgent agent)
        {
            if(agent==null||!SuppressLegacySurface(agent.m_collisionCharacter)) return;
            // Use agent-owned explicit references. No bone is removed or selected
            // solely by a substring in its name.
            foreach(var bone in agent.m_kokanDynamicBones) Suspend(bone,agent.m_collisionCharacter);
            Suspend(agent.m_anaDynamicBones,agent.m_collisionCharacter); Suspend(agent.m_bellyDynamicBone,agent.m_collisionCharacter);
        }
        static void Suspend(DynamicBone bone,ChaControl character) { if(bone==null) return; if(!owned.ContainsKey(bone)) owned.Add(bone,new Saved { Enabled=bone.enabled,Character=character }); bone.enabled=false; }
        public static void RestoreUnready()
        {
            release.Clear(); foreach(var item in owned) if(item.Key==null||!SuppressLegacySurface(item.Value.Character)) release.Add(item.Key);
            foreach(var bone in release) { var saved=owned[bone]; if(bone!=null) { bone.enabled=saved.Enabled; bone.ResetParticlesPosition(); } owned.Remove(bone); }
        }
        public static void Restore()
        { foreach(var item in owned) if(item.Key!=null) { item.Key.enabled=item.Value.Enabled; item.Key.ResetParticlesPosition(); } owned.Clear(); }
    }
}
