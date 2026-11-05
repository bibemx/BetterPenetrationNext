using BetterPenetrationNext.Core;
using Core_BetterPenetration;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed partial class NextRuntime
    {
        // Non-persistent diagnostic switch: compare the retained upstream body
        // planes with the reactive contact owner on exactly the same animation.
        public bool LegacySourceBodyConstraintsForAudit { get; set; }
        public bool LegacySourceSquashForAudit { get; set; }
        public bool LegacyEntranceDynamicsForAudit { get; set; }
        public int ReactiveEntranceOwnershipCalls { get; private set; }
        public int ReactiveSourceConstraintCalls { get; private set; }
        public int LegacySourceConstraintCalls { get; private set; }
        public int ReactiveSourceLengthCalls { get; private set; }
        public int ReactiveSourceGirthCalls { get; private set; }

        bool OwnsSourceContact(DanAgent source, CollisionAgent target)
        {
            if (!isActiveAndEnabled ||
                Mode == NextMode.Original || !UsesNativeEntrance || source == null || target == null)
                return false;
            Observation observation;
            PreparedBody body;
            if (!observations.TryGetValue(source, out observation) ||
                observation.TargetAgent != target || Time.unscaledTime - observation.LastSeen > 1 ||
                observation.Target == null ||
                !bodies.TryGetValue(observation.Target.GetInstanceID(), out body))
                return false;
            return body.AnatomicalConfigured && body.RigOwner == observation &&
                body.RigHasOutput && body.Binding.Alive && body.PoseVerified &&
                (body.RigWorker == null || body.RigWorker.Failure == null) &&
                GuideTargetMatches(source.m_referenceTarget, observation.Target.transform, true);
        }

        bool OwnsSourceShape(DanAgent source)
        {
            Observation observation;
            if(LegacySourceSquashForAudit||source==null||!observations.TryGetValue(source,out observation))return false;
            // Explicit manual sizing owns the uncompressed animation basis
            // already while loading/idle. Contact ownership stays separately
            // gated on a verified, bound receiver.
            bool manual=isActiveAndEnabled&&Mode!=NextMode.Original&&UsesNativeEntrance&&observation.Geometry.Alive&&
                (sourceLengthScale.Value!=1||sourceWidthScale.Value!=1||observation.SourceDimensions.MaximumOffset>1e-10);
            return manual||OwnsSourceContact(source,observation.TargetAgent);
        }

        bool OwnsReceiverEntrance(CollisionAgent receiver)
        {
            if(LegacyEntranceDynamicsForAudit || receiver==null || !isActiveAndEnabled || Mode==NextMode.Original || !UsesNativeEntrance)return false;
            foreach(var observation in observations.Values)
                if(observation.TargetAgent==receiver && OwnsSourceContact(observation.Agent,receiver))return true;
            foreach(var body in bodies.Values)
                if(body.RigOwner!=null&&body.RigOwner.TargetAgent==receiver&&body.RigHasOutput&&body.Binding.Alive&&body.RigOutput!=null&&!body.RigOutput.ReceiverActive&&body.RigOutput.RecoveryFraction<1)return true;
            return false;
        }

        static bool ReactiveEntrancePrefix(CollisionAgent receiver)
        {
            var runtime=Instance;
            if(runtime==null || !runtime.OwnsReceiverEntrance(receiver))return true;
            // These are upstream correction-only transforms. ReturnKokanBones
            // restores the same zero local translations at the end of release.
            // Do not integrate the old axial pull on top of the material loft.
            receiver.currentKokanPull=0;
            foreach(var bone in receiver.m_kokanPullBones)if(bone!=null)bone.localPosition=Vector3.zero;
            if(receiver.m_innerKokan!=null)receiver.m_innerKokan.localRotation=Quaternion.identity;
            runtime.ReactiveEntranceOwnershipCalls++;
            return false;
        }

        static bool EntranceReturnPrefix(CollisionAgent __instance)
        {return ReactiveEntrancePrefix(__instance);}

        static bool SourceLengthPrefix(DanAgent __instance, ref float __result)
        {
            var runtime=Instance;
            if(runtime==null || !runtime.OwnsSourceShape(__instance))return true;
            __result=__instance.m_baseDanLength;
            runtime.ReactiveSourceLengthCalls++;
            return false;
        }

        static bool SourceGirthPrefix(DanAgent __instance, ref float __result)
        {
            var runtime=Instance;
            if(runtime==null || !runtime.OwnsSourceShape(__instance))return true;
            __result=1;
            runtime.ReactiveSourceGirthCalls++;
            return false;
        }

        static bool SourceContactPrefix(DanAgent __instance, Vector3 danStart,
            Vector3 danTargetVector, float danLength, CollisionAgent targetAgent, ref Vector3 __result)
        {
            var runtime = Instance;
            if (runtime == null) return true;
            if (runtime.LegacySourceBodyConstraintsForAudit || !runtime.OwnsSourceContact(__instance, targetAgent))
            {
                runtime.LegacySourceConstraintCalls++;
                return true;
            }
            // Retain the animated root, direction and upstream length input.
            // R0 guidance and the actual cervical surface own contact, instead
            // of also projecting the same tip onto the old approximate planes
            // or pinning it to the upstream inner target.
            __result = danStart + danTargetVector * danLength;
            runtime.ReactiveSourceConstraintCalls++;
            return false;
        }
    }
}
