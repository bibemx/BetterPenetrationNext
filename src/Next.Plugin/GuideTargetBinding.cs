using Core_BetterPenetration;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed partial class NextRuntime
    {
        internal static bool GuideTargetMatches(Transform target,Transform receiver,bool required)
        {
            if(!required)return true;
            return target!=null&&receiver!=null&&target.IsChildOf(receiver)&&
                (target.name==LookTargets.KokanTarget||target.name==LookTargets.BPKokanTarget);
        }
    }
}
