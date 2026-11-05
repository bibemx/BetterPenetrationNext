using BetterPenetrationNext;
using Core_BetterPenetration;

namespace KKS_BetterPenetration
{
    public partial class KKS_BetterPenetration
    {
        UnityEngine.SkinWeights previousSkinWeights;
        void OnDestroy()
        {
            if(NextRuntime.Instance!=null) UnityEngine.Object.Destroy(NextRuntime.Instance);
            if(harmony!=null) { harmony.UnpatchSelf(); harmony=null; }
            CoreGame.OnEndScene();
            if(UnityEngine.QualitySettings.skinWeights==UnityEngine.SkinWeights.Unlimited) UnityEngine.QualitySettings.skinWeights=previousSkinWeights;
            if(instance==this) instance=null;
        }
    }
}
