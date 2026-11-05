using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed class AnatomicalPathGuide : MonoBehaviour
    {
        public Transform[] ControlPoints;
        public bool Spline=true;
        Vec3[] last;
        AnatomicalPathConstraintSystem path;
        bool lastSpline;
        public AnatomicalPathConstraintSystem Capture()
        {
            if(ControlPoints==null||ControlPoints.Length<2) return null;
            bool rebuild=last==null||last.Length!=ControlPoints.Length||lastSpline!=Spline;
            bool changed=rebuild;
            if(changed) last=new Vec3[ControlPoints.Length];
            for(int i=0;i<last.Length;i++) { if(ControlPoints[i]==null) return null; var p=AnimationAdapter.ToCore(ControlPoints[i].position); changed|=(last[i]-p).LengthSquared>1e-16; last[i]=p; }
            if(rebuild||path==null)path=new AnatomicalPathConstraintSystem(last,Spline);
            else if(changed)path.UpdateControls(last);
            lastSpline=Spline;
            return path;
        }
    }
}
