using System;

namespace BetterPenetrationNext.Core
{
    // Recovery belongs to the GLOBAL inactive phase, never to isolated sections
    // losing contact while an object is still admitted to the canal.
    public sealed class ReceiverContactPhase
    {
        public bool Active {get;private set;}
        public bool BeganRelease {get;private set;}
        public double ReleasedSeconds {get;private set;}
        public void Advance(bool active,double dt)
        {if(!Vec3.Finite(dt)||dt<0)throw new ArgumentException("Invalid receiver phase interval");BeganRelease=Active&&!active;Active=active;ReleasedSeconds=active?0:BeganRelease?0:ReleasedSeconds+dt;}
        public double Weight(double recovery)
        {if(Active)return 1;if(recovery<=0)return 0;double t=Math.Max(0,Math.Min(1,ReleasedSeconds/recovery));return 1-t*t*(3-2*t);}
    }
}
