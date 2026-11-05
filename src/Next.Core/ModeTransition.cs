using System;
namespace BetterPenetrationNext.Core
{
    // Mode changes happen only at zero physical displacement, then fade in.
    public sealed class ModeTransition
    {
        public int Current { get; private set; }
        public int Target { get; private set; }
        public double Blend { get; private set; }=1;
        public ModeTransition(int initial) { Current=Target=initial; }
        public void Request(int mode) { Target=mode; }
        public bool Advance(double dt,double seconds)
        {
            if(!Vec3.Finite(dt)||dt<0||!Vec3.Finite(seconds)||seconds<0) throw new ArgumentException("Invalid mode transition.");
            double step=seconds<=0?1:dt/seconds;
            if(Current!=Target) { Blend=Math.Max(0,Blend-step); if(Blend==0) { Current=Target; return true; } }
            else Blend=Math.Min(1,Blend+step);
            return false;
        }
    }
}
