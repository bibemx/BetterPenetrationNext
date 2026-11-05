using System;
namespace BetterPenetrationNext.Core
{
    public static class SmoothTerminalApproach
    {
        // Compact C2 stop: identical to free axial motion outside the approach
        // band, exactly at the real terminal after compression of that band.
        // Withdrawal traverses the same function; no time latch or delayed pose.
        public static double Distance(double free,double terminal,double band)
        {
            if(band<=1e-12)return Math.Min(free,terminal);
            if(free<=terminal-band)return free;if(free>=terminal+band)return terminal;
            double t=(free-terminal+band)/(2*band),t2=t*t;
            return terminal-band+2*band*(t-t*t2+.5*t2*t2);
        }
    }
}
