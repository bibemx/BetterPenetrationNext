using System;
namespace BetterPenetrationNext.Core
{
    public static class ContactPrecision
    {
        // Resolution of the float Unity pose plus a small fraction of the
        // object scale. This is a numerical stopping criterion, not a material
        // parameter. Raw residuals remain visible in diagnostics.
        public static double Tolerance(Vec3 world,double size)
        {
            double magnitude=Math.Max(Math.Abs(world.X),Math.Max(Math.Abs(world.Y),Math.Abs(world.Z))),ulp=0;
            if(magnitude>0)ulp=Math.Pow(2,Math.Floor(Math.Log(magnitude)/Math.Log(2))-23);
            return Math.Max(1e-6,Math.Max(Math.Max(0,size)*1e-4,ulp*4));
        }
    }
}
