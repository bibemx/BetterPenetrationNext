using System;

namespace BetterPenetrationNext.Core
{
    public static class CervicalFollowConstraint
    {
        public static double TravelWithinReach(Vec3 joint,Vec3 root,Vec3 axis,double length,double maximum)
        {
            if(!joint.IsFinite||!root.IsFinite||!axis.IsFinite||!Vec3.Finite(length)||!Vec3.Finite(maximum)||length<0||maximum<0)throw new ArgumentException("Invalid joint reach");
            Vec3 d=joint-root;axis=axis.Normalized;if(axis.LengthSquared<.5||d.LengthSquared>length*length+1e-12)return 0;
            double b=Vec3.Dot(d,axis),disc=b*b-d.LengthSquared+length*length;
            return Math.Max(0,Math.Min(maximum,-b+Math.Sqrt(Math.Max(0,disc))));
        }
        // Unilateral compression only. A withdrawing tip never creates a tensile
        // attachment. The complete organ group receives one rigid translation.
        public static Vec3 Target(double penetration,Vec3 outward,double mobility,double maximumTravel)
        {
            if(!Vec3.Finite(penetration)||!outward.IsFinite||!Vec3.Finite(mobility)||!Vec3.Finite(maximumTravel)||maximumTravel<0)throw new ArgumentException("Invalid cervical follow constraint");
            return -outward.Normalized*Math.Min(maximumTravel,Math.Max(0,penetration)*Math.Max(0,Math.Min(1,mobility)));
        }
        public static Vec3 AlongAxis(double penetration,Vec3 outward,Vec3 axis,double mobility,double maximumTravel,Vec3 current)
        {
            if(!Vec3.Finite(penetration)||!outward.IsFinite||!axis.IsFinite||!current.IsFinite||!Vec3.Finite(mobility)||!Vec3.Finite(maximumTravel)||maximumTravel<0)throw new ArgumentException("Invalid cervical depth constraint");
            axis=axis.Normalized;double position=Math.Max(0,Math.Min(maximumTravel,Vec3.Dot(current,axis))),closing=-Vec3.Dot(outward.Normalized,axis);
            // Jacobian of separation with respect to the anatomical joint DOF.
            // A side contact cannot invent a downward/lateral joint translation.
            if(closing>1e-5)position=Math.Min(maximumTravel,position+Math.Max(0,penetration)*Math.Max(0,Math.Min(1,mobility))/closing);
            return axis*position;
        }
    }
}
