using System;
namespace BetterPenetrationNext.Core
{
    public struct CoupledContactResult
    {
        public Vec3 ReceiverDelta,SourceDelta;
        public double UnresolvedPenetration;
        public bool ReceiverAtLimit;
    }
    // General local contact projection, independent of game rigs. The outer
    // solver supplies an updated normal and signed gap each iteration. This is
    // not by itself a complete rigid-body or multi-contact motion controller.
    public static class CoupledContactProjection
    {
        public static double MaximumAdvance(Vec3 current,Vec3 normal,Vec3 rest,double maximumOffset,Vec3[] neighbors,double[] restLengths,double maximumExtension)
        {
            if(!current.IsFinite||!rest.IsFinite||!normal.IsFinite||normal.LengthSquared<0.5||!Vec3.Finite(maximumOffset)||maximumOffset<0||!Vec3.Finite(maximumExtension)||maximumExtension<0||neighbors==null||restLengths==null||neighbors.Length!=restLengths.Length) throw new ArgumentException("Invalid local strain limit.");
            normal=normal.Normalized;
            double limit=RayBall(current-rest,normal,maximumOffset);
            for(int i=0;i<neighbors.Length;i++)
            {
                if(!neighbors[i].IsFinite||!Vec3.Finite(restLengths[i])||restLengths[i]<=0) throw new ArgumentException("Invalid rest edge.");
                limit=Math.Min(limit,RayBall(current-neighbors[i],normal,restLengths[i]*(1+maximumExtension)));
            }
            return limit;
        }
        static double RayBall(Vec3 offset,Vec3 direction,double radius)
        {
            double b=Vec3.Dot(offset,direction),c=offset.LengthSquared-radius*radius,discriminant=b*b-c;
            // A pre-existing strain violation requires a separate elastic/strain
            // projection. Do not silently increase that violation here.
            if(c>Math.Max(1e-15,radius*radius*1e-10)||discriminant<0) return 0;
            return Math.Max(0,-b+Math.Sqrt(Math.Max(0,discriminant)));
        }
        public static CoupledContactResult Project(double penetration,Vec3 normal,double receiverInverseMass,double sourceInverseMass,double receiverTravel,double sourceTravel,Vec3? sourceAxis=null)
        {
            if(!Vec3.Finite(penetration)||!normal.IsFinite||normal.LengthSquared<0.5||!Vec3.Finite(receiverInverseMass)||receiverInverseMass<0||!Vec3.Finite(sourceInverseMass)||sourceInverseMass<0||!Vec3.Finite(receiverTravel)||receiverTravel<0||!Vec3.Finite(sourceTravel)||sourceTravel<0) throw new ArgumentException("Invalid coupled contact.");
            var result=new CoupledContactResult(); if(penetration<=0) return result; normal=normal.Normalized;
            Vec3 projected=normal;
            if(sourceAxis.HasValue)
            { var axis=sourceAxis.Value; if(!axis.IsFinite||axis.LengthSquared<1e-20) throw new ArgumentException("Invalid motion axis."); axis=axis.Normalized; projected=axis*Vec3.Dot(axis,normal); }
            double effective=sourceInverseMass*Vec3.Dot(normal,projected),sum=receiverInverseMass+effective;
            if(sum<=1e-20) { result.UnresolvedPenetration=penetration; return result; }
            double receiver=receiverInverseMass==0?0:Math.Min(receiverTravel,penetration*receiverInverseMass/sum);
            double source=0,remaining=penetration-receiver,projection=Vec3.Dot(normal,projected);
            if(sourceInverseMass>0&&projection>1e-12)
            {
                var desired=-projected*(remaining/projection); result.SourceDelta=Vec3.ClampLength(desired,sourceTravel); source=-Vec3.Dot(result.SourceDelta,normal);
            }
            // If motion restrictions block the source but tissue still has legal
            // room, give the remainder to tissue before declaring infeasibility.
            if(receiverInverseMass>0) receiver=Math.Min(receiverTravel,Math.Max(receiver,penetration-source));
            result.ReceiverDelta=normal*receiver; result.ReceiverAtLimit=receiverInverseMass==0||receiver>=receiverTravel-1e-12;
            result.UnresolvedPenetration=Math.Max(0,penetration-receiver-source); return result;
        }
    }
}
