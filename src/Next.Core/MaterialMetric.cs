using System;
namespace BetterPenetrationNext.Core
{
 // Average squared material lengths before blending rotated skin matrices.
 // Opposing bone rotations may fold an animated edge without shrinking its
 // available material length. Bone scale and authored morphs remain explicit.
 public struct MaterialMetric
 {
  public double XX,YY,ZZ,XY,XZ,YZ;
  public void Add(Linear3 frame,double weight){XX+=Vec3.Dot(frame.X,frame.X)*weight;YY+=Vec3.Dot(frame.Y,frame.Y)*weight;ZZ+=Vec3.Dot(frame.Z,frame.Z)*weight;XY+=Vec3.Dot(frame.X,frame.Y)*weight;XZ+=Vec3.Dot(frame.X,frame.Z)*weight;YZ+=Vec3.Dot(frame.Y,frame.Z)*weight;}
  public double SquaredLength(Vec3 v){return Math.Max(0,XX*v.X*v.X+YY*v.Y*v.Y+ZZ*v.Z*v.Z+2*(XY*v.X*v.Y+XZ*v.X*v.Z+YZ*v.Y*v.Z));}
 }
}
