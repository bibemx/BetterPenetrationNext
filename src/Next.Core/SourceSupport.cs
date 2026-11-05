using System;
namespace BetterPenetrationNext.Core
{
 public static class SourceSupport
 {
  // Bone endpoints can sit well behind the leading geometry. Use the actual
  // selected animated surface support along its authored root-to-tip axis.
  public static Vec3 LeadingPoint(Vec3[] animated,Vec3 root,Vec3 boneTip,int[] selectedVertices=null){
   Vec3 axis=(boneTip-root).Normalized;if(axis.LengthSquared<.5)throw new ArgumentException("Collapsed source axis");double maximum=double.NegativeInfinity;
   int count=selectedVertices==null?animated.Length:selectedVertices.Length;
   for(int j=0;j<count;j++)maximum=Math.Max(maximum,Vec3.Dot(animated[selectedVertices==null?j:selectedVertices[j]]-root,axis));
   if(!Vec3.Finite(maximum)||maximum<1e-8)throw new ArgumentException("No leading source surface");return root+axis*maximum;
  }
 }
}
