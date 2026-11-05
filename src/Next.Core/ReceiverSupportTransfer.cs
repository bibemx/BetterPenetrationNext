using System;
namespace BetterPenetrationNext.Core
{
 [Serializable]public sealed class ReceiverSupportTransfer
 {
  public int[] PhysicalIndices,Offsets,Indices;public double[] Weights;
  public void Apply(SoftBodySolver solver,Vec3[] output)
  {
   if(output==null||output.Length+1!=Offsets.Length)throw new ArgumentException("Support buffer differs from its cached mapping");
   for(int i=0;i<output.Length;i++){Vec3 value=new Vec3();for(int j=Offsets[i];j<Offsets[i+1];j++)value+=solver.Displacement(Indices[j])*Weights[j];output[i]=value;}
  }
 }
}
