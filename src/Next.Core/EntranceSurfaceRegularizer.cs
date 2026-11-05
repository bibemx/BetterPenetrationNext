using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Regularizes the displacement field on the authored exterior, never the
    // surface coordinates themselves. Creases, thickness, skin identities and
    // asymmetry in the animated baseline therefore remain authored geometry.
    public sealed class EntranceSurfaceRegularizer
    {
        readonly SimulationDefinition mesh;
        readonly int[] active, offsets, neighbours,boundary;
        readonly double[] neighbourWeights, masses;
        readonly bool[] held,midline;
        readonly Vec3[] original, displacement, scratch;
        readonly Linear3[] frames,inverseFrames;
        readonly BilateralDisplacementConstraint bilateral;
        public int PairCount { get { return bilateral.PairCount; } }
        public int VertexCount { get { return active.Length; } }
        public double MaximumResidual { get { return bilateral.MaximumResidual; } }
        public bool Prepared { get; private set; }
        public int MidlineCount {get;private set;}
        public double MidlineResidual {get;private set;}

        public EntranceSurfaceRegularizer(SimulationDefinition mesh,int[] regions,double[] influence,bool[] attachments=null)
        {
            this.mesh=mesh;int count=mesh.Rest.Length;
            if(regions.Length!=count||influence.Length!=count||attachments!=null&&attachments.Length!=count)throw new ArgumentException("Exterior topology mismatch");
            var ids=new List<int>();var adjacency=new HashSet<int>[count];held=new bool[count];midline=new bool[count];masses=new double[count];
            double width=0;for(int i=0;i<count;i++)if(regions[i]==0)width=Math.Max(width,Math.Abs(mesh.Rest[i].X));
            double tolerance=Math.Max(1e-9,width*.0004);
            for(int i=0;i<count;i++)if(regions[i]==0){adjacency[i]=new HashSet<int>();ids.Add(i);held[i]=mesh.PinWeights[i]>=1||influence[i]>=.999999||attachments!=null&&attachments[i];masses[i]=held[i]?0:1;midline[i]=!held[i]&&influence[i]>0&&Math.Abs(mesh.Rest[i].X)<=tolerance;if(midline[i])MidlineCount++;}
            active=ids.ToArray();
            var boundaryIds=new HashSet<int>();
            for(int f=0;f<mesh.Triangles.Length;f+=3)for(int k=0;k<3;k++)
            {int a=mesh.Triangles[f+k],b=mesh.Triangles[f+(k+1)%3];if(a==b)continue;if(regions[a]==0&&(regions[b]==0||regions[b]==1)){adjacency[a].Add(b);if(regions[b]==1)boundaryIds.Add(b);}if(regions[b]==0&&(regions[a]==0||regions[a]==1)){adjacency[b].Add(a);if(regions[a]==1)boundaryIds.Add(a);}}
            boundary=new int[boundaryIds.Count];boundaryIds.CopyTo(boundary);
            offsets=new int[count+1];var nodes=new List<int>();var weights=new List<double>();
            for(int i=0;i<count;i++){offsets[i]=nodes.Count;if(adjacency[i]!=null)foreach(int j in adjacency[i]){nodes.Add(j);weights.Add(1/Math.Max(1e-7,(mesh.Rest[i]-mesh.Rest[j]).Length));}}offsets[count]=nodes.Count;neighbours=nodes.ToArray();neighbourWeights=weights.ToArray();
            // Pair only geometrically verified opposite material identities.
            // Each side retains its own complete skinning Jacobian; coincident
            // seam vertices are never collapsed into a shared control.
            var pairs=new List<BilateralPair>();var used=new bool[count];
            foreach(int a in active)
            {
                if(used[a]||midline[a]||mesh.Rest[a].X>=-1e-7||masses[a]==0||influence[a]<=0)continue;
                Vec3 p=mesh.Rest[a];p.X=-p.X;int best=-1;double error=Math.Pow(Math.Max(1e-5,width*.01),2);
                foreach(int b in active)if(!used[b]&&!midline[b]&&mesh.Rest[b].X>1e-7&&masses[b]>0&&influence[b]>0)
                {double d=(p-mesh.Rest[b]).LengthSquared;if(d<error&&MirroredWeights(a,b)){error=d;best=b;}}
                if(best>=0){used[a]=used[best]=true;pairs.Add(new BilateralPair(a,best,new Vec3(1,0,0)));}
            }
            bilateral=new BilateralDisplacementConstraint(count,pairs.ToArray());
            original=new Vec3[count];displacement=new Vec3[count];scratch=new Vec3[count];frames=new Linear3[count];inverseFrames=new Linear3[count];
        }
        bool MirroredWeights(int a,int b)
        {
            int start=mesh.InfluenceOffsets[a],end=mesh.InfluenceOffsets[a+1];double difference=0;
            for(int i=start;i<end;i++)
            {
                string name=mesh.BoneNames[mesh.BoneIndices[i]],mirror=name.Replace("_L","_#").Replace("_R","_L").Replace("_#","_R");double weight=0;
                for(int j=mesh.InfluenceOffsets[b];j<mesh.InfluenceOffsets[b+1];j++)if(mesh.BoneNames[mesh.BoneIndices[j]]==mirror){weight=mesh.BoneWeights[j];break;}
                difference+=Math.Abs(mesh.BoneWeights[i]-weight);
            }
            return difference<.05;
        }
        public void Apply(Vec3[] animated,Vec3[] positions,Linear3[] bones)
        {
            foreach(int i in boundary)displacement[i]=positions[i]-animated[i];
            foreach(int i in active){original[i]=displacement[i]=positions[i]-animated[i];Vec3 x=new Vec3(),y=new Vec3(),z=new Vec3();for(int j=mesh.InfluenceOffsets[i];j<mesh.InfluenceOffsets[i+1];j++){var b=bones[mesh.BoneIndices[j]];double w=mesh.BoneWeights[j];x+=b.X*w;y+=b.Y*w;z+=b.Z*w;}frames[i]=new Linear3(x,y,z);if(midline[i]&&!frames[i].TryInverse(out inverseFrames[i]))throw new InvalidOperationException("Singular entrance midline frame");}
            // Screened diffusion smooths transfer discontinuities without
            // erasing the requested opening. Fixed work and reused buffers.
            for(int pass=0;pass<8;pass++)
            {
                foreach(int i in active){if(held[i]){scratch[i]=displacement[i];continue;}Vec3 sum=new Vec3();double weight=0;for(int j=offsets[i];j<offsets[i+1];j++){sum+=displacement[neighbours[j]]*neighbourWeights[j];weight+=neighbourWeights[j];}scratch[i]=weight>0?original[i]*.35+sum*(.65/weight):original[i];}
                foreach(int i in active){displacement[i]=scratch[i];if(midline[i]){Vec3 local=inverseFrames[i].Multiply(displacement[i]);local.X=0;displacement[i]=frames[i].Multiply(local);}}
            }
            foreach(int i in active)positions[i]=animated[i]+displacement[i];
            // Spatial relaxation, independent of render dt; this is not a
            // claimed tissue material coefficient. Contact/strain runs after it.
            bilateral.BeginStep(animated,frames,masses,1,0);bilateral.Project(positions);Prepared=true;
        }
        public void Reset(){Prepared=false;MidlineResidual=0;bilateral.Reset();}
        public void Measure(Vec3[] positions,Vec3[] animated){if(!Prepared)return;bilateral.MeasureResidual(positions);MidlineResidual=0;foreach(int i in active)if(midline[i])MidlineResidual=Math.Max(MidlineResidual,Math.Abs(inverseFrames[i].Multiply(positions[i]-animated[i]).X));}
    }
}
