using System;
using System.Collections.Generic;
namespace BetterPenetrationNext.Core
{
    // Smooth the correction field, not the authored tissue surface. Coincident
    // skin identities are supplied explicitly by the caller; no spatial weld.
    public sealed class SurfaceOffsetFairing
    {
        readonly int[] identity,starts,neighbours;readonly double[] weights,sums,mobility;readonly Vec3[] scratch;
        public SurfaceOffsetFairing(Vec3[] rest,EdgeConstraint[] edges,int[] identity,double[] mobility)
        {
            int n=rest.Length;if(identity.Length!=n||mobility.Length!=n)throw new ArgumentException("Fairing topology differs");this.identity=(int[])identity.Clone();this.mobility=(double[])mobility.Clone();starts=new int[n+1];sums=new double[n];scratch=new Vec3[n];
            var unique=new HashSet<long>();var links=new List<EdgeConstraint>();
            foreach(var edge in edges){int a=identity[edge.A],b=identity[edge.B];if(a>b){int t=a;a=b;b=t;}if(a==b||!unique.Add(((long)a<<32)|(uint)b))continue;links.Add(new EdgeConstraint(a,b));starts[a+1]++;starts[b+1]++;}
            for(int i=0;i<n;i++)starts[i+1]+=starts[i];neighbours=new int[starts[n]];weights=new double[neighbours.Length];var cursor=(int[])starts.Clone();
            foreach(var edge in links){int a=edge.A,b=edge.B;double w=1/Math.Max(1e-8,(rest[a]-rest[b]).Length);int x=cursor[a]++,y=cursor[b]++;neighbours[x]=b;neighbours[y]=a;weights[x]=weights[y]=w;sums[a]+=w;sums[b]+=w;}
        }
        public void Apply(Vec3[] offsets,int sweeps,double relaxation)
        {
            if(offsets.Length!=scratch.Length||sweeps<0||sweeps>16||!Vec3.Finite(relaxation)||relaxation<0||relaxation>1)throw new ArgumentException("Invalid surface fairing step");
            for(int pass=0;pass<sweeps;pass++)
            {
                for(int i=0;i<offsets.Length;i++)
                {
                    scratch[i]=offsets[i];if(identity[i]!=i||mobility[i]<=0||sums[i]<=0)continue;Vec3 average=new Vec3();for(int j=starts[i];j<starts[i+1];j++)average+=offsets[neighbours[j]]*weights[j];scratch[i]+=(average/sums[i]-offsets[i])*relaxation;
                }
                for(int i=0;i<offsets.Length;i++)offsets[i]=scratch[identity[i]];
            }
        }
    }
}
