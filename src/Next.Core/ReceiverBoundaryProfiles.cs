using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Keep actual non-circular mouth and cervical attachment profiles. Replacing
    // these loops with a circle twists/shrinks the join even if the interior is smooth.
    public sealed class ReceiverBoundaryProfiles
    {
        readonly int[][] loops=new int[4][];readonly double[][] angles=new double[4][];
        public ReceiverBoundaryProfiles(ReactiveMaterialChart chart)
        {
            for(int layer=0;layer<2;layer++)for(int end=0;end<2;end++)
            {
                var indices=new List<int>();for(int i=0;i<chart.SupportAxial.Length;i++)if(chart.Regions[i]==layer+1&&Math.Abs(chart.SupportAxial[i]-end)<1e-9)indices.Add(i);
                if(layer==1&&end==0&&indices.Count==0){loops[2]=loops[0];angles[2]=angles[0];continue;}
                indices.Sort((a,b)=>chart.SupportAngle[a].CompareTo(chart.SupportAngle[b]));if(indices.Count<3)throw new ArgumentException("Receiver attachment profile missing");int k=layer*2+end;loops[k]=indices.ToArray();angles[k]=new double[indices.Count];for(int j=0;j<indices.Count;j++)angles[k][j]=chart.SupportAngle[indices[j]];
            }
        }
        public Vec3 Point(int region,int end,double angle,Vec3[] world)
        {
            int k=(region==2?2:0)+end;var row=loops[k];var theta=angles[k];angle-=Math.Floor(angle);int a=0,b=row.Length;
            while(a<b){int mid=(a+b)/2;if(theta[mid]<=angle)a=mid+1;else b=mid;}
            int lo=(a+row.Length-1)%row.Length,hi=a%row.Length;double start=theta[lo],finish=theta[hi];if(hi==0)finish+=1;if(angle<start)angle+=1;double t=(angle-start)/Math.Max(1e-12,finish-start);return world[row[lo]]*(1-t)+world[row[hi]]*t;
        }
    }
}
