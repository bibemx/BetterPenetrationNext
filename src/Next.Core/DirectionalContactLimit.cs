using System;
using System.Diagnostics;

namespace BetterPenetrationNext.Core
{
    // A bounded translational sweep against explicitly selected obstacle faces.
    // The source shape is fixed during one query; no caps or radii are invented.
    // This resolves an initially deep intersection whose local radial gradient
    // cannot withdraw a source constrained to travel along its guide.
    public sealed class DirectionalContactLimit
    {
        struct Interval { public double Low,High; }
        readonly TriangleSurface sourceSurface;
        readonly int[] receiverFaces,candidates;
        readonly Interval[] intervals;
        readonly int originalSourceFaces;
        int count;
        public SourceContactBody Source;
        public double Clearance=.0002;
        public long PairTests,BvhCandidates,BoundsTests;
        public double Milliseconds,LastRetreat;
        public bool Infeasible,Enabled=true;
        public DirectionalContactLimit(TriangleSurface source,int sourceFaceCount,int[] receiverTriangles,int maximumIntervals=256)
        {
            if(source==null||sourceFaceCount<1||sourceFaceCount>source.TriangleCount||receiverTriangles==null||receiverTriangles.Length%3!=0||maximumIntervals<1)throw new ArgumentException("Invalid directional contact geometry");
            sourceSurface=source;originalSourceFaces=sourceFaceCount;receiverFaces=(int[])receiverTriangles.Clone();candidates=new int[source.TriangleCount];intervals=new Interval[maximumIntervals];
        }
        public void BeginStep(){PairTests=BvhCandidates=BoundsTests=0;Milliseconds=LastRetreat=0;Infeasible=false;}
        static double Min(double a,double b,double c){return Math.Min(a,Math.Min(b,c));}
        static double Max(double a,double b,double c){return Math.Max(a,Math.Max(b,c));}
        static bool Slab(double s0,double s1,double r0,double r1,double speed,double margin,ref double low,ref double high)
        {
            double lower=r0-s1-margin,upper=r1-s0+margin;
            if(Math.Abs(speed)<1e-14)return lower<=0&&upper>=0;
            double x=lower/speed,y=upper/speed;if(x>y){double swap=x;x=y;y=swap;}
            low=Math.Max(low,x);high=Math.Min(high,y);return low<=high;
        }
        static bool Axis(Vec3 n,Vec3 a,Vec3 b,Vec3 c,Vec3 p,Vec3 q,Vec3 r,Vec3 motion,double margin,ref double low,ref double high)
        {
            double length=n.Length;if(length<1e-20)return true;n/=length;
            double sa=Vec3.Dot(a,n),sb=Vec3.Dot(b,n),sc=Vec3.Dot(c,n),ra=Vec3.Dot(p,n),rb=Vec3.Dot(q,n),rc=Vec3.Dot(r,n),speed=Vec3.Dot(motion,n);
            double lower=Min(ra,rb,rc)-Max(sa,sb,sc)-margin,upper=Max(ra,rb,rc)-Min(sa,sb,sc)+margin;
            if(Math.Abs(speed)<1e-14)return lower<=0&&upper>=0;
            double x=lower/speed,y=upper/speed;if(x>y){double swap=x;x=y;y=swap;}
            low=Math.Max(low,x);high=Math.Min(high,y);return low<=high;
        }
        static bool Sweep(Vec3 a,Vec3 b,Vec3 c,Vec3 p,Vec3 q,Vec3 r,Vec3 motion,double margin,ref double low,ref double high)
        {
            Vec3 e0=b-a,e1=c-b,e2=a-c,f0=q-p,f1=r-q,f2=p-r,n0=Vec3.Cross(e0,e1),n1=Vec3.Cross(f0,f1);
            if(n0.LengthSquared<1e-36||n1.LengthSquared<1e-36)return false;
            if(!Axis(n0,a,b,c,p,q,r,motion,margin,ref low,ref high)||!Axis(n1,a,b,c,p,q,r,motion,margin,ref low,ref high))return false;
            for(int i=0;i<3;i++)
            {
                Vec3 e=i==0?e0:i==1?e1:e2,f=i==0?f0:i==1?f1:f2;
                // In-plane axes also handle coplanar triangles with overlapping AABBs.
                if(!Axis(Vec3.Cross(n0,e),a,b,c,p,q,r,motion,margin,ref low,ref high)||!Axis(Vec3.Cross(n1,f),a,b,c,p,q,r,motion,margin,ref low,ref high))return false;
                for(int j=0;j<3;j++)if(!Axis(Vec3.Cross(e,j==0?f0:j==1?f1:f2),a,b,c,p,q,r,motion,margin,ref low,ref high))return false;
            }
            return true;
        }
        void Add(double low,double high)
        {
            int first=0;while(first<count&&intervals[first].High<low-1e-10)first++;
            int end=first;while(end<count&&intervals[end].Low<=high+1e-10){low=Math.Min(low,intervals[end].Low);high=Math.Max(high,intervals[end].High);end++;}
            int next=count-(end-first)+1;
            if(next>intervals.Length)throw new InvalidOperationException("Directional interval budget exceeded; no collision intervals were silently dropped");
            Array.Copy(intervals,end,intervals,first+1,count-end);intervals[first]=new Interval{Low=low,High=high};count=next;
        }
        Vec3 Local(Vec3 p){return Source.Pivot+Source.Rotate(p-Source.Pivot-Source.Offset,true);}
        public bool Project(Vec3[] receiver)
        {
            if(!Enabled||Source==null||Source.InverseMass<=0||Source.TranslationAxis.LengthSquared<1e-20)return false;
            long started=Stopwatch.GetTimestamp();count=0;Vec3 axis=Source.TranslationAxis.Normalized,localAxis=Source.Rotate(axis,true);
            double back=Source.Travel(-axis),forward=Math.Min(Source.Travel(axis),Math.Max(0,-Vec3.Dot(Source.Offset,axis)));
            if(back<1e-12&&forward<1e-12)return false;
            for(int t=0;t<receiverFaces.Length;t+=3)
            {
                Vec3 p=Local(receiver[receiverFaces[t]]),q=Local(receiver[receiverFaces[t+1]]),r=Local(receiver[receiverFaces[t+2]]);
                var receiverBounds=Bounds3.Empty;receiverBounds.Include(p);receiverBounds.Include(q);receiverBounds.Include(r);
                var bounds=Bounds3.Empty;
                bounds.Include(p+localAxis*back);bounds.Include(q+localAxis*back);bounds.Include(r+localAxis*back);
                double queryEnd=count==0?forward:Math.Min(forward,intervals[0].Low);
                bounds.Include(p-localAxis*queryEnd);bounds.Include(q-localAxis*queryEnd);bounds.Include(r-localAxis*queryEnd);
                Vec3 pad=new Vec3(Clearance,Clearance,Clearance);bounds=new Bounds3(bounds.Min-pad,bounds.Max+pad);
                int n=sourceSurface.CollectTriangles(bounds,candidates);BvhCandidates+=n;
                for(int j=0;j<n;j++)
                {
                    if(candidates[j]>=originalSourceFaces)continue;
                    BoundsTests++;Vec3 a,b,c;sourceSurface.GetTriangle(candidates[j],out a,out b,out c);
                    double low=-back,high=count==0?forward:Math.Min(forward,intervals[0].Low);
                    if(!Slab(Min(a.X,b.X,c.X),Max(a.X,b.X,c.X),receiverBounds.Min.X,receiverBounds.Max.X,localAxis.X,Clearance,ref low,ref high)||
                       !Slab(Min(a.Y,b.Y,c.Y),Max(a.Y,b.Y,c.Y),receiverBounds.Min.Y,receiverBounds.Max.Y,localAxis.Y,Clearance,ref low,ref high)||
                       !Slab(Min(a.Z,b.Z,c.Z),Max(a.Z,b.Z,c.Z),receiverBounds.Min.Z,receiverBounds.Max.Z,localAxis.Z,Clearance,ref low,ref high))continue;
                    PairTests++;if(Sweep(a,b,c,p,q,r,localAxis,Clearance,ref low,ref high))Add(low,high);
                }
            }
            bool moved=false;
            if(count>0)
            {
                // The negative end is the prescribed entry side. Select its
                // connected free interval; do not jump through a terminal wall.
                double boundary=intervals[0].Low-1e-7;
                Infeasible=boundary< -back;
                if(boundary<0){Vec3 before=Source.Offset;Source.Correct(axis*Math.Max(-back,boundary));LastRetreat=(Source.Offset-before).Length;moved=LastRetreat>1e-12;Source.MarkContact();}
                else if(boundary<Clearance)Source.MarkContact();
            }
            Milliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;return moved;
        }
    }
}
