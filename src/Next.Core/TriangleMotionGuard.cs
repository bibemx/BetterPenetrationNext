using System;
using System.Diagnostics;

namespace BetterPenetrationNext.Core
{
    // Continuous first-contact events for a single vertex displacement. The
    // incident faces each have one moving vertex; all other faces are fixed
    // during this coordinate update. Vertex/face and edge/edge events therefore
    // have linear coplanarity equations. Genuine rims remain open.
    public sealed class TriangleMotionGuard
    {
        readonly int[] faces,offsets,incident,candidates;
        readonly TriangleSurface tree;
        readonly Vec3[] positions;
        readonly int[] cachedFace,cachedOther;
        public long CachedBlockedMoves;
        public long PairTests,BoundsQueries,LimitedMoves;
        public long PlaneCertificates;
        public static bool UsePlaneCertificates=true;
        public double Milliseconds;
        public double Clearance=1e-6;
        public long PairCorrections;
        public double MaximumRequestedGap,ReactionMilliseconds;
        int lastFace=-1,lastOther=-1,lastVertex=-1;
        double lastTime;
        Vec3 lastEnd;
        public TriangleMotionGuard(Vec3[] positions,int[] triangles)
        {
            this.positions=positions;faces=(int[])triangles.Clone();offsets=new int[positions.Length+1];
            cachedFace=new int[positions.Length];cachedOther=new int[positions.Length];for(int i=0;i<positions.Length;i++)cachedFace[i]=cachedOther[i]=-1;
            foreach(int i in faces)offsets[i+1]++;
            for(int i=1;i<offsets.Length;i++)offsets[i]+=offsets[i-1];
            incident=new int[faces.Length];var cursor=(int[])offsets.Clone();
            for(int i=0;i<faces.Length;i++)incident[cursor[faces[i]]++]=i/3;
            candidates=new int[faces.Length/3];tree=new TriangleSurface(positions,faces,SurfaceKind.OrientedShell);tree.InitializeIncrementalRefit();
        }
        public void BeginStep(){PairTests=BoundsQueries=LimitedMoves=PlaneCertificates=PairCorrections=CachedBlockedMoves=0;Milliseconds=MaximumRequestedGap=ReactionMilliseconds=0;tree.Refit(positions);}
        public void Commit(int vertex){tree.RefitVertices(positions,vertex,vertex,vertex);}
        bool Adjacent(int a,int b)
        {
            for(int i=0;i<3;i++)for(int j=0;j<3;j++)if(faces[a*3+i]==faces[b*3+j])return true;
            return false;
        }
        static bool InTriangle(Vec3 p,Vec3 a,Vec3 b,Vec3 c)
        {
            Vec3 ab=b-a,ac=c-a,ap=p-a;double x=Vec3.Dot(ab,ab),y=Vec3.Dot(ab,ac),z=Vec3.Dot(ac,ac),den=x*z-y*y;
            if(den<1e-36)return false;
            double v=(z*Vec3.Dot(ap,ab)-y*Vec3.Dot(ap,ac))/den,w=(x*Vec3.Dot(ap,ac)-y*Vec3.Dot(ap,ab))/den;
            return v>=-1e-8&&w>=-1e-8&&v+w<=1+1e-8;
        }
        static double Volume(Vec3 a,Vec3 b,Vec3 c,Vec3 d){return Vec3.Dot(Vec3.Cross(b-a,c-a),d-a);}
        static bool Root(double a,double b,out double t)
        {
            double delta=b-a;t=0;if(Math.Abs(delta)<=Math.Max(1e-28,Math.Max(Math.Abs(a),Math.Abs(b))*1e-12))return false;
            t=-a/delta;return t>=0&&t<=1;
        }
        static bool SegmentsMeet(Vec3 a,Vec3 b,Vec3 c,Vec3 d)
        {
            Vec3 u=b-a,v=d-c,w=a-c;double aa=Vec3.Dot(u,u),bb=Vec3.Dot(u,v),cc=Vec3.Dot(v,v),dd=Vec3.Dot(u,w),ee=Vec3.Dot(v,w),den=aa*cc-bb*bb;
            double scale=Math.Max(aa,cc),epsilon=Math.Max(1e-24,scale*1e-14);
            if(den>aa*cc*1e-12)
            {
                double s=(bb*ee-cc*dd)/den,t=(aa*ee-bb*dd)/den;
                return s>=-1e-8&&s<=1+1e-8&&t>=-1e-8&&t<=1+1e-8&&(w+u*s-v*t).LengthSquared<=epsilon;
            }
            if(aa<1e-30||cc<1e-30)return false;
            if(Vec3.Cross(w,u).LengthSquared>aa*epsilon)return false;
            double t0=Vec3.Dot(c-a,u)/aa,t1=Vec3.Dot(d-a,u)/aa;
            return Math.Max(t0,t1)>=-1e-8&&Math.Min(t0,t1)<=1+1e-8;
        }
        struct ContactTime {public double First;public bool Initial;}
        static void Accept(double t,ref ContactTime first){if(t<=1e-9)first.Initial=true;else if(t<first.First)first.First=t;}
        static void EdgeEvents(Vec3 p,Vec3 end,Vec3 q,Vec3 a,Vec3 b,Vec3 normal,ref ContactTime first)
        {
            double t;
            if(Root(Volume(p,q,a,b),Volume(end,q,a,b),out t))
            {
                if(SegmentsMeet(p+(end-p)*t,q,a,b))Accept(t,ref first);
                return;
            }
            // Coplanar motion needs in-plane endpoint/edge events as well.
            double n=normal.Length;if(n<1e-24)return;normal/=n;
            if(Math.Abs(Volume(p,q,a,b))>1e-18)return;
            if(Root(Vec3.Dot(normal,Vec3.Cross(q-p,a-p)),Vec3.Dot(normal,Vec3.Cross(q-end,a-end)),out t)&&SegmentsMeet(p+(end-p)*t,q,a,b))Accept(t,ref first);
            if(Root(Vec3.Dot(normal,Vec3.Cross(q-p,b-p)),Vec3.Dot(normal,Vec3.Cross(q-end,b-end)),out t)&&SegmentsMeet(p+(end-p)*t,q,a,b))Accept(t,ref first);
            if(Root(Vec3.Dot(normal,Vec3.Cross(b-a,p-a)),Vec3.Dot(normal,Vec3.Cross(b-a,end-a)),out t)&&SegmentsMeet(p+(end-p)*t,q,a,b))Accept(t,ref first);
        }
        static void VertexEvent(Vec3 p,Vec3 end,Vec3 q,Vec3 r,Vec3 vertex,ref ContactTime first)
        {
            double t;if(Root(Volume(p,q,r,vertex),Volume(end,q,r,vertex),out t)&&InTriangle(vertex,p+(end-p)*t,q,r))Accept(t,ref first);
        }
        static bool SegmentCross(Vec3 p,Vec3 q,Vec3 a,Vec3 b,Vec3 c)
        {
            Vec3 ab=b-a,ac=c-a,d=q-p,h=Vec3.Cross(d,ac);double det=Vec3.Dot(ab,h);if(Math.Abs(det)<1e-28)return false;
            Vec3 s=p-a;double u=Vec3.Dot(s,h)/det;if(u<=1e-9||u>=1-1e-9)return false;
            Vec3 x=Vec3.Cross(s,ab);double v=Vec3.Dot(d,x)/det,t=Vec3.Dot(ac,x)/det;
            return v>1e-9&&u+v<1-1e-9&&t>1e-9&&t<1-1e-9;
        }
        static bool StrictInside(Vec3 p,Vec3 a,Vec3 b,Vec3 c)
        {
            Vec3 ab=b-a,ac=c-a,ap=p-a;double x=Vec3.Dot(ab,ab),y=Vec3.Dot(ab,ac),z=Vec3.Dot(ac,ac),den=x*z-y*y;
            if(den<1e-36)return false;double v=(z*Vec3.Dot(ap,ab)-y*Vec3.Dot(ap,ac))/den,w=(x*Vec3.Dot(ap,ac)-y*Vec3.Dot(ap,ab))/den;
            return v>1e-8&&w>1e-8&&v+w<1-1e-8;
        }
        static bool CrossAtStart(Vec3 p,Vec3 end,Vec3 q,Vec3 r,Vec3 a,Vec3 b,Vec3 c)
        {
            p+=(end-p)*1e-5;
            if(SegmentCross(p,q,a,b,c)||SegmentCross(q,r,a,b,c)||SegmentCross(r,p,a,b,c)||SegmentCross(a,b,p,q,r)||SegmentCross(b,c,p,q,r)||SegmentCross(c,a,p,q,r))return true;
            Vec3 n=Vec3.Cross(b-a,c-a);double length=n.Length;
            if(length<1e-24||Math.Abs(Vec3.Dot(p-a,n))>length*1e-12||Math.Abs(Vec3.Dot(q-a,n))>length*1e-12||Math.Abs(Vec3.Dot(r-a,n))>length*1e-12)return false;
            if(StrictInside(p,a,b,c)||StrictInside(q,a,b,c)||StrictInside(r,a,b,c)||StrictInside(a,p,q,r)||StrictInside(b,p,q,r)||StrictInside(c,p,q,r))return true;
            // Strict crossing of coplanar edges also catches overlap without
            // an interior vertex (for example two opposing triangular tips).
            for(int i=0;i<3;i++)for(int j=0;j<3;j++)
            {
                Vec3 x=i==0?p:i==1?q:r,y=i==0?q:i==1?r:p,u=j==0?a:j==1?b:c,v=j==0?b:j==1?c:a;
                double s0=Vec3.Dot(n,Vec3.Cross(y-x,u-x)),s1=Vec3.Dot(n,Vec3.Cross(y-x,v-x)),t0=Vec3.Dot(n,Vec3.Cross(v-u,x-u)),t1=Vec3.Dot(n,Vec3.Cross(v-u,y-u));
                if(s0*s1<0&&t0*t1<0)return true;
            }
            return false;
        }
        public static double FirstContact(Vec3 p,Vec3 end,Vec3 q,Vec3 r,Vec3 a,Vec3 b,Vec3 c)
        {
            var first=new ContactTime{First=1};double t;
            if(Root(Volume(a,b,c,p),Volume(a,b,c,end),out t)&&InTriangle(p+(end-p)*t,a,b,c))Accept(t,ref first);
            VertexEvent(p,end,q,r,a,ref first);VertexEvent(p,end,q,r,b,ref first);VertexEvent(p,end,q,r,c,ref first);
            Vec3 normal=Vec3.Cross(b-a,c-a);
            EdgeEvents(p,end,q,a,b,normal,ref first);EdgeEvents(p,end,q,b,c,normal,ref first);EdgeEvents(p,end,q,c,a,normal,ref first);
            EdgeEvents(p,end,r,a,b,normal,ref first);EdgeEvents(p,end,r,b,c,normal,ref first);EdgeEvents(p,end,r,c,a,normal,ref first);
            // At an existing contact, a tiny temporal probe is insufficient:
            // its distance can fall below floating point / barycentric tolerance.
            // Only a separating plane over the full motion permits release.
            return first.Initial&&!SeparatingMotion(p,end,q,r,a,b,c)?0:first.First;
        }
        static bool SameSide(double a,double b,double c,double d,double e,double f)
        {
            return (a>=0&&b>=0&&c>=0&&d>=0&&e>=0&&f>=0&&(a>0||b>0||c>0||d>0||e>0||f>0))||
                   (a<=0&&b<=0&&c<=0&&d<=0&&e<=0&&f<=0&&(a<0||b<0||c<0||d<0||e<0||f<0));
        }
        static bool SeparatingMotion(Vec3 p,Vec3 end,Vec3 q,Vec3 r,Vec3 a,Vec3 b,Vec3 c)
        {
            double x=Volume(a,b,c,p),y=Volume(a,b,c,q),z=Volume(a,b,c,r),w=Volume(a,b,c,end);
            if(SameSide(x,y,z,w,y,z))return true;
            return SameSide(Volume(p,q,r,a),Volume(p,q,r,b),Volume(p,q,r,c),Volume(end,q,r,a),Volume(end,q,r,b),Volume(end,q,r,c));
        }
        public double Travel(int vertex,Vec3 direction,double travel)
        {
            lastFace=lastOther=lastVertex=-1;
            if(travel<=1e-13)return 0;long started=Stopwatch.GetTimestamp();double first=1;Vec3 p=positions[vertex],end=p+direction*travel;
            int seed=cachedFace[vertex],otherSeed=cachedOther[vertex];
            if(seed>=0&&otherSeed>=0)
            {
                int q=-1,r=-1;for(int k=0;k<3;k++){int i=faces[seed*3+k];if(i==vertex)continue;if(q<0)q=i;else r=i;}
                PairTests++;double hit=FirstContact(p,end,positions[q],positions[r],positions[faces[otherSeed*3]],positions[faces[otherSeed*3+1]],positions[faces[otherSeed*3+2]]);
                if(hit<1&&travel*hit<=Clearance)
                {
                    lastFace=seed;lastOther=otherSeed;lastVertex=vertex;lastTime=hit;lastEnd=end;LimitedMoves++;CachedBlockedMoves++;
                    Milliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;return 0;
                }
            }
            for(int j=offsets[vertex];j<offsets[vertex+1];j++)
            {
                int face=incident[j],q=-1,r=-1;
                for(int k=0;k<3;k++){int i=faces[face*3+k];if(i==vertex)continue;if(q<0)q=i;else r=i;}
                var bounds=Bounds3.Empty;bounds.Include(p);bounds.Include(end);bounds.Include(positions[q]);bounds.Include(positions[r]);
                BoundsQueries++;int count=tree.CollectTriangles(bounds,candidates);
                for(int k=0;k<count;k++)
                {
                    int other=candidates[k];if(Adjacent(face,other))continue;PairTests++;
                    Vec3 a=positions[faces[other*3]],b=positions[faces[other*3+1]],c=positions[faces[other*3+2]];
                    if(UsePlaneCertificates&&SeparatingMotion(p,end,positions[q],positions[r],a,b,c)){PlaneCertificates++;continue;}
                    double t=FirstContact(p,end,positions[q],positions[r],a,b,c);
                    if(t<first){first=t;lastFace=face;lastOther=other;lastVertex=vertex;lastTime=t;lastEnd=end;}
                }
            }
            if(first<1){LimitedMoves++;travel=Math.Max(0,travel*first-Clearance);cachedFace[vertex]=lastFace;cachedOther[vertex]=lastOther;}
            else cachedFace[vertex]=cachedOther[vertex]=-1;
            Milliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;
            return travel;
        }
        struct Witness {public Vec3 A,B,Normal;public double Distance;}
        static Vec3 UnitBary(int i){return i==0?new Vec3(1,0,0):i==1?new Vec3(0,1,0):new Vec3(0,0,1);}
        static Vec3 EdgeBary(int i,double t){return i==0?new Vec3(1-t,t,0):i==1?new Vec3(0,1-t,t):new Vec3(t,0,1-t);}
        static Vec3 Select(int i,Vec3 a,Vec3 b,Vec3 c){return i==0?a:i==1?b:c;}
        static Vec3 Bary(Vec3 p,Vec3 a,Vec3 b,Vec3 c)
        {
            Vec3 u=b-a,v=c-a,w=p-a;double aa=Vec3.Dot(u,u),ab=Vec3.Dot(u,v),bb=Vec3.Dot(v,v),den=aa*bb-ab*ab;
            if(den<1e-36)return new Vec3(1,0,0);double y=(bb*Vec3.Dot(w,u)-ab*Vec3.Dot(w,v))/den,z=(aa*Vec3.Dot(w,v)-ab*Vec3.Dot(w,u))/den;return new Vec3(1-y-z,y,z);
        }
        static void NearestSegments(Vec3 a,Vec3 b,Vec3 c,Vec3 d,out double s,out double t)
        {
            Vec3 u=b-a,v=d-c,w=a-c;double aa=Vec3.Dot(u,u),bb=Vec3.Dot(u,v),cc=Vec3.Dot(v,v),dd=Vec3.Dot(u,w),ee=Vec3.Dot(v,w),den=aa*cc-bb*bb;
            s=den>1e-30?Math.Max(0,Math.Min(1,(bb*ee-cc*dd)/den)):0;t=cc>1e-30?(bb*s+ee)/cc:0;
            if(t<0){t=0;s=aa>1e-30?Math.Max(0,Math.Min(1,-dd/aa)):0;}else if(t>1){t=1;s=aa>1e-30?Math.Max(0,Math.Min(1,(bb-dd)/aa)):0;}
        }
        static Witness ClosestWitness(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 e,Vec3 f)
        {
            var hit=new Witness{Distance=double.PositiveInfinity};
            for(int i=0;i<3;i++)
            {
                Vec3 p=Select(i,a,b,c),q=Geometry.ClosestPoint(p,d,e,f);double distance=(p-q).LengthSquared;
                if(distance<hit.Distance)hit=new Witness{Distance=distance,A=UnitBary(i),B=Bary(q,d,e,f),Normal=Vec3.Cross(e-d,f-d).Normalized};
                p=Select(i,d,e,f);q=Geometry.ClosestPoint(p,a,b,c);distance=(p-q).LengthSquared;
                if(distance<hit.Distance)hit=new Witness{Distance=distance,A=Bary(q,a,b,c),B=UnitBary(i),Normal=Vec3.Cross(b-a,c-a).Normalized};
            }
            for(int i=0;i<3;i++)for(int j=0;j<3;j++)
            {
                Vec3 p=Select(i,a,b,c),q=Select((i+1)%3,a,b,c),r=Select(j,d,e,f),s=Select((j+1)%3,d,e,f);double x,y;NearestSegments(p,q,r,s,out x,out y);double distance=(p+(q-p)*x-r-(s-r)*y).LengthSquared;
                if(distance<hit.Distance)hit=new Witness{Distance=distance,A=EdgeBary(i,x),B=EdgeBary(j,y),Normal=Vec3.Cross(q-p,s-r).Normalized};
            }
            return hit;
        }
        // Transfer a blocked request to BOTH participating faces. Targets are
        // changed, never accepted positions: every recipient still passes its
        // material bounds and continuous motion guard before it can move.
        public bool ProjectLastContact(Vec3[] targets,double[] inverseMass)
        {
            if(lastFace<0)return false;long started=Stopwatch.GetTimestamp();bool result=ProjectLastContactCore(targets,inverseMass);ReactionMilliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;return result;
        }
        bool ProjectLastContactCore(Vec3[] targets,double[] inverseMass)
        {
            if(lastFace<0)return false;int ia=faces[lastFace*3],ib=faces[lastFace*3+1],ic=faces[lastFace*3+2],id=faces[lastOther*3],ie=faces[lastOther*3+1],iff=faces[lastOther*3+2];
            Vec3 a=positions[ia],b=positions[ib],c=positions[ic],d=positions[id],e=positions[ie],f=positions[iff],motion=(lastEnd-positions[lastVertex])*lastTime;
            if(ia==lastVertex)a+=motion;else if(ib==lastVertex)b+=motion;else c+=motion;
            var hit=ClosestWitness(a,b,c,d,e,f);Vec3 n=hit.Normal;if(n.LengthSquared<.5)return false;
            Vec3 before=positions[ia]*hit.A.X+positions[ib]*hit.A.Y+positions[ic]*hit.A.Z-positions[id]*hit.B.X-positions[ie]*hit.B.Y-positions[iff]*hit.B.Z;
            double side=Vec3.Dot(before,n);
            if(Math.Abs(side)<1e-14){Vec3 relative=(targets[ia]-positions[ia])*hit.A.X+(targets[ib]-positions[ib])*hit.A.Y+(targets[ic]-positions[ic])*hit.A.Z-(targets[id]-positions[id])*hit.B.X-(targets[ie]-positions[ie])*hit.B.Y-(targets[iff]-positions[iff])*hit.B.Z;if(Vec3.Dot(relative,n)>0)n=-n;}
            else if(side<0)n=-n;
            Vec3 requested=targets[ia]*hit.A.X+targets[ib]*hit.A.Y+targets[ic]*hit.A.Z-targets[id]*hit.B.X-targets[ie]*hit.B.Y-targets[iff]*hit.B.Z;
            double gap=Vec3.Dot(requested,n)-Clearance;if(gap>=0)return false;
            double wa=inverseMass[ia],wb=inverseMass[ib],wc=inverseMass[ic],wd=inverseMass[id],we=inverseMass[ie],wf=inverseMass[iff];
            double denominator=wa*hit.A.X*hit.A.X+wb*hit.A.Y*hit.A.Y+wc*hit.A.Z*hit.A.Z+wd*hit.B.X*hit.B.X+we*hit.B.Y*hit.B.Y+wf*hit.B.Z*hit.B.Z;if(denominator<1e-30)return false;
            Vec3 correction=n*(-gap/denominator);targets[ia]+=correction*(wa*hit.A.X);targets[ib]+=correction*(wb*hit.A.Y);targets[ic]+=correction*(wc*hit.A.Z);targets[id]-=correction*(wd*hit.B.X);targets[ie]-=correction*(we*hit.B.Y);targets[iff]-=correction*(wf*hit.B.Z);
            PairCorrections++;MaximumRequestedGap=Math.Max(MaximumRequestedGap,-gap);return true;
        }
    }
}
