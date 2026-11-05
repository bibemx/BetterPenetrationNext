using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Four material meridians are intersected with the real chart triangles once.
    // A query interpolates barycentric weights INSIDE one cached triangle, so
    // every surface point retains its original skin identity, including seams.
    public sealed class ReceiverAdaptiveLandmarks
    {
        struct Span
        {
            public int A,B,C;public double U0,U1;public Vec3 W0,W1;
            public Vec3 Evaluate(Vec3[] p,double u){double t=Math.Max(0,Math.Min(1,(u-U0)/(U1-U0)));Vec3 w=W0*(1-t)+W1*t;return p[A]*w.X+p[B]*w.Y+p[C]*w.Z;}
        }
        readonly Span[][] meridians=new Span[4][];
        readonly double[] guideArc;
        public const int Capacity=65;
        public readonly Vec3[] Points=new Vec3[Capacity*5],SourcePairs=new Vec3[Capacity],ExternalPoints=new Vec3[5];
        public readonly double[] Coordinates=new double[Capacity],Arc=new double[Capacity];
        readonly int[] external=new int[4];
        public int Count {get;private set;}
        public int ReferenceTriangleTests {get;private set;}
        public int PointEvaluations {get;private set;}
        public double Spacing {get;private set;}
        public ReceiverAdaptiveLandmarks(SimulationDefinition definition,ReactiveMaterialChart chart,int guideCount,int[] externalIndices)
        {
            if(definition==null||chart==null||chart.SupportAxial==null||chart.SupportAngle==null||guideCount<2)throw new ArgumentException("Receiver material chart required");
            guideArc=new double[guideCount];var faces=definition.ConfinementTriangles;
            for(int ray=0;ray<4;ray++)
            {
                var spans=new List<Span>();double phase=ray*.25;
                for(int f=0;f<faces.Length;f+=3)
                {
                    ReferenceTriangleTests++;int a=faces[f],b=faces[f+1],c=faces[f+2];double ua=chart.SupportAxial[a],ub=chart.SupportAxial[b],uc=chart.SupportAxial[c];if(ua<0||ub<0||uc<0)continue;
                    double ta=chart.SupportAngle[a],tb=chart.SupportAngle[b],tc=chart.SupportAngle[c];tb+=Math.Round(ta-tb);tc+=Math.Round(ta-tc);
                    for(int turn=-1;turn<=1;turn++)
                    {
                        double line=phase+turn;int count=0;double lo=double.PositiveInfinity,hi=double.NegativeInfinity;Vec3 wl=new Vec3(),wh=new Vec3();
                        Cut(ua,ta,ub,tb,new Vec3(1,0,0),new Vec3(0,1,0),line,ref count,ref lo,ref hi,ref wl,ref wh);
                        Cut(ub,tb,uc,tc,new Vec3(0,1,0),new Vec3(0,0,1),line,ref count,ref lo,ref hi,ref wl,ref wh);
                        Cut(uc,tc,ua,ta,new Vec3(0,0,1),new Vec3(1,0,0),line,ref count,ref lo,ref hi,ref wl,ref wh);
                        if(count>=2&&hi-lo>1e-10)spans.Add(new Span{A=a,B=b,C=c,U0=lo,U1=hi,W0=wl,W1=wh});
                    }
                }
                spans.Sort((x,y)=>x.U0!=y.U0?x.U0.CompareTo(y.U0):y.U1.CompareTo(x.U1));if(spans.Count==0)throw new ArgumentException("Receiver meridian has no real surface");meridians[ray]=spans.ToArray();
            }
            for(int i=0;i<4;i++)external[i]=externalIndices!=null&&externalIndices.Length==4?externalIndices[i]:-1;
        }
        static void Point(double u,Vec3 w,ref int count,ref double lo,ref double hi,ref Vec3 wl,ref Vec3 wh)
        {count++;if(u<lo){lo=u;wl=w;}if(u>hi){hi=u;wh=w;}}
        static void Cut(double u0,double t0,double u1,double t1,Vec3 w0,Vec3 w1,double line,ref int count,ref double lo,ref double hi,ref Vec3 wl,ref Vec3 wh)
        {
            double a=t0-line,b=t1-line;if(Math.Abs(a)<1e-10)Point(u0,w0,ref count,ref lo,ref hi,ref wl,ref wh);
            if(Math.Abs(b)<1e-10)Point(u1,w1,ref count,ref lo,ref hi,ref wl,ref wh);
            if(a*b<0){double t=a/(a-b);Point(u0*(1-t)+u1*t,w0*(1-t)+w1*t,ref count,ref lo,ref hi,ref wl,ref wh);}
        }
        Vec3 Surface(int ray,double u,Vec3[] world)
        {
            var spans=meridians[ray];int lo=0,hi=spans.Length;
            while(lo<hi){int mid=(lo+hi)/2;if(spans[mid].U0<=u+1e-10)lo=mid+1;else hi=mid;}
            for(int i=Math.Min(spans.Length-1,lo-1);i>=0;i--){if(u>=spans[i].U0-1e-8&&u<=spans[i].U1+1e-8)return spans[i].Evaluate(world,u);if(spans[i].U0<u-.15)break;}
            throw new InvalidOperationException("Receiver material meridian is discontinuous at "+u);
        }
        public void Update(Vec3[] world,Vec3[] guide,double[] material,double sourceSpacing,Vec3[] sourceCentres)
        {
            if(guide.Length!=guideArc.Length||material.Length!=guide.Length||!Vec3.Finite(sourceSpacing)||sourceSpacing<=0)throw new ArgumentException("Invalid receiver landmark frame");
            guideArc[0]=0;for(int i=1;i<guide.Length;i++)guideArc[i]=guideArc[i-1]+(guide[i]-guide[i-1]).Length;
            double length=guideArc[guide.Length-1];Spacing=Math.Max(sourceSpacing,length/(Capacity-1));Count=Math.Max(2,Math.Min(Capacity,(int)Math.Ceiling(length/Spacing)+1));PointEvaluations=0;
            for(int i=0;i<Count;i++)
            {
                double distance=i==Count-1?length:Math.Min(length,i*Spacing);int a=0;while(a<guide.Length-2&&guideArc[a+1]<distance)a++;
                double t=(distance-guideArc[a])/Math.Max(1e-12,guideArc[a+1]-guideArc[a]),u=material[a]*(1-t)+material[a+1]*t;Coordinates[i]=u;Arc[i]=distance;
                Vec3 centre=new Vec3();for(int k=0;k<4;k++){Vec3 p=Surface(k,u,world);Points[i*5+k+1]=p;centre+=p*.25;PointEvaluations++;}Points[i*5]=centre;
                Vec3 nearest=centre;double best=double.PositiveInfinity;
                if(sourceCentres!=null)for(int k=0;k<sourceCentres.Length-1;k++){Vec3 e=sourceCentres[k+1]-sourceCentres[k];double f=e.LengthSquared<1e-20?0:Math.Max(0,Math.Min(1,Vec3.Dot(centre-sourceCentres[k],e)/e.LengthSquared));Vec3 p=sourceCentres[k]+e*f;double d=(p-centre).LengthSquared;if(d<best){best=d;nearest=p;}}SourcePairs[i]=nearest;
            }
            ExternalPoints[0]=Points[0];for(int k=0;k<4;k++)ExternalPoints[k+1]=external[k]>=0?world[external[k]]:Points[k+1];
        }
        public double NativeRadius(double u)
        {
            int a=0;while(a<Count-2&&Coordinates[a+1]<u)a++;double t=Math.Max(0,Math.Min(1,(u-Coordinates[a])/Math.Max(1e-12,Coordinates[a+1]-Coordinates[a])));
            return Radius(a)*(1-t)+Radius(a+1)*t;
        }
        public Vec3 CentreAtMaterial(double u)
        {int a=0;while(a<Count-2&&Coordinates[a+1]<u)a++;double t=Math.Max(0,Math.Min(1,(u-Coordinates[a])/Math.Max(1e-12,Coordinates[a+1]-Coordinates[a])));return Points[a*5]*(1-t)+Points[(a+1)*5]*t;}
        double Radius(int i){double r=double.PositiveInfinity;for(int k=1;k<=4;k++)r=Math.Min(r,(Points[i*5+k]-Points[i*5]).Length);return r;}
    }
}
