using System;
namespace BetterPenetrationNext.Core
{
    // Dimension overrides modify this frame's animation basis before landmark
    // and contact evaluation. The original indexed skin and asset stay intact.
    public sealed class SourceDimensions
    {
        readonly SourceSectionRig sections;readonly int[] spans;readonly double[] blends;readonly Vec3[] tangents;
        Vec3 scales=new Vec3(1,1,0),velocity;
        public double MaximumOffset {get;private set;}
        public double NativeLength {get;private set;}
        public double EffectiveLength {get;private set;}
        public double NativeWidth {get;private set;}
        public double EffectiveWidth {get;private set;}
        public SourceDimensions(SourceSectionRig rig,double initialLengthScale=1,double initialWidthScale=1)
        {
            if(!Vec3.Finite(initialLengthScale)||!Vec3.Finite(initialWidthScale)||initialLengthScale<.5||initialLengthScale>1.5||initialWidthScale<.5||initialWidthScale>1.5)throw new ArgumentException("Invalid initial source dimensions");
            scales=new Vec3(initialLengthScale,initialWidthScale,0);
            sections=rig??throw new ArgumentNullException("rig");spans=new int[rig.VertexCount];blends=new double[rig.VertexCount];tangents=new Vec3[rig.SectionCount-1];
            for(int i=0;i<spans.Length;i++){double u=rig.MaterialCoordinate(i)*(rig.SectionCount-1);spans[i]=Math.Min(rig.SectionCount-2,(int)u);blends[i]=u-spans[i];}
        }
        public void Apply(Vec3[] animated,Vec3 root,double lengthScale,double widthScale,double dt,double response,Vec3[] output)
        {
            if(animated.Length!=spans.Length||output.Length!=spans.Length||!Vec3.Finite(lengthScale)||!Vec3.Finite(widthScale)||lengthScale<.5||lengthScale>1.5||widthScale<.5||widthScale>1.5)throw new ArgumentException("Invalid source dimension override");
            MaximumOffset=0;
            if(lengthScale==1&&widthScale==1&&(scales-new Vec3(1,1,0)).LengthSquared<1e-20){Array.Copy(animated,output,animated.Length);return;}
            CriticalMotion.Advance(ref scales,ref velocity,new Vec3(lengthScale,widthScale,0),Math.Max(.001,response),Math.Min(.25,Math.Max(0,dt)));
            sections.CaptureAnimation(animated,root);NativeLength=NativeWidth=0;
            for(int s=0;s<tangents.Length;s++){Vec3 edge=sections.AnimatedCentre(s+1)-sections.AnimatedCentre(s);NativeLength+=edge.Length;tangents[s]=edge.Normalized;}
            for(int i=0;i<animated.Length;i++)
            {
                double coordinate=sections.MaterialCoordinate(i);if(coordinate<=1e-8){output[i]=animated[i];continue;}
                int a=spans[i];double t=blends[i];Vec3 centre=sections.AnimatedCentre(a)*(1-t)+sections.AnimatedCentre(a+1)*t,n=tangents[a],d=animated[i]-centre,axial=n*Vec3.Dot(d,n),radial=d-axial;
                // The attached base has zero dimension correction; width
                // blends continuously into the requested shaft proportion.
                double w=Math.Max(0,Math.Min(1,coordinate/.125));w=w*w*(3-2*w);
                output[i]=root+(centre-root)*scales.X+axial*(1+(scales.X-1)*w)+radial*(1+(scales.Y-1)*w);
                if(coordinate>=.125)NativeWidth=Math.Max(NativeWidth,radial.Length*2);
                MaximumOffset=Math.Max(MaximumOffset,(output[i]-animated[i]).Length);
            }
            EffectiveLength=NativeLength*scales.X;EffectiveWidth=NativeWidth*scales.Y;
        }
    }
}
