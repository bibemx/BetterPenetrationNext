using System;

namespace BetterPenetrationNext.Core
{
    public enum ContactMode { Original, OneWay, Bidirectional }
    public enum PathMode { Disabled, SourceFollowsPath, AnatomyConforms }
    public sealed class BidirectionalContactSettings
    {
        public ContactMode Mode=ContactMode.Bidirectional;
        public double TissueCompliance=1e-7, TissueDeformationStrength=1, MaxTissueStrain=.25;
        public double SourceCorrectionStrength=.35, MaxSourceDisplacement=.05, ContactDamping=12, RecoveryTime=4.5;
        // Optional explicit per-particle material labels. No anatomy is inferred from coordinates.
        public double[] RegionCompliance;
        public bool UseActiveContactList=true,UseTriangleHints=true,ProfileStages;
        public void Validate(int count)
        {
            Nonnegative(TissueCompliance); Nonnegative(TissueDeformationStrength); Nonnegative(MaxTissueStrain); Nonnegative(SourceCorrectionStrength); Nonnegative(MaxSourceDisplacement); Nonnegative(ContactDamping); Nonnegative(RecoveryTime);
            if(RegionCompliance!=null) { if(RegionCompliance.Length!=count) throw new ArgumentException("Region count differs from simulation."); foreach(double x in RegionCompliance) if(!Vec3.Finite(x)||x<0) throw new ArgumentException("Invalid regional compliance."); }
        }
        static void Nonnegative(double x) { if(!Vec3.Finite(x)||x<0) throw new ArgumentException("Invalid bidirectional contact parameter."); }
    }
    public interface ICoupledContactSurface : IContactSurface
    { bool TryContact(Vec3 point,double reach,out SurfaceSample sample,out SourceContactBody source); }

    // A six-DOF animated geometric source. Contact solves translation; the path
    // supplies bounded orientation. Positions always refer to this frame's animation.
    public sealed class SourceContactBody : IHintedCoupledSurface,IHintedContactSurface,IContactBounds
    {
        public bool PreserveContactAcrossDeformation;
        public IContactSurface Surface;
        public ISourceDeformation Deformation;
        double TranslationBudget { get { return Math.Max(0,MaximumDisplacement-(Deformation==null?0:Deformation.MaximumDisplacement)); } }
        bool HasDeformationBounds {
            get {
                if(Deformation==null)return false;var b=Deformation.DisplacementBounds;
                if(!b.Min.IsFinite||!b.Max.IsFinite)return false;
                // An AABB of displacement vectors may overestimate the true
                // vector set. Use it only when its undeformed-offset corners
                // are feasible; otherwise retain the conservative ball bound.
                for(int i=0;i<8;i++)if(Corner(b,i).LengthSquared>MaximumDisplacement*MaximumDisplacement)return false;
                return true;
            }
        }
        static Vec3 Corner(Bounds3 b,int i){return new Vec3((i&1)==0?b.Min.X:b.Max.X,(i&2)==0?b.Min.Y:b.Max.Y,(i&4)==0?b.Min.Z:b.Max.Z);}
        public Vec3 Offset, Pivot, RotationVector;
        public Vec3 TranslationAxis;
        public double MaximumContactTravel=double.PositiveInfinity;
        Vec3 guideOrigin;
        public double InverseMass, MaximumDisplacement;
        public Vec3 LastContactReaction;
        public bool ReleaseOnlyWhenSeparated=true;
        public bool HadContact { get; private set; }
        public double SaturationRatio { get { return MaximumDisplacement>1e-15?MaximumCorrection/MaximumDisplacement:0; } }
        public double MaximumCorrection {
            get {
                if(Deformation!=null) {
                    if(!HasDeformationBounds)return Deformation.MaximumDisplacement+Offset.Length;
                    double m=0;for(int i=0;i<8;i++)m=Math.Max(m,(Offset+Corner(Deformation.DisplacementBounds,i)).Length);return m;
                }
                if(RotationVector.LengthSquared<1e-24) return Offset.Length;
                var bounded=Surface as IContactBounds; if(bounded==null) return Offset.Length;
                var b=bounded.ContactBounds; if(!b.Min.IsFinite||!b.Max.IsFinite) return Offset.Length;
                double maximum=0; for(int i=0;i<8;i++) maximum=Math.Max(maximum,(Offset+CornerRotation(b,i)).Length);
                return maximum;
            }
        }
        public bool AtLimit { get { return SaturationRatio>=.999; } }
        Vec3 appliedGuidance;
        public bool HasVolumeBounds { get { var b=Surface as IContactBounds; return b!=null&&b.HasVolumeBounds; } }
        public Bounds3 ContactBounds
        {
            get {
                var bounded=Surface as IContactBounds; if(bounded==null) return Bounds3.Unbounded;
                var b=bounded.ContactBounds; var result=Bounds3.Empty;
                if(RotationVector.LengthSquared<1e-24) return new Bounds3(b.Min+Offset,b.Max+Offset);
                for(int i=0;i<8;i++) result.Include(RigidCorrected(new Vec3((i&1)==0?b.Min.X:b.Max.X,(i&2)==0?b.Min.Y:b.Max.Y,(i&4)==0?b.Min.Z:b.Max.Z)));
                return result;
            }
        }
        public void BeginFrame(BidirectionalContactSettings s,double dt,bool anatomyConforms=false)
        {
            if(!Vec3.Finite(dt)||dt<0) throw new ArgumentException("Invalid frame time.");
            MaximumDisplacement=s.MaxSourceDisplacement;
            InverseMass=s.Mode==ContactMode.Bidirectional?s.SourceCorrectionStrength*(anatomyConforms?.05:1):0;
            Offset-=appliedGuidance; appliedGuidance=new Vec3(); Deformation=null;
            if(!ReleaseOnlyWhenSeparated||!HadContact) Offset*=Math.Exp(-s.ContactDamping*dt);
            Offset=Vec3.ClampLength(Offset,MaximumDisplacement); HadContact=false;
            RotationVector=new Vec3(); TranslationAxis=new Vec3(); LastContactReaction=new Vec3();MaximumContactTravel=double.PositiveInfinity;guideOrigin=new Vec3();
            if(s.Mode!=ContactMode.Bidirectional) { Offset=new Vec3(); RotationVector=new Vec3(); }
            LimitPose();
        }
        public void LimitPose()
        {
            // Preserve requested translation first. In 1.0.5 rotation consumed
            // the entire budget and silently reduced a valid guide offset to 0.
            Offset=Vec3.ClampLength(Offset,Math.Max(0,MaximumDisplacement));
            if(Deformation!=null){
                if(PreserveContactAcrossDeformation&&HasDeformationBounds){Vec3 requested=Offset;Offset=new Vec3();double length=requested.Length;if(length>1e-20)Offset=requested*(Math.Min(length,PoseTravel(requested/length))/length);}
                else Offset=Vec3.ClampLength(Offset,TranslationBudget);
                RotationVector=new Vec3();return;
            }
            RotationVector=Vec3.ClampLength(RotationVector,Math.PI);
            var bounds=Surface as IContactBounds;
            if(bounds!=null&&RotationVector.LengthSquared>1e-24&&bounds.ContactBounds.Min.IsFinite&&bounds.ContactBounds.Max.IsFinite) {
                var b=bounds.ContactBounds;
                if(!WithinBudget(b)) {
                    Vec3 desired=RotationVector; double low=0,high=1;
                    for(int iteration=0;iteration<36;iteration++) {
                        double fraction=(low+high)*.5; RotationVector=desired*fraction;
                        if(WithinBudget(b)) low=fraction; else high=fraction;
                    }
                    RotationVector=desired*low;
                }
            }
        }
        Vec3 CornerRotation(Bounds3 b,int i)
        { Vec3 p=new Vec3((i&1)==0?b.Min.X:b.Max.X,(i&2)==0?b.Min.Y:b.Max.Y,(i&4)==0?b.Min.Z:b.Max.Z)-Pivot; return Rotate(p)-p; }
        bool WithinBudget(Bounds3 bounds)
        { for(int i=0;i<8;i++) if((Offset+CornerRotation(bounds,i)).LengthSquared>MaximumDisplacement*MaximumDisplacement) return false; return true; }
        public Vec3 Rotate(Vec3 v,bool inverse=false)
        {
            double angle=RotationVector.Length; if(angle<1e-14) return v;
            Vec3 axis=RotationVector/angle; if(inverse) angle=-angle;
            return v*Math.Cos(angle)+Vec3.Cross(axis,v)*Math.Sin(angle)+axis*(Vec3.Dot(axis,v)*(1-Math.Cos(angle)));
        }
        Vec3 RigidCorrected(Vec3 p) { return Pivot+Rotate(p-Pivot)+Offset; }
        public Vec3 ToCorrected(Vec3 p) { return RigidCorrected(Deformation==null?p:Deformation.Map(p)); }
        public Vec3 ToCorrectedVertex(int index,Vec3 p){var indexed=Deformation as IIndexedSourceDeformation;return RigidCorrected(indexed==null?(Deformation==null?p:Deformation.Map(p)):indexed.MapVertex(index,p));}
        public bool TrySample(Vec3 p,double reach,out SurfaceSample sample)
        {
            if(Surface==null) { sample=new SurfaceSample(); return false; }
            bool found=Surface.TrySample(Pivot+Rotate(p-Pivot-Offset,true),reach,out sample);
            if(found) { sample.Point=RigidCorrected(sample.Point); sample.Normal=Rotate(sample.Normal); } return found;
        }
        public bool TrySampleHinted(Vec3 p,double reach,ref int hint,out SurfaceSample sample)
        {
            var surface=Surface as IHintedContactSurface;
            if(surface==null) return TrySample(p,reach,out sample);
            bool found=surface.TrySampleHinted(Pivot+Rotate(p-Pivot-Offset,true),reach,ref hint,out sample);
            if(found) {sample.Point=RigidCorrected(sample.Point);sample.Normal=Rotate(sample.Normal);} return found;
        }
        public bool TryContactHinted(Vec3 p,double reach,ref int hint,out SurfaceSample sample,out SourceContactBody source)
        {source=this;return TrySampleHinted(p,reach,ref hint,out sample);}
        public void MarkContact() { HadContact=true; }
        public Vec3 ProjectContactGradient(Vec3 normal) { return TranslationAxis.LengthSquared>1e-20?TranslationAxis.Normalized*Vec3.Dot(TranslationAxis.Normalized,normal):normal; }
        public void ApplyDeformation(ISourceDeformation deformation) { Deformation=deformation;if(!PreserveContactAcrossDeformation)Offset=new Vec3();RotationVector=new Vec3();appliedGuidance=new Vec3();guideOrigin=new Vec3();LimitPose(); }
        public void ApplyGuidance(Vec3 translation,Vec3 rotation,Vec3 pivot)
        {
            // The guide is an absolute target relative to this frame's animated
            // pose. Re-adding yesterday's contact support makes it a second
            // uncommanded guide and can push the tip farther from the route.
            // Contact is projected after this target, into the same pose budget.
            Offset=translation; RotationVector=rotation; Pivot=pivot; LimitPose(); appliedGuidance=Offset;guideOrigin=Offset;
        }
        public bool TryContact(Vec3 p,double reach,out SurfaceSample sample,out SourceContactBody source)
        { source=this; return TrySample(p,reach,out sample); }
        public double Travel(Vec3 direction)
        { return Math.Min(PoseTravel(direction),double.IsPositiveInfinity(MaximumContactTravel)?double.PositiveInfinity:BidirectionalContactSolver.RayLimit(Offset-guideOrigin,direction,MaximumContactTravel)); }
        double PoseTravel(Vec3 direction)
        {
            if(HasDeformationBounds){double amount=double.PositiveInfinity;var deformationBounds=Deformation.DisplacementBounds;for(int i=0;i<8;i++)amount=Math.Min(amount,BidirectionalContactSolver.RayLimit(Offset+Corner(deformationBounds,i),direction,MaximumDisplacement));return amount;}
            if(RotationVector.LengthSquared<1e-24) return BidirectionalContactSolver.RayLimit(Offset,direction,TranslationBudget);
            var bounded=Surface as IContactBounds;
            if(bounded==null||!bounded.ContactBounds.Min.IsFinite||!bounded.ContactBounds.Max.IsFinite) return BidirectionalContactSolver.RayLimit(Offset,direction,MaximumDisplacement);
            double travel=double.PositiveInfinity; var b=bounded.ContactBounds;
            for(int i=0;i<8;i++) travel=Math.Min(travel,BidirectionalContactSolver.RayLimit(Offset+CornerRotation(b,i),direction,MaximumDisplacement));
            return travel;
        }
        public void Correct(Vec3 delta) {
            double length=delta.Length; if(length<1e-20) return;
            Vec3 accepted=delta*(Math.Min(length,Travel(delta/length))/length); Offset+=accepted; LastContactReaction+=accepted;
        }
    }
    public static class BidirectionalContactSolver
    {
        // Unilateral XPBD multiplier. alpha = compliance / h^2. Both gradients
        // use the same normal with opposite signs, including the shared source DOF.
        public static double Multiplier(double gap,double lambda,double tissueMobility,double sourceMobility,double alpha)
        {
            double denominator=tissueMobility+sourceMobility+alpha;
            return denominator>1e-30?Math.Max(0,lambda+(-gap-alpha*lambda)/denominator):lambda;
        }
        public static double RayLimit(Vec3 offset,Vec3 unit,double radius)
        {
            double c=offset.LengthSquared-radius*radius,b=Vec3.Dot(offset,unit);
            if(c>Math.Max(1e-16,radius*radius*1e-9)) return 0;
            return Math.Max(0,-b+Math.Sqrt(Math.Max(0,b*b-c)));
        }
    }
}
