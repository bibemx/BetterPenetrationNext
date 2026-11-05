using System;

namespace BetterPenetrationNext.Core
{
    public struct BilateralPair
    {
        public readonly int A, B;
        public readonly Vec3 MaterialNormal;
        public BilateralPair(int a, int b, Vec3 materialNormal)
        { A=a; B=b; MaterialNormal=materialNormal; }
    }

    // Opt-in XPBD symmetry of procedural displacements, not of the animated
    // surface itself. Each vertex keeps its own skinning Jacobian and identity.
    // No nearest-position pairing, seam welding or Unity calls occur here.
    public sealed class BilateralDisplacementConstraint
    {
        struct State
        {
            public Vec3 AnchorA, AnchorB, Lambda;
            public Linear3 GradientA, GradientB, InverseGram;
            public double MassA, MassB;
            public bool Enabled;
        }
        readonly BilateralPair[] pairs;
        readonly State[] states;
        double alpha;
        bool prepared;
        public int VertexCount { get; private set; }
        public int PairCount { get { return pairs.Length; } }
        public int ActivePairs { get; private set; }
        public int SingularPairs { get; private set; }
        public int PinnedPairs { get; private set; }
        public int ProjectionPasses { get; private set; }
        public double MaximumResidual { get; private set; }
        static Linear3 Identity { get { return new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1)); } }

        public BilateralDisplacementConstraint(int vertexCount, BilateralPair[] bindings)
        {
            if(vertexCount<1)throw new ArgumentOutOfRangeException("vertexCount");
            if(bindings==null)throw new ArgumentNullException("bindings");
            VertexCount=vertexCount; pairs=(BilateralPair[])bindings.Clone(); states=new State[pairs.Length];
            var used=new bool[vertexCount];
            for(int i=0;i<pairs.Length;i++)
            {
                var p=pairs[i];
                if(p.A<0||p.B<0||p.A>=vertexCount||p.B>=vertexCount||p.A==p.B||used[p.A]||used[p.B])
                    throw new ArgumentException("Symmetry pairs must contain distinct, disjoint vertex identities.");
                if(!p.MaterialNormal.IsFinite||!Vec3.Finite(p.MaterialNormal.Length)||p.MaterialNormal.Length<1e-8)
                    throw new ArgumentException("A finite material-space mirror normal is required.");
                used[p.A]=used[p.B]=true;
                pairs[i]=new BilateralPair(p.A,p.B,p.MaterialNormal.Normalized);
            }
        }

        public void Reset()
        {
            prepared=false; Array.Clear(states,0,states.Length);
            ActivePairs=SingularPairs=PinnedPairs=ProjectionPasses=0; MaximumResidual=0;
        }

        public void BeginStep(Vec3[] animated, Linear3[] skinningFrames, double[] inverseMass, double dt, double compliance)
        {
            prepared=false;
            if(animated==null||animated.Length!=VertexCount||inverseMass==null||inverseMass.Length!=VertexCount||
               (skinningFrames!=null&&skinningFrames.Length!=VertexCount))throw new ArgumentException("Symmetry topology changed.");
            if(!Vec3.Finite(dt)||dt<=0||!Vec3.Finite(compliance)||compliance<0)
                throw new ArgumentException("Invalid symmetry step or compliance.");
            alpha=compliance/(dt*dt);
            if(!Vec3.Finite(alpha))throw new ArgumentException("Symmetry compliance divided by step squared overflowed.");
            ActivePairs=SingularPairs=PinnedPairs=ProjectionPasses=0; MaximumResidual=0;
            for(int i=0;i<pairs.Length;i++)
            {
                var p=pairs[i]; Validate(animated[p.A],inverseMass[p.A]); Validate(animated[p.B],inverseMass[p.B]);
                Linear3 fa=skinningFrames==null?Identity:skinningFrames[p.A],fb=skinningFrames==null?Identity:skinningFrames[p.B];
                if(!Finite(fa)||!Finite(fb))throw new ArgumentException("Non-finite skinning Jacobian.");
            }
            for(int i=0;i<pairs.Length;i++)
            {
                var p=pairs[i]; var s=new State { AnchorA=animated[p.A],AnchorB=animated[p.B],MassA=inverseMass[p.A],MassB=inverseMass[p.B] };
                Linear3 fa=skinningFrames==null?Identity:skinningFrames[p.A],fb=skinningFrames==null?Identity:skinningFrames[p.B];
                Linear3 ia,ib;
                if(!fa.TryInverse(out ia)||!fb.TryInverse(out ib)||!Finite(ia)||!Finite(ib))SingularPairs++;
                else if(s.MassA+s.MassB==0)PinnedPairs++;
                else
                {
                    s.GradientA=ia;
                    // C = inv(Ja) da - mirror(inv(Jb) db).
                    s.GradientB=new Linear3(Reflect(ib.X,p.MaterialNormal),Reflect(ib.Y,p.MaterialNormal),Reflect(ib.Z,p.MaterialNormal));
                    var g=new Linear3(
                        GramColumn(s.GradientA,0)*s.MassA+GramColumn(s.GradientB,0)*s.MassB+new Vec3(alpha,0,0),
                        GramColumn(s.GradientA,1)*s.MassA+GramColumn(s.GradientB,1)*s.MassB+new Vec3(0,alpha,0),
                        GramColumn(s.GradientA,2)*s.MassA+GramColumn(s.GradientB,2)*s.MassB+new Vec3(0,0,alpha));
                    if(Finite(g)&&g.TryInverse(out s.InverseGram)&&Finite(s.InverseGram)) { s.Enabled=true; ActivePairs++; }
                    else SingularPairs++;
                }
                states[i]=s;
            }
            prepared=true;
        }

        public void Project(Vec3[] positions)
        {
            ValidatePositions(positions); MaximumResidual=0;
            for(int i=0;i<pairs.Length;i++)
            {
                var s=states[i]; if(!s.Enabled)continue; var p=pairs[i];
                Vec3 c=s.GradientA.Multiply(positions[p.A]-s.AnchorA)-s.GradientB.Multiply(positions[p.B]-s.AnchorB);
                Vec3 delta=s.InverseGram.Multiply(-(c+s.Lambda*alpha));
                positions[p.A]+=TransposeMultiply(s.GradientA,delta)*s.MassA;
                positions[p.B]-=TransposeMultiply(s.GradientB,delta)*s.MassB;
                s.Lambda+=delta; states[i]=s;
                c=s.GradientA.Multiply(positions[p.A]-s.AnchorA)-s.GradientB.Multiply(positions[p.B]-s.AnchorB);
                MaximumResidual=Math.Max(MaximumResidual,c.Length);
            }
            ProjectionPasses++;
        }

        // Call after contact/strain constraints to report the residual actually
        // returned by the solver. A competing contact need not be symmetric.
        public double MeasureResidual(Vec3[] positions)
        {
            ValidatePositions(positions); MaximumResidual=0;
            for(int i=0;i<pairs.Length;i++)
            {
                var s=states[i]; if(!s.Enabled)continue; var p=pairs[i];
                Vec3 c=s.GradientA.Multiply(positions[p.A]-s.AnchorA)-s.GradientB.Multiply(positions[p.B]-s.AnchorB);
                MaximumResidual=Math.Max(MaximumResidual,c.Length);
            }
            return MaximumResidual;
        }
        void ValidatePositions(Vec3[] positions)
        {
            if(!prepared)throw new InvalidOperationException("BeginStep must succeed before symmetry projection.");
            if(positions==null||positions.Length!=VertexCount)throw new ArgumentException("Symmetry topology changed.");
            for(int i=0;i<pairs.Length;i++)if(!positions[pairs[i].A].IsFinite||!positions[pairs[i].B].IsFinite)
                throw new ArgumentException("Non-finite symmetry position.");
        }
        static void Validate(Vec3 p,double mass)
        { if(!p.IsFinite||!Vec3.Finite(mass)||mass<0)throw new ArgumentException("Invalid animated symmetry position or inverse mass."); }
        static bool Finite(Linear3 m) { return m.X.IsFinite&&m.Y.IsFinite&&m.Z.IsFinite; }
        static Vec3 Reflect(Vec3 v,Vec3 n) { return v-n*(2*Vec3.Dot(n,v)); }
        static Vec3 TransposeMultiply(Linear3 m,Vec3 v) { return new Vec3(Vec3.Dot(m.X,v),Vec3.Dot(m.Y,v),Vec3.Dot(m.Z,v)); }
        static Vec3 GramColumn(Linear3 m,int axis) { return m.X*m.X.Axis(axis)+m.Y*m.Y.Axis(axis)+m.Z*m.Z.Axis(axis); }
    }
}
