using System;
namespace BetterPenetrationNext.Core
{
    // Secondary layers follow an interpolated landmark velocity field in the
    // stable material frame. Render weights and mesh topology remain separate.
    public sealed class ReceiverFlowTransfer
    {
        readonly Vec3[] rest,state,previous,target,velocity,sample;
        readonly int[] physical;
        readonly double[] influence;
        readonly EdgeConstraint[] edges;
        readonly PointMotionBvh tree;
        readonly int[] queryHints;
        readonly bool[] isPhysical;
        readonly bool cached,correctTime;
        readonly Vec3[] midpoint;
        public int LastSubsteps { get; private set; }
        public long SearchQueries { get { return tree.Queries; } }
        public long SearchPointTests { get { return tree.PointTests; } }
        public long SearchVisits { get { return tree.Visits; } }
        public long SearchHintTests { get { return tree.HintPointTests; } }

        public ReceiverFlowTransfer(Vec3[] fullRest,Vec3[] physicalRest,EdgeConstraint[] edges,ReceiverSupportTransfer mapping,double[] influence,bool cached=false,bool correctTime=false)
        {
            rest=(Vec3[])fullRest.Clone();state=(Vec3[])fullRest.Clone();previous=(Vec3[])physicalRest.Clone();
            target=new Vec3[previous.Length];velocity=new Vec3[previous.Length];sample=new Vec3[previous.Length];
            this.edges=edges;physical=(int[])mapping.PhysicalIndices.Clone();this.influence=(double[])influence.Clone();tree=new PointMotionBvh(physicalRest);
            this.cached=cached;this.correctTime=correctTime;if(correctTime)midpoint=new Vec3[fullRest.Length];
            if(cached)
            {
                queryHints=new int[fullRest.Length*tree.NeighbourCount];for(int i=0;i<queryHints.Length;i++)queryHints[i]=-1;
                isPhysical=new bool[fullRest.Length];foreach(int index in physical)isPhysical[index]=true;
            }
        }
        public void Apply(SoftBodySolver solver,Vec3[] physicalBase,Vec3[] fullBase,Linear3 root,Vec3 translation,Vec3[] output)
        {
            Linear3 inverse;if(!root.TryInverse(out inverse))throw new ArgumentException("Stable rig transform is singular");
            for(int i=0;i<target.Length;i++)target[i]=inverse.Multiply(physicalBase[i]+solver.Displacement(i)-translation);
            Integrate(fullBase,root,translation,output);
        }
        public void ApplyLocalTargets(Vec3[] localTargets,Vec3[] fullBase,Linear3 root,Vec3 translation,Vec3[] output)
        {
            if(localTargets==null||localTargets.Length!=target.Length)throw new ArgumentException("Landmark topology changed");
            Array.Copy(localTargets,target,target.Length);Integrate(fullBase,root,translation,output);
        }
        void Integrate(Vec3[] fullBase,Linear3 root,Vec3 translation,Vec3[] output)
        {
            tree.ResetStatistics();double motion=0,restDistance=0;bool moving=false;
            for(int i=0;i<target.Length;i++)
            {
                velocity[i]=target[i]-previous[i];moving|=velocity[i].LengthSquared!=0;
                restDistance=Math.Max(restDistance,(target[i]-rest[physical[i]]).Length);
            }
            foreach(var edge in edges)
            {
                double length=(previous[edge.A]-previous[edge.B]).Length;
                if(length>1e-8)motion=Math.Max(motion,(velocity[edge.A]-velocity[edge.B]).Length/length);
            }
            int steps=cached&&!moving?0:Math.Max(1,Math.Min(12,(int)Math.Ceiling(motion/.2)));LastSubsteps=steps;
            for(int s=0;s<steps;s++)
            {
                if(correctTime)
                {
                    // The velocity field changes with landmark time. Midpoint RK2
                    // evaluates its first stage at the start, then at the middle.
                    double step=1.0/steps,t0=(double)s/steps,tm=(s+.5)/steps;
                    for(int i=0;i<sample.Length;i++)sample[i]=previous[i]+velocity[i]*t0;
                    tree.Refit(sample);
                    for(int i=0;i<state.Length;i++)if(influence[i]>0&&(!cached||!isPhysical[i]))
                        midpoint[i]=state[i]+tree.Velocity(state[i],velocity,queryHints,i*tree.NeighbourCount)*(influence[i]*step*.5);
                    for(int i=0;i<sample.Length;i++)sample[i]=previous[i]+velocity[i]*tm;
                    tree.Refit(sample);
                    for(int i=0;i<state.Length;i++)if(influence[i]>0&&(!cached||!isPhysical[i]))
                        state[i]+=tree.Velocity(midpoint[i],velocity,queryHints,i*tree.NeighbourCount)*(influence[i]*step);
                    continue;
                }
                double t=(s+.5)/steps;for(int i=0;i<sample.Length;i++)sample[i]=previous[i]+velocity[i]*t;
                tree.Refit(sample);double h=1.0/steps;
                for(int i=0;i<state.Length;i++)if(influence[i]>0&&(!cached||!isPhysical[i]))
                {
                    int hint=i*tree.NeighbourCount;
                    Vec3 a=tree.Velocity(state[i],velocity,queryHints,hint)*influence[i];
                    Vec3 mid=state[i]+a*(h*.5),b=tree.Velocity(mid,velocity,queryHints,hint)*influence[i];
                    state[i]+=b*h;
                }
            }
            for(int i=0;i<physical.Length;i++){state[physical[i]]=target[i];previous[i]=target[i];}
            if(restDistance<1e-10)Array.Copy(rest,state,rest.Length);
            for(int i=0;i<output.Length;i++)output[i]=root.Multiply(state[i])+translation-fullBase[i];
        }
    }
}
