using System;
using BetterPenetrationNext.Core;

static class BilateralConstraintTests
{
    static readonly Vec3 X=new Vec3(1,0,0);
    static readonly Linear3 I=new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1));
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    static void Near(Vec3 a,Vec3 b,string message,double tolerance=1e-11){Require((a-b).Length<=tolerance,message+": "+(a-b).Length);}
    static BilateralDisplacementConstraint Pair(int count=2){return new BilateralDisplacementConstraint(count,new[]{new BilateralPair(0,1,X)});}
    static Vec3 Mirror(Vec3 p){return new Vec3(-p.X,p.Y,p.Z);}
    static Linear3 Frame(double angle,double scale=1)
    {return new Linear3(new Vec3(Math.Cos(angle),Math.Sin(angle),0)*scale,new Vec3(-Math.Sin(angle),Math.Cos(angle),0),new Vec3(0,0,.8));}
    public static void Run(Action<string,Action> test)
    {
        test("Bilateral constraint changes offsets while retaining asymmetric animated geometry",()=>{
            var basis=new[]{new Vec3(-1,.2,0),new Vec3(1.2,-.1,.03),new Vec3(3,0,0)};var p=(Vec3[])basis.Clone();var s=Pair(3);
            s.BeginStep(basis,null,new[]{1.0,1.0,1.0},1.0/60,0);s.Project(p);for(int i=0;i<p.Length;i++)Near(p[i],basis[i],"Equilibrium changed",0);
            p[0]+=new Vec3(-.2,.1,.3);s.Project(p);Near(p[0]-basis[0],new Vec3(-.1,.05,.15),"Left projection");Near(p[1]-basis[1],new Vec3(.1,.05,.15),"Right projection");Near(p[2],basis[2],"Unpaired vertex moved",0);
        });
        test("Bilateral inverse masses allocate the correction and preserve hard pins",()=>{
            var basis=new[]{new Vec3(-1,0,0),new Vec3(1,0,0)};var p=(Vec3[])basis.Clone();var s=Pair();p[0]+=X*.1;
            s.BeginStep(basis,null,new[]{1.0,3.0},.01,0);s.Project(p);Near(p[0]-basis[0],X*.075,"Mass A");Near(p[1]-basis[1],X*(-.075),"Mass B");
            p=(Vec3[])basis.Clone();p[1]+=new Vec3(.2,.1,.1);s.BeginStep(basis,null,new[]{0.0,1.0},.01,0);s.Project(p);Near(p[0],basis[0],"Pinned vertex",0);Near(p[1],basis[1],"Free partner did not converge");
            s.BeginStep(basis,null,new[]{0.0,0.0},.01,0);Require(s.PinnedPairs==1&&s.ActivePairs==0,"Pinned-pair diagnostic");
        });
        test("Bilateral projection respects separate nonuniform skinning Jacobians",()=>{
            var basis=new[]{new Vec3(-2,3,5),new Vec3(1,2,4)};var p=(Vec3[])basis.Clone();var frames=new[]{Frame(.8,1.7),new Linear3(new Vec3(1,.3,.1),new Vec3(.2,1.3,0),new Vec3(0,.2,.7))};
            p[0]+=frames[0].Multiply(new Vec3(-.1,.07,.2));p[1]+=frames[1].Multiply(new Vec3(.04,-.1,.06));var s=Pair();s.BeginStep(basis,frames,new[]{.5,1.5},.02,0);s.Project(p);
            Vec3 a,b;Require(frames[0].TryInverseMultiply(p[0]-basis[0],out a)&&frames[1].TryInverseMultiply(p[1]-basis[1],out b),"Invertible test frames");
            frames[0].TryInverseMultiply(p[0]-basis[0],out a);frames[1].TryInverseMultiply(p[1]-basis[1],out b);Near(a,Mirror(b),"Material-space mirror");Require(s.MaximumResidual<1e-11,"Residual");
        });
        test("Bilateral compliance follows XPBD and repeated iterations do not harden it",()=>{
            var basis=new[]{new Vec3(-1,0,0),new Vec3(1,0,0)};var p=(Vec3[])basis.Clone();p[0]+=new Vec3(0,0,.1);var s=Pair();const double dt=.02,compliance=.001;
            s.BeginStep(basis,null,new[]{1.0,1.0},dt,compliance);s.Project(p);double expected=.1*(compliance/(dt*dt))/(2+compliance/(dt*dt));Require(Math.Abs(s.MaximumResidual-expected)<1e-12,"XPBD soft residual");var one=(Vec3[])p.Clone();for(int i=0;i<20;i++)s.Project(p);Near(p[0],one[0],"Iteration changed compliance");Near(p[1],one[1],"Iteration changed compliance");
        });
        test("Bilateral animation rebases each step and retains independently animated seam vertices",()=>{
            var basis=new[]{new Vec3(-1,0,0),new Vec3(1,0,0),new Vec3(-1,0,0),new Vec3(1,0,0)};var p=new Vec3[4];var f=new Linear3[4];
            var s=new BilateralDisplacementConstraint(4,new[]{new BilateralPair(0,1,X),new BilateralPair(2,3,X)});
            for(int k=0;k<600;k++){
                for(int i=0;i<4;i++){f[i]=Frame(k*.01+i*.15);basis[i]=new Vec3(i%2==0?-1:1,Math.Sin(k*.01+i)*.04,k*.001);p[i]=basis[i];}
                p[0]+=f[0].Multiply(new Vec3(-.1,0,.1));p[2]+=f[2].Multiply(new Vec3(-.2,.07,.05));
                s.BeginStep(basis,f,new[]{1.0,1.0,1.0,1.0},1.0/60,0);s.Project(p);Require(s.MaximumResidual<1e-12,"Animated residual");
                Require(((p[0]-basis[0])-(p[2]-basis[2])).Length>.01,"Separate seam tracks collapsed");
            }
            Array.Copy(basis,p,4);s.BeginStep(basis,f,new[]{1.0,1.0,1.0,1.0},1.0/60,0);s.Project(p);for(int i=0;i<4;i++)Near(p[i],basis[i],"Old deformation accumulated",0);
        });
        test("Bilateral singular frames are reported without corrupting other pairs",()=>{
            var basis=new[]{new Vec3(-1,0,0),new Vec3(1,0,0),new Vec3(-1,1,0),new Vec3(1,1,0)};var p=(Vec3[])basis.Clone();p[0]+=new Vec3(0,0,.2);p[2]+=new Vec3(0,0,.1);
            var s=new BilateralDisplacementConstraint(4,new[]{new BilateralPair(0,1,X),new BilateralPair(2,3,X)});s.BeginStep(basis,new[]{new Linear3(),I,I,I},new[]{1.0,1.0,1.0,1.0},.01,0);s.Project(p);
            Require(s.SingularPairs==1&&s.ActivePairs==1,"Singular diagnostic");Near(p[0]-basis[0],new Vec3(0,0,.2),"Singular pair was changed",0);Near(p[2]-basis[2],p[3]-basis[3],"Valid pair was skipped");foreach(var v in p)Require(v.IsFinite,"Nonfinite output");
        });
        test("Bilateral bindings reject implicit seam merging and invalid input before projection",()=>{
            int rejected=0;try{new BilateralDisplacementConstraint(3,new[]{new BilateralPair(0,1,X),new BilateralPair(0,2,X)});}catch(ArgumentException){rejected++;}
            try{new BilateralDisplacementConstraint(2,new[]{new BilateralPair(0,1,new Vec3())});}catch(ArgumentException){rejected++;}
            var s=Pair();var a=new[]{new Vec3(-1,0,0),new Vec3(1,0,0)};try{s.Project(a);}catch(InvalidOperationException){rejected++;}
            try{s.BeginStep(a,null,new[]{1.0,1.0},double.NaN,0);}catch(ArgumentException){rejected++;}
            try{s.BeginStep(a,null,new[]{1.0,-1.0},.01,0);}catch(ArgumentException){rejected++;}
            Require(rejected==5,"Invalid input was accepted");
        });
        test("Bilateral finite inputs that overflow a solve matrix cannot publish NaN",()=>{
            var basis=new[]{new Vec3(-1,0,0),new Vec3(1,0,0)};var p=(Vec3[])basis.Clone();p[0]+=new Vec3(0,0,.1);var before=(Vec3[])p.Clone();var s=Pair();
            s.BeginStep(basis,null,new[]{double.MaxValue,double.MaxValue},.01,0);s.Project(p);Require(s.SingularPairs==1&&s.ActivePairs==0,"Overflowed system was enabled");for(int i=0;i<p.Length;i++)Near(p[i],before[i],"Overflowed system changed positions",0);
            bool rejected=false;try{new BilateralDisplacementConstraint(2,new[]{new BilateralPair(0,1,new Vec3(double.MaxValue,1,0))});}catch(ArgumentException){rejected=true;}Require(rejected,"Unnormalizable mirror plane accepted");
        });
        test("Bilateral reset invalidates the old animated baseline and cached multipliers",()=>{
            var basis=new[]{new Vec3(-1,0,0),new Vec3(1,0,0)};var p=(Vec3[])basis.Clone();var s=Pair();s.BeginStep(basis,null,new[]{1.0,1.0},.01,.001);p[0]+=new Vec3(0,0,.1);s.Project(p);Require(s.MaximumResidual>0,"Soft test has no residual");s.Reset();
            Require(s.MaximumResidual==0&&s.ProjectionPasses==0&&s.ActivePairs==0,"Reset left stale diagnostics");bool rejected=false;try{s.Project(p);}catch(InvalidOperationException){rejected=true;}Require(rejected,"Old animated baseline remained usable after reset");
            Array.Copy(basis,p,2);s.BeginStep(basis,null,new[]{1.0,1.0},.01,0);s.Project(p);for(int i=0;i<2;i++)Near(p[i],basis[i],"Old multiplier affected new baseline",0);
        });
        test("Bilateral cached projection allocates zero managed bytes per animated step",()=>{
            var a=new[]{new Vec3(-1,0,0),new Vec3(1,0,0)};var p=new Vec3[2];var frames=new[]{I,I};var mass=new[]{1.0,1.0};var s=Pair();
            for(int k=0;k<30;k++){Array.Copy(a,p,2);s.BeginStep(a,frames,mass,.01,0);s.Project(p);s.MeasureResidual(p);}long before=GC.GetAllocatedBytesForCurrentThread();
            for(int k=0;k<1000;k++){Array.Copy(a,p,2);p[0]+=new Vec3(0,0,.01);s.BeginStep(a,frames,mass,.01,0);s.Project(p);s.MeasureResidual(p);}Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Per-step symmetry allocation");
        });
        test("Optional bilateral constraint participates in the existing soft body solver",()=>{
            var m=PrimitiveMeshes.Grid(2,2,1,1);var solver=new SoftBodySolver(new SimulationTopology(m.Vertices,m.Triangles),new double[m.Vertices.Length]);
            solver.Symmetry=new BilateralDisplacementConstraint(m.Vertices.Length,new[]{new BilateralPair(0,2,X)});solver.GuideTargets=(Vec3[])m.Vertices.Clone();solver.GuideTargetEnabled=new bool[m.Vertices.Length];solver.GuideTargetEnabled[0]=true;solver.GuideTargets[0]+=new Vec3(0,0,.08);
            solver.Advance(m.Vertices,null,new SoftBodySettings{ShapeCompliance=1e4,StretchCompliance=1e4,BendCompliance=1e4,MaximumSpeedRatio=100},PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,1.4);
            Require(solver.Symmetry.ProjectionPasses>0,"Symmetry was never executed");Require(solver.SolvedDisplacement(0).Z>.01&&solver.SolvedDisplacement(2).Z>.01,"Actual positions did not move");Require(solver.Symmetry.MaximumResidual<1e-10,"Final symmetry residual was not measured");
        });
    }
}
