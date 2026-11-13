using System;
using BetterPenetrationNext.Core;

static class ClosedEntranceTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    sealed class Fixture
    {
        public SimulationDefinition Support,Physical;
        public ClosedEntranceDefinition Definition;
        public ReceiverGuideDefinition Guide;
        public Vec3[] Native,Positions;
        public readonly int[] Ring={0,1,2,3};
        public Fixture()
        {
            Native=new Vec3[18];var faces=new int[48];
            for(int k=0;k<4;k++)
            {
                double a=k*Math.PI*.5;Native[k]=new Vec3(Math.Cos(a)*.0005,Math.Sin(a)*.02,0);Native[k+4]=new Vec3(Math.Cos(a)*.0015,Math.Sin(a)*.024,0);Native[k+8]=new Vec3(Math.Cos(a)*.07,Math.Sin(a)*.08,0);Native[k+12]=Native[k+8];
                int next=(k+1)%4,j=k*12;faces[j]=k;faces[j+1]=k+4;faces[j+2]=next+4;faces[j+3]=k;faces[j+4]=next+4;faces[j+5]=next;faces[j+6]=k+4;faces[j+7]=k+8;faces[j+8]=next+8;faces[j+9]=k+4;faces[j+10]=next+8;faces[j+11]=next+4;
            }
            Native[16]=Native[4];Native[17]=Native[4];
            Support=new SimulationDefinition{Rest=(Vec3[])Native.Clone(),BoneNames=new[]{"a","b"},InfluenceOffsets=new int[19],BoneIndices=new int[18],BoneWeights=new double[18]};
            for(int i=0;i<18;i++){Support.InfluenceOffsets[i]=i;Support.BoneWeights[i]=1;}Support.InfluenceOffsets[18]=18;Support.BoneIndices[16]=1;
            var morph=new SimulationMorph{Name="independent",Deltas=new Vec3[18]};morph.Deltas[17]=new Vec3(0,0,.01);Support.Morphs=new[]{morph};
            Positions=new Vec3[8];for(int k=0;k<4;k++){double a=k*Math.PI*.5;Positions[k]=new Vec3(Math.Cos(a)*.025,Math.Sin(a)*.025,0);Positions[k+4]=Positions[k]+new Vec3(0,0,.01);}
            Physical=new SimulationDefinition{Rest=(Vec3[])Positions.Clone()};
            Guide=new ReceiverGuideDefinition{Boundaries=new[]{Ring,new[]{4,5,6,7}},Contours=new[]{new ReceiverContour{Vertices=Ring,Weights=new[]{.25,.25,.25,.25}},new ReceiverContour{Vertices=new[]{4,5,6,7},Weights=new[]{.25,.25,.25,.25}}}};
            Definition=new ClosedEntranceDefinition{Schema=1,Controls=new int[18],Boundary=Ring,AnchorA=new int[18],AnchorB=new int[18],AnchorT=new double[18],Distance=new double[18],Influence=new double[18],Triangles=faces,RingA=Ring,RingB=Ring,RingT=new double[4]};
            for(int i=0;i<18;i++){Definition.Controls[i]=i;Definition.AnchorA[i]=Definition.AnchorB[i]=i%4;Definition.Influence[i]=i<4?1:i<8?.75:0;}Definition.Influence[16]=Definition.Influence[17]=.2;
        }
        public ClosedEntranceConstraint Solver(){return new ClosedEntranceConstraint(Definition,Support,Physical,Guide);}
        public void Step(ClosedEntranceConstraint solver,bool active=true,double dt=1.0/60,double[] profile=null){solver.Apply(Native,Positions,Ring,new Vec3(0,0,1),new Vec3(1,0,0),active,dt,5.4,4.5,4,1,profile);}
    }
    public static void Run(Action<string,Action> test)
    {
        test("Closed material boundary opens fully by repairing free neighbours instead of halving the aperture",()=>{
            var f=new Fixture();var s=f.Solver();for(int i=0;i<120;i++)f.Step(s);Check(s.AppliedScale>.99,"Whole aperture was reduced by a free triangle");Check(Math.Abs((f.Native[0]+s.Offsets[0]).X-.025)<1e-8,"Contact radius not attained");Check(s.MinimumAreaRatio>=.03&&s.MaximumStrain<=4.00001,"Material guard was bypassed");for(int i=8;i<16;i++)Check(s.Offsets[i].Length==0,"Unloaded outer skin moved");
        });
        test("Closed material recovery finishes at 4.5 seconds and repeated contact starts from the current shape",()=>{
            var f=new Fixture();var s=f.Solver();for(int i=0;i<100;i++)f.Step(s);for(int i=0;i<60;i++)f.Step(s,false);Vec3 previous=s.Offsets[0];f.Step(s,true,1e-7);Check((previous-s.Offsets[0]).Length<1e-6,"Repeated contact snapped");for(int i=0;i<271;i++)f.Step(s,false);foreach(var q in s.Offsets)Check(q.Length<1e-10,"Closure did not restore its native baseline");
        });
        test("Closed boundary uses independent angular contact demand rather than a maximum-radius circle",()=>{
            var f=new Fixture();var s=f.Solver();var radii=new double[16];for(int i=0;i<16;i++)radii[i]=.015+.01*Math.Max(0,Math.Cos(i*2*Math.PI/16));for(int i=0;i<120;i++)f.Step(s,true,1.0/60,radii);Check(Math.Abs((f.Native[0]+s.Offsets[0]).X-.025)<1e-8,"Loaded side did not match profile");Check(Math.Abs((f.Native[2]+s.Offsets[2]).X+.015)<1e-8,"Opposite side inherited maximum load");
        });
        test("Closed surface retains coincident nodes with different bone weights or morph deltas",()=>{
            var f=new Fixture();var s=f.Solver();for(int i=0;i<100;i++)f.Step(s);Check((s.Offsets[4]-s.Offsets[16]).Length>.005,"Different bone weights were coupled");Check((s.Offsets[4]-s.Offsets[17]).Length>.005,"Different morph identities were coupled");
        });
        test("Closed boundary hot loop reuses cached topology and all correction buffers",()=>{
            var f=new Fixture();var s=f.Solver();for(int i=0;i<120;i++)f.Step(s);long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++)f.Step(s);Check(GC.GetAllocatedBytesForCurrentThread()-before==0,"Contact repair allocated");
        });
        test("Entrance contact proxy remains finite and preserves independent angular loads",()=>{
            var p=new EntranceContactProxy{Active=true,Depth=.03,Centre=new Vec3(),Axis=new Vec3(0,0,1),Side=new Vec3(1,0,0)};for(int s=0;s<EntranceContactProxy.Slices;s++)for(int k=0;k<16;k++)p.Radii[s*16+k]=k<8?.025:.015;
            Check(p.Correction(new Vec3(.01,0,-.01),1).X>.014,"Contact proxy omitted the near surface");Check(p.Radius(-.031,0)==0&&p.Radius(.01,0)==0,"Open proxy was extrapolated past its sampled ends");p.Active=false;Check(p.Correction(new Vec3(.001,0,0),1).Length==0,"Inactive proxy changed material");
        });
        test("Finite contact profiles leave empty cuts empty before any longitudinal field smoothing",()=>{
            var field=new ReceiverRadialField(3);field.Begin(new double[3]);field.Include(1,new Vec3(.025,0,0),new Vec3(1,0,0),new Vec3(0,1,0),0);field.Update(new[]{new Vec3(),new Vec3(0,0,.01),new Vec3(0,0,.02)},1.0/60,12,true);
            Check(field.ContactRadius(0,0)==0&&field.ContactRadius(2,0)==0&&field.ContactRadius(1,0)>.025,"Raw finite cuts gained an artificial cap");
        });
        test("Contact proxy opens material behind the rim and retains area, pins and an allocation-free step",()=>{
            var f=new Fixture();var s=f.Solver();var p=new EntranceContactProxy{Active=true,Depth=.03,Centre=new Vec3(),Axis=new Vec3(0,0,1),Side=new Vec3(1,0,0)};for(int i=0;i<p.Radii.Length;i++)p.Radii[i]=.025;
            Action step=()=>s.Apply(f.Native,f.Positions,f.Ring,new Vec3(0,0,1),new Vec3(1,0,0),true,1.0/60,5.4,4.5,4,1,null,0,null,p);for(int i=0;i<120;i++)step();Check((f.Native[4]+s.Offsets[4]).Length>.0249,"Fold behind the boundary still blocks contact");Check(s.MinimumAreaRatio>=.03&&s.MaximumStrain<=4.00001,"Proxy bypassed material checks");for(int i=8;i<16;i++)Check(s.Offsets[i].Length==0,"Proxy moved outer pins");long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++)step();Check(GC.GetAllocatedBytesForCurrentThread()-before==0,"Proxy hot path allocated");
        });
    }
}
