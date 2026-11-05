using System;
using BetterPenetrationNext.Core;

static class ReceiverLandmarkTests
{
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    static void Pipe(out SimulationDefinition definition,out ReactiveMaterialChart chart)
    {
        const int rings=17,sides=24;var vertices=new Vec3[rings*sides];var faces=new int[(rings-1)*sides*6];var u=new double[vertices.Length];var angle=new double[vertices.Length];int f=0;
        for(int r=0;r<rings;r++)for(int s=0;s<sides;s++){int i=r*sides+s;u[i]=r/(double)(rings-1);angle[i]=s/(double)sides;double t=angle[i]*Math.PI*2;vertices[i]=new Vec3(.2*Math.Cos(t),.1*Math.Sin(t),u[i]*2);}
        for(int r=0;r<rings-1;r++)for(int s=0;s<sides;s++){int a=r*sides+s,b=r*sides+(s+1)%sides,c=a+sides,d=b+sides;faces[f++]=a;faces[f++]=b;faces[f++]=c;faces[f++]=b;faces[f++]=d;faces[f++]=c;}
        definition=new SimulationDefinition{Rest=vertices,ConfinementTriangles=faces};chart=new ReactiveMaterialChart{SupportAxial=u,SupportAngle=angle};
    }
    static Vec3[] Guide(){return new[]{new Vec3(),new Vec3(0,0,1),new Vec3(0,0,2)};}
    public static void Run(Action<string,Action> test)
    {
        test("Canal-facing terminal selects the front throat rather than a small internal boundary",()=>{
            const int sides=32;double[] z={0,.001,.01,.03},r={.006,.0013,.0013,.0018};var p=new Vec3[sides*z.Length];var faces=new int[(z.Length-1)*sides*6];var regions=new int[faces.Length/3];int t=0;
            for(int s=0;s<z.Length;s++)for(int k=0;k<sides;k++){double a=k*Math.PI*2/sides;p[s*sides+k]=new Vec3(r[s]*Math.Cos(a),r[s]*Math.Sin(a),z[s]);}
            for(int s=0;s<z.Length-1;s++)for(int k=0;k<sides;k++){int a=s*sides+k,b=s*sides+(k+1)%sides,c=a+sides,d=b+sides;faces[t++]=a;faces[t++]=b;faces[t++]=c;faces[t++]=b;faces[t++]=d;faces[t++]=c;}for(int i=0;i<regions.Length;i++)regions[i]=3;
            var rn=new CervicalOpeningLandmark(p,faces,regions,new Vec3(0,0,1));rn.Update(p,new Vec3(0,0,1),new Vec3(1,0,0));
            Require(rn.IsCanalFacingThroat&&rn.SurfacePointCount==32&&Math.Abs(rn.Centre.Z-.001)<1e-12,"Terminal targeted the deep internal os");
            rn.Update(p,new Vec3(0,0,-1),new Vec3(1,0,0));Require(rn.Normal.Z>.999,"A changing guide reversed the anatomical normal");
            for(int k=1;k<5;k++)Require(rn.PointIndices[k]>=32&&rn.PointIndices[k]<64,"Terminal point left its bound material ring");
        });
        test("Terminal Rn follows the actual opening rim and its animated orientation",()=>{
            var p=new[]{new Vec3(-1,-1,1),new Vec3(1,-1,1),new Vec3(1,1,1),new Vec3(-1,1,1),new Vec3(-.2,-.2,1),new Vec3(.2,-.2,1),new Vec3(.2,.2,1),new Vec3(-.2,.2,1)};
            var f=new[]{0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7};var regions=new int[8];for(int i=0;i<8;i++)regions[i]=3;
            var rn=new CervicalOpeningLandmark(p,f,regions);Require(rn.Available,"Real opening was not bound");rn.Update(p,new Vec3(0,0,1),new Vec3(1,0,0));
            Require((rn.Centre-new Vec3(0,0,1)).Length<1e-12&&rn.Normal.Z>.999,"Wrong opening frame");
            var pose=new Vec3[p.Length];for(int i=0;i<p.Length;i++)pose[i]=new Vec3(p[i].X,-p[i].Z,p[i].Y)+new Vec3(3,4,5);
            rn.Update(pose,new Vec3(0,-1,0),new Vec3(1,0,0));Require((rn.Centre-new Vec3(3,3,5)).Length<1e-12&&rn.Normal.Y<-.999,"Animated Rn was not carried by its real vertices");
            for(int k=1;k<=4;k++){bool found=false;for(int i=4;i<8;i++)found|=(rn.Points[k]-pose[i]).Length<1e-12;Require(found,"Rn surface point left its actual rim");}
            for(int i=0;i<20;i++)rn.Update(pose,new Vec3(0,-1,0),new Vec3(1,0,0));long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<30;i++)rn.Update(pose,new Vec3(0,-1,0),new Vec3(1,0,0));Require(GC.GetAllocatedBytesForCurrentThread()==before,"Rn rebuilt buffers during animation");
        });
        test("Terminal landmark does not invent an opening on a solid patch",()=>{
            var p=new[]{new Vec3(-1,-1,1),new Vec3(1,-1,1),new Vec3(0,1,1)};
            Require(!new CervicalOpeningLandmark(p,new[]{0,1,2},new[]{3}).Available,"Unverified rim was labelled an opening");
        });
        test("One-sided receiver contact leaves the opposite wall at its baseline radius",()=>{
            var field=new ReceiverRadialField(3);var baseline=new[]{.01,.01,.01};var path=Guide();field.Begin(baseline);
            for(int s=0;s<3;s++)field.Include(s,new Vec3(0,.03,0),new Vec3(1,0,0),new Vec3(0,1,0),0);
            field.Update(path,.02,12);
            Require(field.Radius(0,.5,Math.PI*.5)>=.03,"Loaded wall failed to cover the contact");
            Require(Math.Abs(field.Radius(0,.5,Math.PI*1.5)-.01)<1e-12,"Opposite wall expanded under a remote load");
        });
        test("Directional wall profiles are continuous across their material seam",()=>{
            var field=new ReceiverRadialField(3);field.Begin(new[]{.01,.01,.01});
            for(int s=0;s<3;s++)field.Include(s,new Vec3(.03,-.000001,0),new Vec3(1,0,0),new Vec3(0,1,0),.0001);
            field.Update(Guide(),.02,12);
            Require(Math.Abs(field.Radius(0,.5,1e-9)-field.Radius(0,.5,2*Math.PI-1e-9))<1e-9,"Angular seam is discontinuous");
            Require(field.Radius(0,.5,-.00001)>=.03,"Contact between angular samples was lost");
        });
        test("Warm directional wall updates reuse buffers",()=>{
            var field=new ReceiverRadialField(3);var baseline=new[]{.01,.01,.01};var path=Guide();var x=new Vec3(1,0,0);var y=new Vec3(0,1,0);
            for(int i=0;i<30;i++){field.Begin(baseline);field.Include(1,new Vec3(0,.03,0),x,y,0);field.Update(path,.02,12);}
            long before=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<30;i++){field.Begin(baseline);field.Include(1,new Vec3(0,.03,0),x,y,0);field.Update(path,.02,12);}
            Require(GC.GetAllocatedBytesForCurrentThread()==before,"Directional wall update allocated buffers");
        });
        test("Receiver R0 and axial sections use exact triangle-bound surface points",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var m=new ReceiverAdaptiveLandmarks(d,c,3,null);m.Update(d.Rest,Guide(),new[]{0.0,.5,1.0},.16,null);var mesh=new TriangleSurface(d.Rest,d.ConfinementTriangles,SurfaceKind.OrientedShell);
            Require(m.Count==14,"Source step was ignored");for(int i=0;i<m.Count;i++){Require((m.Points[i*5]-new Vec3(0,0,m.Arc[i])).Length<1e-10,"Virtual centre left the real sections");for(int k=1;k<=4;k++){SurfaceSample p;Require(mesh.TrySample(m.Points[i*5+k],.001,out p)&&(p.Point-m.Points[i*5+k]).Length<1e-10,"Landmark left original surface");}}
        });
        test("Receiver landmarks follow animated and independently indexed geometry without recutting",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var m=new ReceiverAdaptiveLandmarks(d,c,3,null);int before=m.ReferenceTriangleTests;var pose=(Vec3[])d.Rest.Clone();for(int i=0;i<pose.Length;i++)pose[i]+=new Vec3(12,3,-4);var guide=Guide();for(int i=0;i<guide.Length;i++)guide[i]+=new Vec3(12,3,-4);m.Update(pose,guide,new[]{0.0,.5,1.0},.2,null);Require(m.ReferenceTriangleTests==before,"Animated mesh was recut");Require((m.Points[0]-new Vec3(12,3,-4)).Length<1e-10,"Animated centre stale");
        });
        test("Receiver and Source sections pair by current 3D positions",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var m=new ReceiverAdaptiveLandmarks(d,c,3,null);m.Update(d.Rest,Guide(),new[]{0.0,.5,1.0},.2,new[]{new Vec3(.05,0,-1),new Vec3(.05,0,3)});for(int i=0;i<m.Count;i++)Require((m.SourcePairs[i]-m.Points[i*5]-new Vec3(.05,0,0)).Length<1e-10,"Spatial matching failed");
        });
        test("Active Receiver never spends RecoveryTime, including long low-FPS contact",()=>{
            var p=new ReceiverContactPhase();for(int i=0;i<300;i++){p.Advance(true,.065);Require(p.Active&&p.ReleasedSeconds==0&&p.Weight(4.5)==1,"Recovery ran during active contact");}p.Advance(false,.065);Require(p.BeganRelease&&p.Weight(4.5)==1,"Release snapped");for(int i=0;i<90;i++)p.Advance(false,.05);Require(p.Weight(4.5)<1e-12,"Recovery did not finish");
        });
        test("Repeat admission interrupts inactive recovery immediately",()=>{
            var p=new ReceiverContactPhase();p.Advance(true,.02);p.Advance(false,.02);p.Advance(false,2);Require(p.Weight(4.5)>0&&p.Weight(4.5)<1,"No actual recovery");p.Advance(true,.02);Require(p.Weight(4.5)==1&&p.ReleasedSeconds==0,"Recontact was delayed by recovery");
        });
        test("Cervical follow is compression-only with explicit configurable travel",()=>{
            var n=new Vec3(0,0,-1);Require(CervicalFollowConstraint.Target(.08,n,1,.025).Z==.025,"Travel cap ignored");Require(CervicalFollowConstraint.Target(-.02,n,1,.025).Length==0,"Tip acquired a tensile attachment");Require(Math.Abs(CervicalFollowConstraint.Target(.02,n,.5,.025).Z-.01)<1e-12,"Mobility ignored");
        });
        test("Anatomical joint keeps oblique compression on its own depth axis",()=>{
            Vec3 axis=new Vec3(0,0,1),normal=new Vec3(0,-.6,-.8);
            Vec3 d=CervicalFollowConstraint.AlongAxis(.012,normal,axis,1,.04,new Vec3(.1,.2,.005));
            Require(Math.Abs(d.Z-.02)<1e-12&&d.X==0&&d.Y==0,"Contact normal dragged the anatomical joint sideways");
            Require(Math.Abs(-Vec3.Dot(d-axis*.005,normal)-.012)<1e-12,"Oblique joint Jacobian did not remove the specified compression");
            Require(CervicalFollowConstraint.AlongAxis(.1,normal,axis,1,.025,d).Length<=.0250000001,"Anatomical travel limit ignored");
        });
        test("Side contact and withdrawal do not create depth-axis traction",()=>{
            Vec3 axis=new Vec3(0,0,1),current=axis*.01;
            Require((CervicalFollowConstraint.AlongAxis(.02,new Vec3(1,0,0),axis,1,.04,current)-current).Length<1e-12,"Tangential load invented joint travel");
            Require((CervicalFollowConstraint.AlongAxis(-.02,-axis,axis,1,.04,current)-current).Length<1e-12,"Withdrawal acquired tensile joint travel");
            Require(CervicalFollowConstraint.AlongAxis(.02,-axis,axis,1,0,current).Length==0,"Zero travel did not lock the joint");
        });
        test("Continuous receiver radius field has no overshoot or section-edge steps",()=>{
            var f=new ReceiverSectionField(5);var p=new[]{new Vec3(),new Vec3(0,0,1),new Vec3(0,0,2),new Vec3(0,0,3),new Vec3(0,0,4)};f.Update(p,new[]{.1,.1,.3,.1,.1});for(int i=0;i<4;i++){Require(Math.Abs(f.Radius(i,1)-f.Radius(i+1==4?i:i+1,i+1==4?1:0))<1e-12,"Section discontinuity");for(int j=0;j<=40;j++)Require(f.Radius(i,j/40.0)>=.1&&f.Radius(i,j/40.0)<=.300001,"Radius overshoot");}
        });
        test("Warm Receiver landmarks reuse all buffers with source-sized sampling",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var m=new ReceiverAdaptiveLandmarks(d,c,3,null);var guide=Guide();var coordinates=new[]{0.0,.5,1.0};for(int i=0;i<20;i++)m.Update(d.Rest,guide,coordinates,.08,null);long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++)m.Update(d.Rest,guide,coordinates,.08,null);Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Receiver landmarks allocated per frame");
        });
        test("R0 guidance rotates the complete source about a fixed attachment",()=>{
            var aim=new EntryTargetGuidance();var direction=new Vec3(.2,0,1).Normalized;for(int i=0;i<120;i++)aim.Advance(new Vec3(0,0,1),direction,1.0/60,.4,1,12);
            Vec3 pivot=new Vec3(1,2,3),a=new Vec3(.05,0,.2),b=new Vec3(-.05,0,.2);Vec3 pa=pivot+EntryTargetGuidance.Rotate(a,aim.RotationVector),pb=pivot+EntryTargetGuidance.Rotate(b,aim.RotationVector);
            Require((EntryTargetGuidance.Rotate(new Vec3(),aim.RotationVector)).Length==0,"Attachment translated");Require(Math.Abs((pa-pb).Length-(a-b).Length)<1e-12,"Whole-rig aim changed width");Require((EntryTargetGuidance.Rotate(new Vec3(0,0,1),aim.RotationVector)-direction).Length<1e-8,"Rig did not face R0");
        });
        test("Oriented R0 favours its live axis while retaining a feasible aperture hit",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var indices=new int[24];for(int i=0;i<24;i++)indices[i]=i;
            var entry=new OrientedEntryFrame(24);entry.Update(d.Rest,indices,new Vec3(),new Vec3(0,0,.2),new Vec3(1,0,0));
            Require((entry.DesiredDirection(new Vec3(.05,0,-1))-new Vec3(0,0,1)).Length<1e-12,"Feasible normal alignment ignored");
            var pivot=new Vec3(2,0,-1);var direction=entry.DesiredDirection(pivot);var hit=pivot+direction/ direction.Z;Require(entry.Contains(hit),"Fixed-base aim missed the actual aperture");
            Vec3 shift=new Vec3(5,-3,4),rotation=new Vec3(.2,.5,.1);var pose=(Vec3[])d.Rest.Clone();for(int i=0;i<pose.Length;i++)pose[i]=EntryTargetGuidance.Rotate(pose[i],rotation)+shift;
            entry.Update(pose,indices,shift,EntryTargetGuidance.Rotate(new Vec3(0,0,.2),rotation)+shift,EntryTargetGuidance.Rotate(new Vec3(1,0,0),rotation));Require((entry.Normal-EntryTargetGuidance.Rotate(new Vec3(0,0,1),rotation)).Length<1e-12,"R0 did not follow the animated receiver");
        });
        test("R0 admission rejects distant half-space overlap and accepts a real open-shell crossing",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var indices=new int[24];for(int i=0;i<24;i++)indices[i]=i;var entry=new OrientedEntryFrame(24);entry.Update(d.Rest,indices,new Vec3(),new Vec3(0,0,1),new Vec3(1,0,0));
            var source=new AdaptiveMeshLandmarks(d.Rest,d.ConfinementTriangles,new Vec3(0,0,1));var pose=(Vec3[])d.Rest.Clone();for(int i=0;i<pose.Length;i++)pose[i]+=new Vec3(0,0,-1);source.Update(pose,new Vec3(1,0,0));Require(entry.Intersects(source),"Actual open-shell crossing missed");
            for(int i=0;i<pose.Length;i++)pose[i]+=new Vec3(10,0,0);source.Update(pose,new Vec3(1,0,0));Require(!entry.Intersects(source),"Remote mesh activated the entrance");
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++){entry.Update(d.Rest,indices,new Vec3(),new Vec3(0,0,1),new Vec3(1,0,0));entry.Intersects(source);}Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Oriented entry allocated each frame");
        });
        test("R0 aim keeps an explicit angular speed and displacement-independent pose budget",()=>{
            var aim=new EntryTargetGuidance();double previous=0;for(int i=0;i<60;i++){aim.Advance(new Vec3(0,0,1),new Vec3(1,0,0),1.0/60,.25,.5,12);Require(aim.RotationVector.Length<=.250000001&&aim.RotationVector.Length-previous<=.5/60+1e-10,"Abrupt or excessive rig turn");previous=aim.RotationVector.Length;}
            aim.Reset();Require(aim.RotationVector.Length==0,"Aim left a residual rotation");
        });
        test("Shared surface feasibility retains one amplitude instead of making isolated spikes",()=>{
            var baseline=new[]{new Vec3(),new Vec3(1,0,0),new Vec3(0,1,0)};var target=new[]{new Vec3(),new Vec3(4,0,0),new Vec3(0,4,0)};
            var topology=new SimulationTopology(baseline,new[]{0,1,2},true);var lengths=new double[topology.Stretch.Length];for(int i=0;i<lengths.Length;i++)lengths[i]=(baseline[topology.Stretch[i].A]-baseline[topology.Stretch[i].B]).Length;
            double blend=CoherentSurfaceLimiter.Project(baseline,target,topology.Stretch,lengths,new[]{0,1,2},.25);Require(blend>0&&blend<1,"No meaningful constraint correction");Require(Math.Abs(target[1].X-target[2].Y)<1e-12,"Independent vertex corrections fractured the field");Require(target[1].X<=1.2500001,"Shared strain budget exceeded");
        });
        test("Receiver landmarks resample an elongating distal attachment at the same Source step",()=>{
            SimulationDefinition d;ReactiveMaterialChart c;Pipe(out d,out c);var m=new ReceiverAdaptiveLandmarks(d,c,3,null);var coordinates=new[]{0.0,.5,1.0};m.Update(d.Rest,Guide(),coordinates,.16,null);int before=m.Count;var world=(Vec3[])d.Rest.Clone();for(int i=0;i<world.Length;i++)world[i]+=new Vec3(0,0,c.SupportAxial[i]*.5);var guide=new[]{new Vec3(),new Vec3(0,0,1.25),new Vec3(0,0,2.5)};m.Update(world,guide,coordinates,.16,null);Require(m.Count>before&&m.Spacing==.16,"Extension did not add Source-sized sections");Require(m.Points[0].Length<1e-12&&Math.Abs(m.Points[(m.Count-1)*5].Z-2.5)<1e-12,"Entry or distal attachment was lost");
        });
        test("Shared feasibility allows compression at zero tensile strain and catches an interior quadratic loss",()=>{
            Require(CoherentSurfaceLimiter.PositivePrefix(-.25,1,0)==1,"A safe zero-strain compression was blocked");Require(Math.Abs(CoherentSurfaceLimiter.PositivePrefix(1,-1,.16)-.2)<1e-12,"An intermediate infeasible interval was skipped");
        });
        test("Worker receiver surface survives independent changes in the current animation baseline",()=>{
            var identity=new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1));var solved=new Vec3(.02,.03,.04);var oldOffset=new Vec3(.01,0,0);var newNative=new Vec3(.1,-.2,.3);var shift=new Vec3(10,2,-4);
            Vec3 offset=ReceiverPoseRebase.Offset(solved,oldOffset,newNative,identity,new Vec3(),identity,shift,1);Require((newNative+offset-solved-shift).Length<1e-12,"A new native bone pose contaminated the solved receiver surface");
        });
        test("Receiver pose ownership preserves native motion in unowned neighbouring tissues",()=>{
            var identity=new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1));var correction=new Vec3(.01,0,0);var newNative=new Vec3(.1,-.2,.3);
            Require((ReceiverPoseRebase.Offset(new Vec3(),correction,newNative,identity,new Vec3(),identity,new Vec3(10,2,-4),0)-correction).Length<1e-12,"Unowned tissue was replaced with a previous-frame pose");
        });
    }
}

