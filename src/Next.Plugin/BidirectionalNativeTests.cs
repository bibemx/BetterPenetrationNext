using System;
using System.Collections;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        IEnumerator RunBidirectional()
        {
            var grid=PrimitiveMeshes.Grid(14,14,1,1); var receiver=Skin("Neutral coupled receiver",grid,Vector3.zero);
            var original=receiver.sharedMesh; var material=receiver.sharedMaterial; var d=Definition(original,receiver,grid);
            var delta=new Vector3[grid.Vertices.Length]; var zero=new Vector3[delta.Length]; var coreDelta=new Vec3[delta.Length];
            for(int i=0;i<delta.Length;i++) { delta[i]=new Vector3(0,0,.008f*(float)Math.Cos(grid.Vertices[i].X*4)); coreDelta[i]=AnimationAdapter.ToCore(delta[i]); }
            original.AddBlendShapeFrame("Laboratory receiver shape",100,delta,zero,zero); d.Morphs=new[]{new SimulationMorph { Name="Laboratory receiver shape",Deltas=coreDelta }};
            var probe=Skin("Neutral coupled probe",PrimitiveMeshes.Sphere(32,16,.16),new Vector3(0,0,.12f)); var sourceOriginal=probe.sharedMesh; var sourceMaterial=probe.sharedMaterial;
            var sd=new Vector3[sourceOriginal.vertexCount]; var sz=new Vector3[sd.Length]; for(int i=0;i<sd.Length;i++) sd[i]=new Vector3(.002f,0,.004f); sourceOriginal.AddBlendShapeFrame("Laboratory source shape",100,sd,sz,sz);
            var evidence=new double[sd.Length]; for(int i=0;i<evidence.Length;i++) evidence[i]=1;
            using(var geometry=new ContactDetection(probe,evidence)) using(var target=new MeshDeformationBinding(receiver,d)) {
                var animation=new AnimationAdapter(d,receiver); var solver=new SoftBodySolver(new SimulationTopology(grid.Vertices,grid.Triangles),d.PinWeights);
                var body=new SourceContactBody { Surface=geometry }; var settings=new BidirectionalContactSettings { TissueCompliance=0,MaxSourceDisplacement=.2,SourceCorrectionStrength=.1 }; var tissue=new SoftBodySettings { MaximumOffsetRatio=.2 };
                double sourceError=0,receiverError=0,maximumTissue=0,maximumSource=0; var times=new double[90]; var baked=new Mesh(); assets.Add(baked);
                for(int f=0;f<90;f++) {
                    receiver.bones[0].localRotation=Quaternion.Euler(0,Mathf.Sin(f*.07f)*3,0); receiver.SetBlendShapeWeight(0,f%100); probe.bones[0].localRotation=Quaternion.Euler(0,f*.1f,0); probe.SetBlendShapeWeight(0,(f*2)%100);
                    var watch=System.Diagnostics.Stopwatch.StartNew(); geometry.Capture(true); animation.Capture(solver); body.BeginFrame(settings,1.0/60);
                    solver.Advance(animation.Anchors,body,tissue,PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,animation.PosedSize,settings);
                    target.Apply(solver,.2*animation.PosedSize,1); geometry.ApplyCorrection(body); watch.Stop(); times[f]=watch.Elapsed.TotalMilliseconds;
                    probe.BakeMesh(baked); var actual=baked.vertices;
                    for(int i=0;i<actual.Length;i++) sourceError=Math.Max(sourceError,(AnimationAdapter.ToCore(AnimationAdapter.BakedToWorld(probe).MultiplyPoint3x4(actual[i]))-body.ToCorrected(geometry.AnimatedVertices[i])).Length);
                    receiverError=Math.Max(receiverError,target.VerifyAppliedPose(solver));
                    for(int i=0;i<solver.Positions.Length;i++) maximumTissue=Math.Max(maximumTissue,solver.SolvedDisplacement(i).Length); maximumSource=Math.Max(maximumSource,body.Offset.Length);
                    yield return null;
                }
                Require(maximumTissue>.005&&maximumSource>.001,"1.0.4 native animated pair applies actual corrections to both sides","tissue="+maximumTissue+" source="+maximumSource);
                Require(sourceError<.0001,"1.0.4 source inverse skinning matches native BakeMesh with changing bones and Blend Shapes","error="+sourceError);
                Require(receiverError<.0001,"1.0.4 receiver deformation matches native BakeMesh with changing bones and Blend Shapes","error="+receiverError);
                Require(solver.MaximumEdgeStrain<=settings.MaxTissueStrain+1e-8,"1.0.4 native pair preserves simulation strain cap");
                Require(probe.sharedMaterial==sourceMaterial&&receiver.sharedMaterial==material,"1.0.4 native pair preserves original materials");
                geometry.ReleaseCorrection(); Require(probe.sharedMesh==sourceOriginal&&probe.GetBlendShapeWeight(0)==78,"1.0.4 source restoration preserves mesh reference and current Blend Shape weight");
                for(int f=0;f<271;f++) { animation.Capture(solver); solver.Advance(animation.Anchors,null,tissue,PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,animation.PosedSize,settings); target.Apply(solver,.2*animation.PosedSize,1); if(f%30==0) yield return null; }
                double remaining=0; for(int i=0;i<solver.Positions.Length;i++) remaining=Math.Max(remaining,solver.SolvedDisplacement(i).Length);
                Require(remaining<1e-8&&target.VerifyAppliedPose(solver)<.0001,"1.0.4 native pair recovers within 4.5 seconds","remaining="+remaining);
                var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(100,99.6,100),new Vec3(100,100,100.02),new Vec3(100,100.4,100.04)});
                body.BeginFrame(settings,.016); Vec3 point=new Vec3(100.08,100,100); double before=path.Closest(point).Distance;
                for(int f=0;f<30;f++) path.Constrain(body,point,new Vec3(.1,1,0),.016);
                geometry.Capture(true); geometry.ApplyCorrection(body);
                Require(path.Closest(point+body.Offset).Distance<before*.1&&body.RotationVector.Length>0,"1.0.4 native source path reduces position and orientation deviation");
                geometry.ApplyCorrection(body,.5); probe.BakeMesh(baked); var faded=baked.vertices; double fadeError=0;
                for(int i=0;i<faded.Length;i++) { var baseline=geometry.AnimatedVertices[i]; var expected=baseline+(body.ToCorrected(baseline)-baseline)*.5; fadeError=Math.Max(fadeError,(AnimationAdapter.ToCore(AnimationAdapter.BakedToWorld(probe).MultiplyPoint3x4(faded[i]))-expected).Length); }
                Require(fadeError<.0001,"1.0.4 source mode transition applies a bounded half correction without accumulation","error="+fadeError); geometry.ApplyCorrection(body);
                var visible=geometry.XRaySnapshot(); probe.BakeMesh(baked); var corrected=baked.vertices; var snapshot=visible.vertices; double snapshotError=0;
                for(int i=0;i<corrected.Length;i++) snapshotError=Math.Max(snapshotError,(corrected[i]-snapshot[i]).magnitude);
                Require(snapshotError<.0001,"1.0.4 XRay snapshot uses mechanically corrected source geometry","error="+snapshotError);
                geometry.ReleaseCorrection(); settings.Mode=ContactMode.Original; solver.Advance(animation.Anchors,body,tissue,PhysicsPolicy.Resolve(PhysicsQuality.Balanced),.02,animation.PosedSize,settings); target.Apply(solver,.2,1);
                Require(solver.SolvedDisplacement(grid.Vertices.Length/2).Length==0,"1.0.4 Original mode restores animation baseline");
                Array.Sort(times); double total=0; foreach(double value in times) total+=value;
                report.checks.Add(new Check { name="1.0.4 native coupled performance",status="PASS",detail="particles="+grid.Vertices.Length+" sourceVertices="+evidence.Length+" frames=90 meanMs="+total/90+" p95Ms="+times[85]+" Capture+BVH+Solve+Apply; verification excluded, not game FPS" });
            }
            Require(receiver.sharedMesh==original&&probe.sharedMesh==sourceOriginal,"1.0.4 both private motion meshes restore on disposal"); receiver.enabled=false; probe.enabled=false;
            for(int scenario=0;scenario<3;scenario++) {
                var tube=LabScenarios.Tube(24,12,.2,1); var r=Skin("Neutral scenario "+(char)('A'+scenario),tube,Vector3.zero); var def=Definition(r.sharedMesh,r,tube);
                for(int i=0;i<def.PinWeights.Length;i++) def.PinWeights[i]=Math.Abs(tube.Vertices[i].Y)>.49?1:0;
                var mesh=scenario==1?PrimitiveMeshes.Cylinder(24,6,.17,.25):PrimitiveMeshes.Sphere(24,12,scenario==2?.24:.215); var instrument=Skin("Neutral scenario contact model",mesh,new Vector3(.035f,0,0));
                var weights=new double[mesh.Vertices.Length]; for(int i=0;i<weights.Length;i++) weights[i]=1;
                using(var g=new ContactDetection(instrument,weights)) using(var map=new MeshDeformationBinding(r,def)) {
                    var a=new AnimationAdapter(def,r); var s=new SoftBodySolver(new SimulationTopology(tube.Vertices,tube.Triangles),def.PinWeights); var b=new SourceContactBody { Surface=g }; var c=new BidirectionalContactSettings { MaxSourceDisplacement=.12,SourceCorrectionStrength=.05 };
                    for(int f=0;f<30;f++) { g.Capture(true); a.Capture(s); b.BeginFrame(c,1.0/60); s.Advance(a.Anchors,b,new SoftBodySettings(),PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,a.PosedSize,c); map.Apply(s,.2*a.PosedSize,1); g.ApplyCorrection(b); if(f%5==0) yield return null; }
                    double displacement=0; foreach(var p in s.Positions) RequireFinite(p); for(int i=0;i<s.Positions.Length;i++) displacement=Math.Max(displacement,s.SolvedDisplacement(i).Length);
                    Require(s.MaximumEdgeStrain<=c.MaxTissueStrain+1e-8,"1.0.4 neutral scenario "+(char)('A'+scenario)+" obeys native tube strain and finite coordinate limits","tissue="+displacement+" source="+b.Offset.Length+" residual="+s.MaximumResidualPenetration);
                }
                r.enabled=false; instrument.enabled=false;
            }
        }
        static void RequireFinite(Vec3 p) { if(!p.IsFinite) throw new InvalidOperationException("Nonfinite native scenario."); }
    }
}
