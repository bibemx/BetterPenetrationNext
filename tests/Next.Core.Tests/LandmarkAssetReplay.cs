using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json;
using BetterPenetrationNext.Core;

internal static class LandmarkAssetReplay
{
    static Vec3 V(JsonElement p){return new Vec3(p[0].GetDouble(),p[1].GetDouble(),p[2].GetDouble());}
    static Vec3[] Vertices(JsonElement a){return a.EnumerateArray().Select(V).ToArray();}
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run(string input,string output)
    {
        using(var document=JsonDocument.Parse(File.ReadAllText(input)))
        {
            var rows=new List<object>();
            foreach(var role in document.RootElement.GetProperty("Roles").EnumerateArray())
            {
                var reference=Vertices(role.GetProperty("Reference"));var faces=role.GetProperty("Triangles").EnumerateArray().Select(x=>x.GetInt32()).ToArray();
                var frames=role.GetProperty("Frames").EnumerateArray().Select(x=>new{Id=x.GetProperty("Id").GetInt32(),Vertices=Vertices(x.GetProperty("Vertices"))}).ToArray();
                var seams=role.GetProperty("DifferentWeightSeams").EnumerateArray().Select(x=>new[]{x[0].GetInt32(),x[1].GetInt32()}).ToArray();
                var timer=Stopwatch.StartNew();var landmarks=new AdaptiveMeshLandmarks(reference,faces,V(role.GetProperty("Axis")));timer.Stop();double setup=timer.Elapsed.TotalMilliseconds;
                int cuts=landmarks.ReferenceTriangleTests;var surface=new TriangleSurface(reference,faces,SurfaceKind.OrientedShell);double surfaceError=0,seamGap=0;int validated=0;
                foreach(var frame in frames)
                {
                    Require(frame.Vertices.Length==reference.Length,"Recorded vertex identity changed");landmarks.Update(frame.Vertices,new Vec3(1,0,0));surface.Refit(frame.Vertices);
                    foreach(var section in landmarks.Sections)
                    {
                        Require(section.Centre.IsFinite&&section.Tangent.IsFinite,"Nonfinite recorded section frame");
                        foreach(var p in section.Radial){SurfaceSample sample;Require(surface.TrySample(p,.01,out sample),"Recorded radial landmark has no original surface");surfaceError=Math.Max(surfaceError,(p-sample.Point).Length);validated++;}
                    }
                    foreach(var seam in seams)seamGap=Math.Max(seamGap,(frame.Vertices[seam[0]]-frame.Vertices[seam[1]]).Length);
                    foreach(var p in landmarks.Extrema)Require(p.IsFinite,"Nonfinite recorded extremum");
                }
                Require(surfaceError<1e-7,"Cached landmarks left the original ZIPMOD surface");
                const int iterations=600;var times=new double[iterations];
                for(int i=0;i<120;i++)landmarks.Update(frames[i%frames.Length].Vertices,new Vec3(1,0,0));
                long before=GC.GetAllocatedBytesForCurrentThread();
                for(int i=0;i<iterations;i++){long start=Stopwatch.GetTimestamp();landmarks.Update(frames[i%frames.Length].Vertices,new Vec3(1,0,0));times[i]=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency;}
                long allocated=GC.GetAllocatedBytesForCurrentThread()-before;double total=0;foreach(var ms in times)total+=ms;Array.Sort(times);
                Require(allocated==0,"Recorded landmark updates allocated");Require(landmarks.ReferenceTriangleTests==cuts,"Recorded landmark updates recut triangles");
                var topology=new SimulationTopology(reference,faces,true);
                rows.Add(new{Role=role.GetProperty("Role").GetString(),Vertices=reference.Length,Triangles=faces.Length/3,topology.BoundaryEdges,RecordedPoses=frames.Select(x=>x.Id).ToArray(),Sections=landmarks.Sections.Length,ValidatedRadialPoints=validated,MaximumDistanceToActualSurface=surfaceError,DifferentWeightSeamPairs=seams.Length,MaximumRecordedSeamGap=seamGap,ReferenceTriangleCuts=cuts,PerUpdateTriangleCuts=landmarks.ReferenceTriangleTests-cuts,SetupMs=setup,MeanLandmarkUpdateMs=total/iterations,P95LandmarkUpdateMs=times[(int)Math.Ceiling(iterations*.95)-1],WarmAllocatedBytes=allocated,RepeatedUpdates=iterations});
            }
            var report=new{Scope="New landmark component only, on actual cached original ZIPMOD geometry and recorded Runtime animation; independent surface checks excluded from component timing. No new live user-scene, contact-solver or containment acceptance.",Passed=true,Rows=rows};
            string json=JsonSerializer.Serialize(report,new JsonSerializerOptions{WriteIndented=true});File.WriteAllText(output,json);Console.WriteLine(json);
        }
    }
}
