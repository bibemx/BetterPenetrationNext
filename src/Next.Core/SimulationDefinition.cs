using System;
using System.IO;
using System.Security.Cryptography;

namespace BetterPenetrationNext.Core
{
    [Serializable] public sealed class SimulationMorph { public string Name; public Vec3[] Deltas; public float DefaultWeight; }
    [Serializable] public sealed class VisualUvPatch
    {
        public int[] Duplicates,Triangles;public double[] Coordinates;
        public void Validate(int originalVertices,int[] originalTriangles)
        {
            if(Duplicates==null||Triangles==null||Coordinates==null||Coordinates.Length!=2*(originalVertices+Duplicates.Length)||Triangles.Length!=originalTriangles.Length)throw new ArgumentException("UV patch topology mismatch");
            foreach(int v in Duplicates)if(v<0||v>=originalVertices)throw new ArgumentException("Invalid UV seam identity");
            foreach(double v in Coordinates)if(!Vec3.Finite(v))throw new ArgumentException("Nonfinite UV coordinate");
            for(int i=0;i<Triangles.Length;i++){int v=Triangles[i];if(v<0||v>=originalVertices+Duplicates.Length||(v<originalVertices?v:Duplicates[v-originalVertices])!=originalTriangles[i])throw new ArgumentException("UV patch changed geometric topology");}
        }
    }
    [Serializable] public sealed class SimulationDefinition
    {
        public int Schema=1;
        public string BodyGuid="org.betterpenetration.next.body.v1",VisualMeshSignature;
        public Vec3[] Rest;
        public int[] Triangles,InfluenceOffsets,BoneIndices;
        [System.Runtime.Serialization.OptionalField] public int[] ConfinementTriangles;
        public double[] PinWeights,BoneWeights;
        [System.Runtime.Serialization.OptionalField] public double[] GuideCoordinates,GuideWeights;
        public string[] BoneNames;
        public float[] Bindposes;
        public VisualBinding[] Bindings;
        public double MaximumBindingDistance;
        public SimulationMorph[] Morphs;
        public bool PreserveVolume;
        [System.Runtime.Serialization.OptionalField] public bool AllowJunctions;
        public int[] XRayExcludedSubmeshes;
        [System.Runtime.Serialization.OptionalField] public VisualUvPatch UvPatch;
        public void Validate(int visualVertices)
        {
            if(Schema!=1||BodyGuid!="org.betterpenetration.next.body.v1"||string.IsNullOrEmpty(VisualMeshSignature)||Rest==null||Rest.Length<3||Rest.Length>10000||PinWeights==null||PinWeights.Length!=Rest.Length) throw new ArgumentException("Unsupported simulation package.");
            new SimulationTopology(Rest,Triangles,AllowJunctions);
            if(BoneNames==null||BoneNames.Length==0||Bindposes==null||Bindposes.Length!=BoneNames.Length*16||InfluenceOffsets==null||InfluenceOffsets.Length!=Rest.Length+1||BoneIndices==null||BoneWeights==null||BoneIndices.Length!=BoneWeights.Length||InfluenceOffsets[Rest.Length]!=BoneIndices.Length) throw new ArgumentException("Incomplete simulation skinning.");
            for(int i=0;i<Rest.Length;i++)
            {
                if(!Vec3.Finite(PinWeights[i])||PinWeights[i]<0||PinWeights[i]>1||InfluenceOffsets[i]<0||InfluenceOffsets[i+1]<=InfluenceOffsets[i]) throw new ArgumentException("Invalid simulation weights.");
                double sum=0; for(int w=InfluenceOffsets[i];w<InfluenceOffsets[i+1];w++)
                { if(BoneIndices[w]<0||BoneIndices[w]>=BoneNames.Length||!Vec3.Finite(BoneWeights[w])||BoneWeights[w]<0) throw new ArgumentException("Invalid bone influence."); sum+=BoneWeights[w]; }
                if(Math.Abs(sum-1)>1e-4) throw new ArgumentException("Unnormalized simulation weights.");
            }
            if(Bindings==null||Bindings.Length==0) throw new ArgumentException("Missing Visual/Simulation mapping.");
            var seen=new bool[visualVertices];
            foreach(var b in Bindings)
            {
                if(b.Vertex<0||b.Vertex>=visualVertices||seen[b.Vertex]||b.A<0||b.A>=Rest.Length||b.B<0||b.B>=Rest.Length||b.C<0||b.C>=Rest.Length
                    ||!Vec3.Finite(b.U)||!Vec3.Finite(b.V)||!Vec3.Finite(b.W)||Math.Abs(b.U+b.V+b.W-1)>1e-6||b.U<-1e-5||b.V<-1e-5||b.W<-1e-5||b.Influence<0||b.Influence>1) throw new ArgumentException("Invalid visual binding.");
                seen[b.Vertex]=true;
            }
            foreach(float value in Bindposes) if(float.IsNaN(value)||float.IsInfinity(value)) throw new ArgumentException("Nonfinite bindpose.");
            if(Morphs!=null) foreach(var morph in Morphs)
            { if(morph==null||string.IsNullOrEmpty(morph.Name)||morph.Deltas==null||morph.Deltas.Length!=Rest.Length) throw new ArgumentException("Incomplete simulation morph."); foreach(var p in morph.Deltas) if(!p.IsFinite) throw new ArgumentException("Nonfinite morph delta."); }
        }
        public static string Signature(Vec3[] vertices,int[] triangles)
        {
            using(var memory=new MemoryStream())
            {
                using(var writer=new BinaryWriter(memory,System.Text.Encoding.UTF8,true))
                {
                    writer.Write(vertices.Length); writer.Write(triangles.Length);
                    foreach(var p in vertices) { writer.Write((float)p.X); writer.Write((float)p.Y); writer.Write((float)p.Z); }
                    foreach(int i in triangles) writer.Write(i);
                }
                memory.Position=0; using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(memory)).Replace("-","");
            }
        }
    }
}
