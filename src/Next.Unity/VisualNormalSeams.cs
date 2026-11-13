using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    // Share a shading frame across true UV copies, never across independent
    // skin/morph/procedural identities or opposite sides of a material wall.
    sealed class VisualNormalSeams
    {
        sealed class Group {public int[] Vertices;public Vector3 AreaReference,UnitReference;}
        readonly List<Group> groups=new List<Group>();
        public int Count {get{return groups.Count;}}
        static ulong Mix(ulong h,long x){unchecked{return (h^(ulong)x)*1099511628211UL;}}
        static ulong Number(ulong h,double x){return Mix(h,BitConverter.DoubleToInt64Bits(x));}
        static ulong Vector(ulong h,Vector3 p){return Number(Number(Number(h,p.x),p.y),p.z);}
        static Vector3 Unit(Vector3 p){return AnimationAdapter.ToUnity(AnimationAdapter.ToCore(p).Normalized);}
        public VisualNormalSeams(Mesh mesh,SimulationDefinition data,Vector3[] rest,Vector3[] authored,Vector3[] areaReference,Vector3[] unitReference,int[] affected,int[] offsets,int[] bones,double[] weights)
        {
            var hashes=new ulong[rest.Length];var byVertex=new VisualBinding[rest.Length];var bound=new bool[rest.Length];foreach(var b in data.Bindings){byVertex[b.Vertex]=b;bound[b.Vertex]=true;}
            foreach(int i in affected)
            {
                ulong h=Vector(Vector(14695981039346656037UL,rest[i]),authored[i]);h=Mix(h,bound[i]?1:0);
                if(bound[i]){var b=byVertex[i];h=Mix(Mix(Mix(h,b.A),b.B),b.C);h=Number(Number(Number(Number(h,b.U),b.V),b.W),b.Influence);}
                for(int k=offsets[i];k<offsets[i+1];k++)h=Number(Mix(h,bones[k]),weights[k]);hashes[i]=h;
            }
            if(mesh.blendShapeCount>0)
            {
                var dv=new Vector3[rest.Length];var dn=new Vector3[rest.Length];var dt=new Vector3[rest.Length];
                for(int s=0;s<mesh.blendShapeCount;s++)for(int f=0;f<mesh.GetBlendShapeFrameCount(s);f++)
                {mesh.GetBlendShapeFrameVertices(s,f,dv,dn,dt);foreach(int i in affected)hashes[i]=Vector(Vector(hashes[i],dv[i]),dn[i]);}
            }
            var buckets=new Dictionary<ulong,List<List<int>>>();
            foreach(int i in affected)
            {
                List<List<int>> bucket;if(!buckets.TryGetValue(hashes[i],out bucket)){bucket=new List<List<int>>();buckets.Add(hashes[i],bucket);}List<int> match=null;
                foreach(var candidate in bucket)
                {
                    int j=candidate[0];bool same=rest[i].Equals(rest[j])&&authored[i].Equals(authored[j])&&bound[i]==bound[j]&&offsets[i+1]-offsets[i]==offsets[j+1]-offsets[j]&&Vector3.Dot(unitReference[i],unitReference[j])>0;
                    if(same&&bound[i]){var a=byVertex[i];var b=byVertex[j];same=a.A==b.A&&a.B==b.B&&a.C==b.C&&a.U==b.U&&a.V==b.V&&a.W==b.W&&a.Influence==b.Influence;}
                    for(int k=0;same&&k<offsets[i+1]-offsets[i];k++)same=bones[offsets[i]+k]==bones[offsets[j]+k]&&weights[offsets[i]+k]==weights[offsets[j]+k];
                    if(same){match=candidate;break;}
                }
                if(match==null){match=new List<int>();bucket.Add(match);}match.Add(i);
            }
            foreach(var bucket in buckets.Values)foreach(var vertices in bucket)if(vertices.Count>1)
            {var group=new Group{Vertices=vertices.ToArray()};foreach(int i in group.Vertices){group.AreaReference+=areaReference[i];group.UnitReference+=unitReference[i];}group.AreaReference=Unit(group.AreaReference);group.UnitReference=Unit(group.UnitReference);groups.Add(group);}
        }
        public void Apply(Vector3[] positions,Vector3[] geometric,bool unitInput,Vector3[] authored,Vector4[] authoredTangents,Vector3[] normals,Vector4[] tangents)
        {
            foreach(var group in groups)
            {
                int first=group.Vertices[0];bool coincident=true;Vector3 sum=Vector3.zero;
                foreach(int i in group.Vertices){coincident&=(positions[i]-positions[first]).sqrMagnitude<=1e-14f;sum+=geometric[i];}
                if(!coincident)continue;Vector3 target=Unit(sum),reference=unitInput?group.UnitReference:group.AreaReference;if(target.sqrMagnitude<.5f||reference.sqrMagnitude<.5f)continue;
                var rotation=Quaternion.FromToRotation(reference,target);
                foreach(int i in group.Vertices)
                {
                    normals[i]=Unit(rotation*authored[i]);
                    if(tangents.Length==positions.Length){var t=authoredTangents[i];Vector3 q=rotation*new Vector3(t.x,t.y,t.z);tangents[i]=new Vector4(q.x,q.y,q.z,t.w);}
                }
            }
        }
        public double Measure(Vector3[] positions,Vector3[] normals)
        {
            double maximum=0;foreach(var group in groups){int first=group.Vertices[0];foreach(int i in group.Vertices)if((positions[i]-positions[first]).sqrMagnitude<1e-10f)maximum=Math.Max(maximum,(Unit(normals[i])-Unit(normals[first])).magnitude);}return maximum;
        }
    }
}
