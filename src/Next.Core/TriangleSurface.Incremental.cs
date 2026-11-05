using System;
namespace BetterPenetrationNext.Core
{
 public sealed partial class TriangleSurface
 {
  int[] vertexFaceOffsets,vertexFaces,faceLeaves,parents,changedLeaves,leafStamps;
  int refitStamp;
  public void InitializeIncrementalRefit()
  {
   if(vertexFaceOffsets!=null)return;
   if(kind!=SurfaceKind.OrientedShell&&!RobustSign)throw new InvalidOperationException("Incremental closed refit requires ray classification; pseudonormals are not updated");
   vertexFaceOffsets=new int[vertices.Length+1];foreach(int v in triangles)vertexFaceOffsets[v+1]++;
   for(int i=1;i<vertexFaceOffsets.Length;i++)vertexFaceOffsets[i]+=vertexFaceOffsets[i-1];
   vertexFaces=new int[triangles.Length];var cursor=(int[])vertexFaceOffsets.Clone();for(int t=0;t<triangles.Length;t++)vertexFaces[cursor[triangles[t]]++]=t/3;
   faceLeaves=new int[queries.Length];parents=new int[used];changedLeaves=new int[used];leafStamps=new int[used];parents[0]=-1;
   for(int n=0;n<used;n++){var node=nodes[n];if(node.Count>0)for(int j=node.Start;j<node.Start+node.Count;j++)faceLeaves[order[j]]=n;else{parents[node.Left]=n;parents[node.Right]=n;}}
  }
  public void RefitVertices(Vec3[] positions,int a,int b,int c)
  {
   if(vertexFaceOffsets==null)throw new InvalidOperationException("Initialize incremental refit before stepping");
   if(positions.Length!=vertices.Length||!positions[a].IsFinite||!positions[b].IsFinite||!positions[c].IsFinite)throw new ArgumentException("Invalid incremental pose");
   vertices[a]=positions[a];vertices[b]=positions[b];vertices[c]=positions[c];
   if(++refitStamp==int.MaxValue){Array.Clear(leafStamps,0,leafStamps.Length);refitStamp=1;}int count=0;
   for(int i=0;i<3;i++){int v=i==0?a:i==1?b:c;for(int j=vertexFaceOffsets[v];j<vertexFaceOffsets[v+1];j++){int face=vertexFaces[j],t=face*3;queries[face]=new TriangleQuery(vertices[triangles[t]],vertices[triangles[t+1]],vertices[triangles[t+2]]);int leaf=faceLeaves[face];if(leafStamps[leaf]!=refitStamp){leafStamps[leaf]=refitStamp;changedLeaves[count++]=leaf;}}}
   for(int i=0;i<count;i++){int n=changedLeaves[i];var node=nodes[n];var bounds=Bounds3.Empty;for(int j=node.Start;j<node.Start+node.Count;j++)bounds.Include(TriangleBounds(order[j]));node.Bounds=bounds;nodes[n]=node;
    for(n=parents[n];n>=0;n=parents[n]){node=nodes[n];bounds=Bounds3.Empty;bounds.Include(nodes[node.Left].Bounds);bounds.Include(nodes[node.Right].Bounds);node.Bounds=bounds;nodes[n]=node;}}
  }
 }
}
