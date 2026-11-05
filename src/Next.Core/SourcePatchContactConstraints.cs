using System;
using System.Collections.Generic;
using System.Diagnostics;
namespace BetterPenetrationNext.Core
{
 // Each source triangle belongs to one small convex patch. Every original
 // indexed vertex is retained, including coincident independently skinned seams.
 // Local receiver face half-spaces certify the whole patch, rather than a sphere
 // whose radius unnecessarily inflates a thin, long source triangle.
 public sealed class SourcePatchContactConstraints
 {
  readonly int[] receiverFaces,sourceFaces,samples,cellOffsets,cellVertices,overlaps,hints;
  readonly Vec3[] sourceWorld,closedPositions;
  readonly int[][] boundaries;
  readonly TriangleSurface closed,side;
  readonly int[] pairVertex,pairFace;
  readonly Vec3[] pairNormal,pairBary;
  readonly double[] pairViolation,lambda;
  readonly bool[] valid;
  Vec3 sourcePoseOffset;
  bool built;
  public SourceContactBody Source;
  public Vec3[] AnimatedSource;
  public Vec3[] WorldSourceOverride;
  public double Clearance=.0002;
  public int BuildCount,ClassificationQueries,SideQueries,BroadphaseQueries,SupportTests,ActiveContacts,ResidualBuilds;
  public double BuildMilliseconds,ProjectMilliseconds,ResidualMilliseconds,MaximumViolation,MaximumCapViolation;
  public SourcePatchContactConstraints(Vec3[] receiver,int[] receiverTriangles,Vec3[] sourceReference,int[] sourceTriangles,int cellCount=512)
  {
   receiverFaces=(int[])receiverTriangles.Clone();sourceFaces=(int[])sourceTriangles.Clone();sourceWorld=new Vec3[sourceReference.Length];cellCount=Math.Min(cellCount,sourceReference.Length);samples=new int[cellCount];hints=new int[cellCount];var distance=new double[sourceReference.Length];for(int i=0;i<distance.Length;i++)distance[i]=double.PositiveInfinity;int next=0;
   for(int s=0;s<cellCount;s++){samples[s]=next;hints[s]=-1;double largest=-1;for(int i=0;i<distance.Length;i++){distance[i]=Math.Min(distance[i],(sourceReference[i]-sourceReference[next]).LengthSquared);if(distance[i]>largest){largest=distance[i];next=i;}}}
   var members=new HashSet<int>[cellCount];for(int s=0;s<cellCount;s++){members[s]=new HashSet<int>();members[s].Add(samples[s]);}
   for(int t=0;t<sourceFaces.Length;t+=3){int owner=0;double best=double.PositiveInfinity;for(int s=0;s<cellCount;s++){double r=0;for(int j=0;j<3;j++)r=Math.Max(r,(sourceReference[sourceFaces[t+j]]-sourceReference[samples[s]]).LengthSquared);if(r<best){best=r;owner=s;}}for(int j=0;j<3;j++)members[owner].Add(sourceFaces[t+j]);}
   cellOffsets=new int[cellCount+1];var all=new List<int>();for(int s=0;s<cellCount;s++){var indices=new int[members[s].Count];members[s].CopyTo(indices);Array.Sort(indices);all.AddRange(indices);cellOffsets[s+1]=all.Count;}cellVertices=all.ToArray();
   var edgeCount=new Dictionary<long,int>();var directed=new Dictionary<long,EdgeConstraint>();for(int t=0;t<receiverFaces.Length;t+=3)for(int j=0;j<3;j++){int a=receiverFaces[t+j],b=receiverFaces[t+(j+1)%3];long k=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b);int n;edgeCount.TryGetValue(k,out n);edgeCount[k]=n+1;directed[k]=new EdgeConstraint(a,b);}
   var graph=new Dictionary<int,List<int>>();foreach(var entry in edgeCount)if(entry.Value==1){var e=directed[entry.Key];if(!graph.ContainsKey(e.A))graph[e.A]=new List<int>();if(!graph.ContainsKey(e.B))graph[e.B]=new List<int>();graph[e.A].Add(e.B);graph[e.B].Add(e.A);}var loops=new List<int[]>();var seen=new HashSet<int>();var loopIndex=new int[receiver.Length];for(int i=0;i<loopIndex.Length;i++)loopIndex[i]=-1;
   foreach(var entry in graph){if(seen.Contains(entry.Key))continue;var loop=new List<int>();int first=entry.Key,v=first,previous=-1;do{if(graph[v].Count!=2)throw new ArgumentException("Receiver has a branched open boundary");loop.Add(v);seen.Add(v);int n=graph[v][0]==previous?graph[v][1]:graph[v][0];previous=v;v=n;}while(v!=first);foreach(int i in loop)loopIndex[i]=loops.Count;loops.Add(loop.ToArray());}boundaries=loops.ToArray();closedPositions=new Vec3[receiver.Length+boundaries.Length];Array.Copy(receiver,closedPositions,receiver.Length);var capped=new List<int>(receiverFaces);
   for(int l=0;l<boundaries.Length;l++){int extra=receiver.Length+l;foreach(int i in boundaries[l])closedPositions[extra]+=receiver[i]/boundaries[l].Length;foreach(var entry in edgeCount)if(entry.Value==1){var e=directed[entry.Key];if(loopIndex[e.A]==l){capped.Add(e.B);capped.Add(e.A);capped.Add(extra);}}}
   closed=new TriangleSurface(closedPositions,capped.ToArray(),SurfaceKind.ClosedVolume){RobustSign=false};side=new TriangleSurface(receiver,receiverFaces,SurfaceKind.OrientedShell);overlaps=new int[closed.TriangleCount];int count=cellCount+receiverFaces.Length/3;pairVertex=new int[count];pairFace=new int[count];pairNormal=new Vec3[count];pairBary=new Vec3[count];pairViolation=new double[count];lambda=new double[count];valid=new bool[count];
  }
  static Vec3 Corrected(SourceContactBody body,int index,Vec3 point)
  {var indexed=body.Deformation as IIndexedSourceDeformation;Vec3 p=indexed!=null?indexed.MapVertex(index,point):body.Deformation==null?point:body.Deformation.Map(point);return body.Pivot+body.Rotate(p-body.Pivot)+body.Offset;}
  public void BeginStep(){built=false;BuildCount=ClassificationQueries=SideQueries=BroadphaseQueries=SupportTests=ActiveContacts=ResidualBuilds=0;BuildMilliseconds=ProjectMilliseconds=ResidualMilliseconds=MaximumViolation=MaximumCapViolation=0;Array.Clear(lambda,0,lambda.Length);}
  void Add(int slot,int face,int vertex,Vec3 normal,Vec3 point,Vec3[] receiver)
  {
   int t=face*3,a=receiverFaces[t],b=receiverFaces[t+1],c=receiverFaces[t+2];var triangle=new TriangleQuery(receiver[a],receiver[b],receiver[c]);Vec3 q=triangle.Closest(point);double gap=Vec3.Dot(point-q,normal)-Clearance,violation=-gap;if(violation<=0||valid[slot]&&pairViolation[slot]>=violation)return;Vec3 ab=receiver[b]-receiver[a],ac=receiver[c]-receiver[a],aq=q-receiver[a];double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),den=aa*cc-bb*bb;if(Math.Abs(den)<1e-24)return;double v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/den,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/den;
   valid[slot]=true;pairVertex[slot]=vertex;pairFace[slot]=face;pairNormal[slot]=normal;pairBary[slot]=new Vec3(1-v-w,v,w);pairViolation[slot]=violation;MaximumViolation=Math.Max(MaximumViolation,violation);
  }
  void Build(Vec3[] receiver,bool residual)
  {
   long started=Stopwatch.GetTimestamp();built=true;if(residual)ResidualBuilds++;else BuildCount++;Array.Clear(valid,0,valid.Length);Array.Clear(lambda,0,lambda.Length);MaximumViolation=MaximumCapViolation=0;
   Array.Copy(receiver,closedPositions,receiver.Length);for(int l=0;l<boundaries.Length;l++){Vec3 centre=new Vec3();foreach(int i in boundaries[l])centre+=receiver[i]/boundaries[l].Length;closedPositions[receiver.Length+l]=centre;}closed.Refit(closedPositions);side.Refit(receiver);sourcePoseOffset=Source.Offset;for(int i=0;i<sourceWorld.Length;i++)sourceWorld[i]=WorldSourceOverride==null?Corrected(Source,i,AnimatedSource[i]):Source.Pivot+Source.Rotate(WorldSourceOverride[i]-Source.Pivot)+Source.Offset;double orientation=closed.OutwardOrientation;
   for(int s=0;s<samples.Length;s++){
    Vec3 centre=sourceWorld[samples[s]];SurfaceSample sample;ClassificationQueries++;if(!closed.TrySampleWithRays(centre,1,ref hints[s],out sample))continue;
    if(sample.SignedDistance>0){SurfaceSample hit=sample;if(hit.Triangle>=receiverFaces.Length/3){SideQueries++;if(!side.TrySample(centre,1,out hit))continue;}Vec3 normal=(hit.Point-centre).Normalized;if(normal.LengthSquared<.5)normal=-sample.Normal;int vertex=samples[s];double lowest=double.PositiveInfinity;for(int j=cellOffsets[s];j<cellOffsets[s+1];j++){int i=cellVertices[j];double d=Vec3.Dot(sourceWorld[i],normal);SupportTests++;if(d<lowest){lowest=d;vertex=i;}}Add(s,hit.Triangle,vertex,normal,sourceWorld[vertex],receiver);continue;}
    var bounds=Bounds3.Empty;for(int j=cellOffsets[s];j<cellOffsets[s+1];j++)bounds.Include(sourceWorld[cellVertices[j]]);Vec3 pad=new Vec3(Clearance,Clearance,Clearance);bounds=new Bounds3(bounds.Min-pad,bounds.Max+pad);BroadphaseQueries++;int count=closed.CollectTriangles(bounds,overlaps);
    for(int f=0;f<count;f++){int face=overlaps[f];Vec3 a,b,c;closed.GetTriangle(face,out a,out b,out c);Vec3 outward=Vec3.Cross(b-a,c-a).Normalized*orientation;if(outward.LengthSquared<.5)continue;double largest=double.NegativeInfinity;int vertex=-1;for(int j=cellOffsets[s];j<cellOffsets[s+1];j++){int i=cellVertices[j];double d=Vec3.Dot(sourceWorld[i]-a,outward);SupportTests++;if(d>largest){largest=d;vertex=i;}}double violation=largest+Clearance;if(violation<=0)continue;if(face>=receiverFaces.Length/3){MaximumCapViolation=Math.Max(MaximumCapViolation,violation);MaximumViolation=Math.Max(MaximumViolation,violation);continue;}Add(samples.Length+face,face,vertex,-outward,sourceWorld[vertex],receiver);}
   }
   double elapsed=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;if(residual)ResidualMilliseconds+=elapsed;else BuildMilliseconds+=elapsed;
  }
  public double RefreshResidual(Vec3[] receiver){Build(receiver,true);return MaximumViolation;}
  public void Project(Vec3[] receiver,Vec3[] anchors,double[] inverseMass,double maximumOffset,BidirectionalContactSettings settings,double h)
  {
   if(Source==null||AnimatedSource==null)return;if(!built)Build(receiver,false);long started=Stopwatch.GetTimestamp();ActiveContacts=0;double alpha=settings.TissueCompliance/(h*h);
   for(int k=0;k<valid.Length;k++)if(valid[k]){int t=pairFace[k]*3,a=receiverFaces[t],b=receiverFaces[t+1],c=receiverFaces[t+2];Vec3 bary=pairBary[k],normal=pairNormal[k],point=sourceWorld[pairVertex[k]]+Source.Offset-sourcePoseOffset,q=receiver[a]*bary.X+receiver[b]*bary.Y+receiver[c]*bary.Z;double gap=Vec3.Dot(point-q,normal)-Clearance;Vec3 gradient=Source.ProjectContactGradient(normal);double wa=inverseMass[a]*settings.TissueDeformationStrength,wb=inverseMass[b]*settings.TissueDeformationStrength,wc=inverseMass[c]*settings.TissueDeformationStrength,wr=wa*bary.X*bary.X+wb*bary.Y*bary.Y+wc*bary.Z*bary.Z,ws=Source.InverseMass*gradient.LengthSquared;double next=BidirectionalContactSolver.Multiplier(gap,lambda[k],wr,ws,alpha),delta=next-lambda[k];lambda[k]=next;
    if(delta>0){Vec3 before=Source.Offset;Source.Correct(gradient*(Source.InverseMass*delta));double accepted=Math.Max(0,Vec3.Dot(Source.Offset-before,normal));if(wr>1e-20)delta+=Math.Max(0,ws*delta-accepted)/wr;receiver[a]=anchors[a]+Vec3.ClampLength(receiver[a]-anchors[a]-normal*(wa*bary.X*delta),maximumOffset);receiver[b]=anchors[b]+Vec3.ClampLength(receiver[b]-anchors[b]-normal*(wb*bary.Y*delta),maximumOffset);receiver[c]=anchors[c]+Vec3.ClampLength(receiver[c]-anchors[c]-normal*(wc*bary.Z*delta),maximumOffset);}
    if(next>0){ActiveContacts++;Source.MarkContact();}
   }
   ProjectMilliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;
  }
 }
}
