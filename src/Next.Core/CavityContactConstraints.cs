using System;
using System.Collections.Generic;
namespace BetterPenetrationNext.Core
{
 // An oriented, open-channel confinement constraint. No synthetic end cap is
 // added. Points before the entrance or beyond the supplied path are excluded.
 public sealed class CavityContactConstraints
 {
  readonly int[] triangles,samples,hints;readonly double[] multipliers;
  readonly TriangleSurface surface;
  readonly TriangleSurface classifier;readonly Vec3[] classificationPositions;readonly int[][] boundaries;readonly int[] boundaryOfVertex;
  public bool UseVolumeClassification=true;
  public bool UseCachedContacts=true;
  readonly int[] contactFaces;readonly Vec3[] contactNormals,barycentrics;readonly bool[] valid;
  bool built;
  readonly int[] sourceTriangles,triangleOwners;readonly Vec3[] correctedSource;readonly double[] coverRadii;
  public bool UseConservativeProxy;
  public double MaximumCoverRadius {get;private set;}
  public int Builds;public double BuildMilliseconds,ProjectionMilliseconds;
  public int ResidualSearchQueries,ResidualBuilds;public double ResidualMilliseconds,MaximumTrueViolation;
  public bool WholeSourceCoverage;
  public double ReceiverOutwardSign {get{return classifier.CalculateSignedVolume()<0?-1:1;}}
  public bool UseExactSurfaceCoverage;
  readonly int[] vertexOwners,contactVertices,vertexHints,worstVertices;readonly SurfaceSample[] worstHits;readonly double[] worstClearance;
  readonly double[] clearanceBounds;readonly Vec3[] checkedSource,checkedReceiver;bool certificatesValid;public int ResidualCertificateHits;
  public SourceContactBody Source;
  public Vec3[] AnimatedSource;
  public Vec3[] WorldSourceOverride;
  public AnatomicalPathConstraintSystem Path;
  public int Queries,Count,ClassifierQueries;
  public double MaximumViolation;
  double sign=1;
  public CavityContactConstraints(Vec3[] receiver,int[] faces,Vec3[] sourceReference,int sampleCount=512,int[] sourceFaces=null)
  {
   triangles=(int[])faces.Clone();surface=new TriangleSurface(receiver,faces,SurfaceKind.OrientedShell);surface.InitializeIncrementalRefit();
   var edges=new Dictionary<long,int>();var directed=new Dictionary<long,EdgeConstraint>();for(int t=0;t<faces.Length;t+=3)for(int j=0;j<3;j++){int a=faces[t+j],b=faces[t+(j+1)%3];long key=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b);int n;edges.TryGetValue(key,out n);edges[key]=n+1;directed[key]=new EdgeConstraint(a,b);}
   var boundaryNeighbors=new Dictionary<int,List<int>>();foreach(var e in edges)if(e.Value==1){var ab=directed[e.Key];if(!boundaryNeighbors.ContainsKey(ab.A))boundaryNeighbors[ab.A]=new List<int>();if(!boundaryNeighbors.ContainsKey(ab.B))boundaryNeighbors[ab.B]=new List<int>();boundaryNeighbors[ab.A].Add(ab.B);boundaryNeighbors[ab.B].Add(ab.A);}
   boundaryOfVertex=new int[receiver.Length];for(int i=0;i<boundaryOfVertex.Length;i++)boundaryOfVertex[i]=-1;var loops=new List<int[]>();var visited=new HashSet<int>();foreach(var pair in boundaryNeighbors){if(visited.Contains(pair.Key))continue;var loop=new List<int>();int first=pair.Key,current=first,previous=-1;do{if(boundaryNeighbors[current].Count!=2)throw new ArgumentException("Confinement requires simple boundary loops");loop.Add(current);visited.Add(current);int walkNext=boundaryNeighbors[current][0]==previous?boundaryNeighbors[current][1]:boundaryNeighbors[current][0];previous=current;current=walkNext;}while(current!=first);int l=loops.Count;foreach(int v in loop)boundaryOfVertex[v]=l;loops.Add(loop.ToArray());}
   boundaries=loops.ToArray();classificationPositions=new Vec3[receiver.Length+boundaries.Length];Array.Copy(receiver,classificationPositions,receiver.Length);var closed=new List<int>(faces);for(int l=0;l<boundaries.Length;l++){int extra=receiver.Length+l;foreach(int v in boundaries[l])classificationPositions[extra]+=receiver[v]/boundaries[l].Length;foreach(var e in edges)if(e.Value==1){var ab=directed[e.Key];if(boundaryOfVertex[ab.A]==l){closed.Add(ab.B);closed.Add(ab.A);closed.Add(extra);}}}
   classifier=new TriangleSurface(classificationPositions,closed.ToArray(),SurfaceKind.ClosedVolume){RobustSign=true};classifier.InitializeIncrementalRefit();
   int count=Math.Min(sampleCount,sourceReference.Length);samples=new int[count];hints=new int[count];multipliers=new double[count];var distances=new double[sourceReference.Length];for(int i=0;i<distances.Length;i++)distances[i]=double.PositiveInfinity;
   contactFaces=new int[count];contactNormals=new Vec3[count];barycentrics=new Vec3[count];valid=new bool[count];
   int next=0;for(int s=0;s<count;s++){samples[s]=next;hints[s]=-1;Vec3 point=sourceReference[next];double maximum=-1;for(int i=0;i<sourceReference.Length;i++){distances[i]=Math.Min(distances[i],(sourceReference[i]-point).LengthSquared);if(distances[i]>maximum){maximum=distances[i];next=i;}}}
   correctedSource=new Vec3[sourceReference.Length];coverRadii=new double[count];if(sourceFaces!=null){sourceTriangles=(int[])sourceFaces.Clone();triangleOwners=new int[sourceFaces.Length/3];for(int t=0;t<triangleOwners.Length;t++){double best=double.PositiveInfinity;int owner=0;for(int s=0;s<count;s++){double radius=0;for(int j=0;j<3;j++)radius=Math.Max(radius,(sourceReference[sourceFaces[t*3+j]]-sourceReference[samples[s]]).LengthSquared);if(radius<best){best=radius;owner=s;}}triangleOwners[t]=owner;}}
   vertexOwners=new int[sourceReference.Length];vertexHints=new int[sourceReference.Length];contactVertices=(int[])samples.Clone();worstVertices=new int[count];worstHits=new SurfaceSample[count];worstClearance=new double[count];for(int i=0;i<vertexOwners.Length;i++){double best=double.PositiveInfinity;vertexHints[i]=-1;for(int s=0;s<count;s++){double d=(sourceReference[i]-sourceReference[samples[s]]).LengthSquared;if(d<best){best=d;vertexOwners[i]=s;}}}
   clearanceBounds=new double[sourceReference.Length];checkedSource=new Vec3[sourceReference.Length];checkedReceiver=new Vec3[receiver.Length];
  }
  public void BeginStep(){Array.Clear(multipliers,0,multipliers.Length);Queries=Count=Builds=ClassifierQueries=ResidualSearchQueries=ResidualBuilds=ResidualCertificateHits=0;MaximumViolation=MaximumTrueViolation=BuildMilliseconds=ProjectionMilliseconds=ResidualMilliseconds=0;built=false;}
  public double RefreshResidual(Vec3[] positions){Array.Clear(multipliers,0,multipliers.Length);if(UseExactSurfaceCoverage)ExactResidual(positions);else Build(positions,true);return MaximumTrueViolation;}
  public void Orient(Vec3[] receiver,AnatomicalPathConstraintSystem path)
  {
   surface.Refit(receiver);int positive=0,negative=0;for(int i=1;i<path.Count-1;i++){SurfaceSample hit;if(surface.TrySample(path[i],1,out hit)){if(hit.SignedDistance>=0)positive++;else negative++;}}sign=positive>=negative?1:-1;
  }
  static Vec3 Corrected(SourceContactBody source,int vertex,Vec3 p)
  {
   var indexed=source.Deformation as IIndexedSourceDeformation;
   Vec3 q=indexed==null?(source.Deformation==null?p:source.Deformation.Map(p)):indexed.MapVertex(vertex,p);
   return source.Pivot+source.Rotate(q-source.Pivot)+source.Offset;
  }
  Vec3 Point(int vertex){if(WorldSourceOverride==null)return Corrected(Source,vertex,AnimatedSource[vertex]);return Source.Pivot+Source.Rotate(WorldSourceOverride[vertex]-Source.Pivot)+Source.Offset;}
  public void Project(Vec3[] positions,Vec3[] anchors,double[] inverseMass,double offsetLimit,BidirectionalContactSettings settings,double h)
  {
   if(Source==null||AnimatedSource==null||Path==null)return;
   if(UseCachedContacts){ProjectCached(positions,anchors,inverseMass,offsetLimit,settings,h);return;}
   surface.Refit(positions);var start=Path.AtDistance(0);var end=Path.AtDistance(Path.Length);Count=0;
   if(UseVolumeClassification){Array.Copy(positions,classificationPositions,positions.Length);for(int l=0;l<boundaries.Length;l++){Vec3 centre=new Vec3();foreach(int i in boundaries[l])centre+=positions[i]/boundaries[l].Length;classificationPositions[positions.Length+l]=centre;}classifier.Refit(classificationPositions);}
   double alpha=settings.TissueCompliance/(h*h),margin=.0002;
   for(int s=0;s<samples.Length;s++)
   {
    int vertex=samples[s];Vec3 point=Point(vertex);
    var closest=Path.Closest(point);if((closest.Parameter<1e-5&&Vec3.Dot(point-start.Point,start.Tangent)<0)||(closest.Parameter>1-1e-5&&Vec3.Dot(point-end.Point,end.Tangent)>0))continue;
    Queries++;SurfaceSample hit;if(!surface.TrySampleHinted(point,.25,ref hints[s],out hit))continue;
    double signed=hit.SignedDistance*sign;if(UseVolumeClassification){SurfaceSample volume;if(!classifier.TrySample(point,1,out volume))continue;signed=volume.SignedDistance<=0?Math.Abs(hit.SignedDistance):-Math.Abs(hit.SignedDistance);}double gap=signed-margin;if(gap>=0&&multipliers[s]<=0)continue;
    // The signed distance is Euclidean, including at boundary edges/vertices.
    // There its derivative is the signed closest-point direction, not the face
    // normal. A face normal would spend an axial end-edge correction sideways.
    Vec3 difference=point-hit.Point;double distance=difference.Length;
    Vec3 n=distance>1e-12?difference*((signed>=0?1:-1)/distance):hit.Normal*sign;int t=hit.Triangle*3,a=triangles[t],b=triangles[t+1],c=triangles[t+2];
    Vec3 ab=positions[b]-positions[a],ac=positions[c]-positions[a],aq=hit.Point-positions[a];double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),den=aa*cc-bb*bb;if(Math.Abs(den)<1e-24)continue;
    double v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/den,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/den,u=1-v-w;
    Vec3 gradient=Source.ProjectContactGradient(n);double wa=inverseMass[a]*settings.TissueDeformationStrength,wb=inverseMass[b]*settings.TissueDeformationStrength,wc=inverseMass[c]*settings.TissueDeformationStrength;
    double wr=wa*u*u+wb*v*v+wc*w*w,ws=Source.InverseMass*gradient.LengthSquared;
    double next=BidirectionalContactSolver.Multiplier(gap,multipliers[s],wr,ws,alpha),delta=next-multipliers[s];multipliers[s]=next;
    if(delta>0){Vec3 before=Source.Offset;Source.Correct(gradient*(Source.InverseMass*delta));double sourceAccepted=Math.Max(0,Vec3.Dot(Source.Offset-before,n));if(wr>1e-20)delta+=Math.Max(0,ws*delta-sourceAccepted)/wr;positions[a]=anchors[a]+Vec3.ClampLength(positions[a]-anchors[a]-n*(wa*u*delta),offsetLimit);positions[b]=anchors[b]+Vec3.ClampLength(positions[b]-anchors[b]-n*(wb*v*delta),offsetLimit);positions[c]=anchors[c]+Vec3.ClampLength(positions[c]-anchors[c]-n*(wc*w*delta),offsetLimit);surface.RefitVertices(positions,a,b,c);if(UseVolumeClassification){classificationPositions[a]=positions[a];classificationPositions[b]=positions[b];classificationPositions[c]=positions[c];classifier.RefitVertices(classificationPositions,a,b,c);for(int l=0;l<boundaries.Length;l++)if(boundaryOfVertex[a]==l||boundaryOfVertex[b]==l||boundaryOfVertex[c]==l){int extra=positions.Length+l;Vec3 centre=new Vec3();foreach(int i in boundaries[l])centre+=positions[i]/boundaries[l].Length;classificationPositions[extra]=centre;classifier.RefitVertices(classificationPositions,extra,extra,extra);}}}
    if(next>0){Count++;Source.MarkContact();}MaximumViolation=Math.Max(MaximumViolation,Math.Max(0,-gap));
   }
  }
  void ProjectCached(Vec3[] positions,Vec3[] anchors,double[] inverseMass,double offsetLimit,BidirectionalContactSettings settings,double h)
  {
   if(!built)Build(positions);
   long started=System.Diagnostics.Stopwatch.GetTimestamp();Count=0;MaximumViolation=0;double alpha=settings.TissueCompliance/(h*h);
   for(int s=0;s<samples.Length;s++)if(valid[s]){
    int vertex=UseExactSurfaceCoverage?contactVertices[s]:samples[s];int t=contactFaces[s]*3,a=triangles[t],b=triangles[t+1],c=triangles[t+2];Vec3 bary=barycentrics[s],n=contactNormals[s],point=Point(vertex),q=positions[a]*bary.X+positions[b]*bary.Y+positions[c]*bary.Z;double gap=Vec3.Dot(point-q,n)-.0002-(UseConservativeProxy&&!UseExactSurfaceCoverage?coverRadii[s]:0);
    Vec3 gradient=Source.ProjectContactGradient(n);double wa=inverseMass[a]*settings.TissueDeformationStrength,wb=inverseMass[b]*settings.TissueDeformationStrength,wc=inverseMass[c]*settings.TissueDeformationStrength,wr=wa*bary.X*bary.X+wb*bary.Y*bary.Y+wc*bary.Z*bary.Z,ws=Source.InverseMass*gradient.LengthSquared;
    double next=BidirectionalContactSolver.Multiplier(gap,multipliers[s],wr,ws,alpha),delta=next-multipliers[s];multipliers[s]=next;
    if(delta>0){Vec3 before=Source.Offset;Source.Correct(gradient*(Source.InverseMass*delta));double accepted=Math.Max(0,Vec3.Dot(Source.Offset-before,n));if(wr>1e-20)delta+=Math.Max(0,ws*delta-accepted)/wr;positions[a]=anchors[a]+Vec3.ClampLength(positions[a]-anchors[a]-n*(wa*bary.X*delta),offsetLimit);positions[b]=anchors[b]+Vec3.ClampLength(positions[b]-anchors[b]-n*(wb*bary.Y*delta),offsetLimit);positions[c]=anchors[c]+Vec3.ClampLength(positions[c]-anchors[c]-n*(wc*bary.Z*delta),offsetLimit);}
    if(next>0){Count++;Source.MarkContact();}MaximumViolation=Math.Max(MaximumViolation,Math.Max(0,-gap));
   }
   ProjectionMilliseconds+=(System.Diagnostics.Stopwatch.GetTimestamp()-started)*1000.0/System.Diagnostics.Stopwatch.Frequency;
  }
  void Build(Vec3[] positions,bool audit=false)
  {
   long started=System.Diagnostics.Stopwatch.GetTimestamp();built=true;if(audit)ResidualBuilds++;else Builds++;MaximumTrueViolation=0;surface.Refit(positions);Array.Clear(valid,0,valid.Length);var start=Path.AtDistance(0);var end=Path.AtDistance(Path.Length);
   MaximumCoverRadius=0;Array.Clear(coverRadii,0,coverRadii.Length);if(UseConservativeProxy&&!UseExactSurfaceCoverage){if(sourceTriangles==null)throw new InvalidOperationException("A conservative surface proxy requires original indexed source triangles");for(int i=0;i<correctedSource.Length;i++)correctedSource[i]=Point(i);for(int t=0;t<triangleOwners.Length;t++){int owner=triangleOwners[t];Vec3 centre=correctedSource[samples[owner]];for(int j=0;j<3;j++)coverRadii[owner]=Math.Max(coverRadii[owner],(correctedSource[sourceTriangles[t*3+j]]-centre).LengthSquared);}for(int s=0;s<coverRadii.Length;s++){coverRadii[s]=Math.Sqrt(coverRadii[s]);MaximumCoverRadius=Math.Max(MaximumCoverRadius,coverRadii[s]);}}
   if(UseExactSurfaceCoverage)classifier.RobustSign=false;
   if(UseVolumeClassification){Array.Copy(positions,classificationPositions,positions.Length);for(int l=0;l<boundaries.Length;l++){Vec3 centre=new Vec3();foreach(int i in boundaries[l])centre+=positions[i]/boundaries[l].Length;classificationPositions[positions.Length+l]=centre;}classifier.Refit(classificationPositions);}
   for(int s=0;s<samples.Length;s++){
    int vertex=UseExactSurfaceCoverage?contactVertices[s]:samples[s];Vec3 p=Point(vertex);if(!WholeSourceCoverage){var closest=Path.Closest(p);if((closest.Parameter<1e-5&&Vec3.Dot(p-start.Point,start.Tangent)<0)||(closest.Parameter>1-1e-5&&Vec3.Dot(p-end.Point,end.Tangent)>0))continue;}SurfaceSample hit;double signed;
    if(UseVolumeClassification){SurfaceSample volume;if(audit)ResidualSearchQueries++;else ClassifierQueries++;if(!classifier.TrySample(p,1,out volume))continue;if(volume.Triangle<triangles.Length/3){hit=volume;signed=-volume.SignedDistance;}else{if(audit)ResidualSearchQueries++;else Queries++;if(!surface.TrySampleHinted(p,.25,ref hints[s],out hit))continue;signed=volume.SignedDistance<=0?Math.Abs(hit.SignedDistance):-Math.Abs(hit.SignedDistance);}}
    else{if(audit)ResidualSearchQueries++;else Queries++;if(!surface.TrySampleHinted(p,.25,ref hints[s],out hit))continue;signed=hit.SignedDistance*sign;}
    MaximumTrueViolation=Math.Max(MaximumTrueViolation,.0002+(UseConservativeProxy?coverRadii[s]:0)-signed);if(signed>.001+(UseConservativeProxy?coverRadii[s]:0))continue;
    int t=hit.Triangle*3,a=triangles[t],b=triangles[t+1],c=triangles[t+2];Vec3 ab=positions[b]-positions[a],ac=positions[c]-positions[a],aq=hit.Point-positions[a];double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),den=aa*cc-bb*bb;if(Math.Abs(den)<1e-24)continue;double v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/den,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/den;Vec3 difference=p-hit.Point;double distance=difference.Length;
    valid[s]=true;contactFaces[s]=hit.Triangle;barycentrics[s]=new Vec3(1-v-w,v,w);contactNormals[s]=distance>1e-12?difference*((signed>=0?1:-1)/distance):hit.Normal*sign;
   }
   double milliseconds=(System.Diagnostics.Stopwatch.GetTimestamp()-started)*1000.0/System.Diagnostics.Stopwatch.Frequency;if(audit)ResidualMilliseconds+=milliseconds;else BuildMilliseconds+=milliseconds;
  }
  void ExactResidual(Vec3[] positions)
  {
   long started=System.Diagnostics.Stopwatch.GetTimestamp();ResidualBuilds++;MaximumTrueViolation=0;Array.Copy(positions,classificationPositions,positions.Length);for(int l=0;l<boundaries.Length;l++){Vec3 centre=new Vec3();foreach(int i in boundaries[l])centre+=positions[i]/boundaries[l].Length;classificationPositions[positions.Length+l]=centre;}classifier.RobustSign=false;classifier.Refit(classificationPositions);surface.Refit(positions);
   double receiverMotion=0;if(certificatesValid)for(int i=0;i<positions.Length;i++)receiverMotion=Math.Max(receiverMotion,(positions[i]-checkedReceiver[i]).Length);
   for(int s=0;s<samples.Length;s++){worstClearance[s]=double.PositiveInfinity;worstVertices[s]=-1;valid[s]=false;}
   for(int i=0;i<AnimatedSource.Length;i++){Vec3 point=Point(i);correctedSource[i]=point;double bound=clearanceBounds[i]-receiverMotion-(point-checkedSource[i]).Length;if(certificatesValid&&bound>=.0002){clearanceBounds[i]=bound;checkedSource[i]=point;ResidualCertificateHits++;continue;}SurfaceSample hit;ResidualSearchQueries++;if(!classifier.TrySampleHinted(point,1,ref vertexHints[i],out hit))continue;double clearance=-hit.SignedDistance;clearanceBounds[i]=clearance;checkedSource[i]=point;MaximumTrueViolation=Math.Max(MaximumTrueViolation,.0002-clearance);int owner=vertexOwners[i];if(clearance<worstClearance[owner]){worstClearance[owner]=clearance;worstVertices[owner]=i;worstHits[owner]=hit;}}
   Array.Copy(positions,checkedReceiver,positions.Length);certificatesValid=true;
   for(int s=0;s<samples.Length;s++){int vertex=worstVertices[s];if(vertex<0||worstClearance[s]>.001)continue;Vec3 p=correctedSource[vertex];SurfaceSample hit=worstHits[s];double signed=worstClearance[s];if(hit.Triangle>=triangles.Length/3){ResidualSearchQueries++;if(!surface.TrySampleHinted(p,1,ref hints[s],out hit))continue;signed=signed>=0?Math.Abs(hit.SignedDistance):-Math.Abs(hit.SignedDistance);}int t=hit.Triangle*3,a=triangles[t],b=triangles[t+1],c=triangles[t+2];Vec3 ab=positions[b]-positions[a],ac=positions[c]-positions[a],aq=hit.Point-positions[a];double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),den=aa*cc-bb*bb;if(Math.Abs(den)<1e-24)continue;double v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/den,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/den;Vec3 delta=p-hit.Point;double distance=delta.Length;valid[s]=true;contactVertices[s]=vertex;contactFaces[s]=hit.Triangle;barycentrics[s]=new Vec3(1-v-w,v,w);contactNormals[s]=distance>1e-12?delta*((signed>=0?1:-1)/distance):hit.Normal*sign;}
   built=true;ResidualMilliseconds+=(System.Diagnostics.Stopwatch.GetTimestamp()-started)*1000.0/System.Diagnostics.Stopwatch.Frequency;
  }
 }
}


