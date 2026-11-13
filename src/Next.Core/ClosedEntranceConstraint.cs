using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    [Serializable] public sealed class ClosedEntranceDefinition
    {
        public int Schema,SourceVertexStart,SourceVertexCount,OriginalFaceCount;
        public int[] Controls,Boundary,AnchorA,AnchorB,Triangles,RingA,RingB,RemovedInnerFaces;
        public double[] AnchorT,Distance,RingT,Influence;
        public int[] BlendOffsets,BlendIndices;
        public double[] BlendWeights;
        public string NativeRestSHA256;
        public void Validate(SimulationDefinition support,SimulationDefinition physical)
        {
            int n=Controls==null?0:Controls.Length,m=Boundary==null?0:Boundary.Length;
            if(Schema!=1||n<3||m<3||AnchorA.Length!=n||AnchorB.Length!=n||AnchorT.Length!=n||Distance.Length!=n||RingA.Length!=m||RingB.Length!=m||RingT.Length!=m||Triangles.Length%3!=0)throw new ArgumentException("Closed entrance contract differs.");
            var seen=new HashSet<int>();foreach(int i in Controls)if(i<0||i>=support.Rest.Length||!seen.Add(i))throw new ArgumentException("Closed entrance control identity differs.");
            for(int i=0;i<m;i++)if(!seen.Contains(Boundary[i])||RingA[i]<0||RingB[i]<0||RingA[i]>=physical.Rest.Length||RingB[i]>=physical.Rest.Length||!Vec3.Finite(RingT[i])||RingT[i]<0||RingT[i]>1)throw new ArgumentException("Closed entrance boundary differs.");
            for(int i=0;i<n;i++)if(AnchorA[i]<0||AnchorA[i]>=m||AnchorB[i]<0||AnchorB[i]>=m||!Vec3.Finite(AnchorT[i])||AnchorT[i]<0||AnchorT[i]>1||!Vec3.Finite(Distance[i])||Distance[i]<0)throw new ArgumentException("Closed entrance surface mapping differs.");
            foreach(int i in Triangles)if(i<0||i>=n)throw new ArgumentException("Closed entrance topology differs.");
            if(Influence!=null){if(Influence.Length!=n)throw new ArgumentException("Closed entrance influence count differs.");foreach(double w in Influence)if(!Vec3.Finite(w)||w<0||w>1)throw new ArgumentException("Closed entrance influence differs.");}
            if(BlendOffsets!=null){if(BlendOffsets.Length!=n+1||BlendIndices==null||BlendWeights==null||BlendWeights.Length!=BlendIndices.Length||BlendOffsets[n]!=BlendIndices.Length)throw new ArgumentException("Closed entrance geodesic transfer differs.");for(int i=0;i<n;i++){double sum=0;for(int j=BlendOffsets[i];j<BlendOffsets[i+1];j++){if(BlendIndices[j]<0||BlendIndices[j]>=m||!Vec3.Finite(BlendWeights[j])||BlendWeights[j]<0)throw new ArgumentException("Invalid closed entrance geodesic weight.");sum+=BlendWeights[j];}if(Math.Abs(sum-1)>1e-7)throw new ArgumentException("Unnormalized closed entrance transfer.");}}
        }
    }

    // Two apposed material lips are opened by a cached boundary-to-surface
    // attachment. The native SAC folds are the per-frame animation baseline.
    // No projection onto the Profundis exterior occurs here.
    public sealed class ClosedEntranceConstraint
    {
        readonly ClosedEntranceDefinition d;
        readonly int[] boundaryLocal;
        readonly Vec3[] delta,release,proposed,desiredBoundary;
        readonly Vec3[] referenceNormals;
        readonly double[] referenceAreas,mobility,edgeLimits;
        readonly double[] boundaryAngles;
        readonly int[] materialIdentity;
        readonly Vec3[] harmonic,repairMemory;
        double previousAdmission;
        Vec3 projectionAxis;
        EntranceContactProxy contactProxy;
        readonly EdgeConstraint[] edges;
        readonly double[] restLengths;
        readonly SurfaceOffsetFairing fairing;
        bool engaged,releasing;
        double releaseAge;
        public readonly Vec3[] Offsets;
        public readonly double ContactDepth;
        public double MaximumOffset,MinimumAreaRatio=1,MaximumStrain,Admission,RequiredRadius,AppliedScale=1;
        public int GuardBacktracks;
        public int LimitingTriangle=-1,LimitingEdge=-1;
        public double RejectedArea,RejectedStrain;
        public bool RepairEnabled=true;
        public bool FairingEnabled=true;
        public int RepairSweeps,RepairedFaces;
        public double MaximumBoundaryResidual,CentreCorrection;
        public double MaximumContactResidual;public int ContactProjections;

        public ClosedEntranceConstraint(ClosedEntranceDefinition definition,SimulationDefinition support,SimulationDefinition physical,ReceiverGuideDefinition guide=null)
        {
            definition.Validate(support,physical);d=definition;boundaryLocal=new int[d.Boundary.Length];var lookup=new Dictionary<int,int>();for(int i=0;i<d.Controls.Length;i++)lookup.Add(d.Controls[i],i);
            for(int i=0;i<boundaryLocal.Length;i++)boundaryLocal[i]=lookup[d.Boundary[i]];
            delta=new Vec3[boundaryLocal.Length];release=new Vec3[delta.Length];proposed=new Vec3[d.Controls.Length];Offsets=new Vec3[proposed.Length];
            desiredBoundary=new Vec3[delta.Length];
            referenceNormals=new Vec3[d.Triangles.Length/3];referenceAreas=new double[referenceNormals.Length];mobility=new double[proposed.Length];
            harmonic=new Vec3[proposed.Length];repairMemory=new Vec3[proposed.Length];materialIdentity=new int[proposed.Length];
            for(int i=0;i<materialIdentity.Length;i++)
            {
                materialIdentity[i]=i;int a=d.Controls[i];for(int j=0;j<i;j++){int b=d.Controls[j];if((support.Rest[a]-support.Rest[b]).LengthSquared<1e-22&&SameSkin(support,a,b)){materialIdentity[i]=materialIdentity[j];break;}}
            }
            for(int i=0;i<mobility.Length;i++)mobility[i]=d.Influence==null||d.Influence[i]>1e-8?1:0;
            foreach(int i in boundaryLocal)mobility[i]=0;
            foreach(int i in boundaryLocal)mobility[materialIdentity[i]]=0;
            if(guide!=null)
            {
                boundaryAngles=new double[delta.Length];Vec3 centre=new Vec3(),next=new Vec3();foreach(int i in guide.Boundaries[0])centre+=physical.Rest[i]/guide.Boundaries[0].Length;var section=guide.Contours[1];for(int j=0;j<section.Vertices.Length;j++)next+=physical.Rest[section.Vertices[j]]*section.Weights[j];
                Vec3 n=(next-centre).Normalized,x=(new Vec3(1,0,0)-n*n.X).Normalized,y=Vec3.Cross(n,x);double rx=0,ry=0;
                for(int i=0;i<d.Controls.Length;i++)if(d.Distance[i]<.035)ContactDepth=Math.Max(ContactDepth,Math.Abs(Vec3.Dot(support.Rest[d.Controls[i]]-centre,n)));ContactDepth=Math.Max(.01,ContactDepth*1.1);
                foreach(int i in guide.Boundaries[0]){Vec3 q=physical.Rest[i]-centre;rx=Math.Max(rx,Math.Abs(Vec3.Dot(q,x)));ry=Math.Max(ry,Math.Abs(Vec3.Dot(q,y)));}
                for(int j=0;j<boundaryAngles.Length;j++){Vec3 q=physical.Rest[d.RingA[j]]*(1-d.RingT[j])+physical.Rest[d.RingB[j]]*d.RingT[j]-centre;boundaryAngles[j]=Math.Atan2(Vec3.Dot(q,y)/Math.Max(1e-9,ry),Vec3.Dot(q,x)/Math.Max(1e-9,rx));}
                // The two tips stay on the median plane. Within each material
                // lip use arc length, rather than the polar angle of a nearly
                // closed slit (whose derivative is unbounded near its sides).
                int top=0,bottom=0;for(int j=1;j<boundaryAngles.Length;j++){if(Math.Sin(boundaryAngles[j])>Math.Sin(boundaryAngles[top]))top=j;if(Math.Sin(boundaryAngles[j])<Math.Sin(boundaryAngles[bottom]))bottom=j;}
                ParameterizeLip(support,top,bottom,Math.PI*.5);ParameterizeLip(support,bottom,top,Math.PI*1.5);
            }
            var edgeSet=new HashSet<long>();var edgeList=new List<EdgeConstraint>();
            for(int f=0;f<d.Triangles.Length;f+=3)for(int k=0;k<3;k++){int a=d.Triangles[f+k],b=d.Triangles[f+(k+1)%3];if(a>b){int t=a;a=b;b=t;}if(a==b||!edgeSet.Add(((long)a<<32)|(uint)b))continue;edgeList.Add(new EdgeConstraint(a,b));}
            edges=edgeList.ToArray();restLengths=new double[edges.Length];edgeLimits=new double[edges.Length];for(int i=0;i<edges.Length;i++)restLengths[i]=(support.Rest[d.Controls[edges[i].A]]-support.Rest[d.Controls[edges[i].B]]).Length;
            var materialRest=new Vec3[d.Controls.Length];for(int i=0;i<materialRest.Length;i++)materialRest[i]=support.Rest[d.Controls[i]];fairing=new SurfaceOffsetFairing(materialRest,edges,materialIdentity,mobility);
        }
        static bool SameSkin(SimulationDefinition support,int a,int b)
        {
            int first=support.InfluenceOffsets[a],last=support.InfluenceOffsets[a+1],other=support.InfluenceOffsets[b];if(last-first!=support.InfluenceOffsets[b+1]-other)return false;
            for(int k=0;k<last-first;k++)if(support.BoneIndices[first+k]!=support.BoneIndices[other+k]||support.BoneWeights[first+k]!=support.BoneWeights[other+k])return false;
            if(support.Morphs!=null)foreach(var morph in support.Morphs)if((morph.Deltas[a]-morph.Deltas[b]).LengthSquared>1e-22)return false;
            return true;
        }
        void ParameterizeLip(SimulationDefinition support,int start,int end,double angle)
        {
            if(start==end)return;double length=0;int count=boundaryLocal.Length;
            for(int i=start;i!=end;i=(i+1)%count)length+=(support.Rest[d.Boundary[(i+1)%count]]-support.Rest[d.Boundary[i]]).Length;
            if(length<1e-12)return;double arc=0;
            for(int i=start;i!=end;i=(i+1)%count){boundaryAngles[i]=angle+Math.PI*arc/length;arc+=(support.Rest[d.Boundary[(i+1)%count]]-support.Rest[d.Boundary[i]]).Length;}
        }
        static double Smooth(double x){x=Math.Max(0,Math.Min(1,x));return x*x*(3-2*x);}
        public void Apply(Vec3[] native,Vec3[] physical,int[] ring,Vec3 axis,Vec3 side,bool active,double dt,double response,double recovery,double maxStrain,double materialScale,double[] radii=null,double profileRotation=0,Vec3? contactCentre=null,EntranceContactProxy proxy=null)
        {
            if(native.Length!=Offsets.Length||ring.Length<3)throw new ArgumentException("Closed entrance input differs.");
            if(!Vec3.Finite(dt)||dt<0||!Vec3.Finite(response)||response<0||!Vec3.Finite(recovery)||recovery<0||!Vec3.Finite(maxStrain)||maxStrain<0||!Vec3.Finite(materialScale)||materialScale<=0||!axis.IsFinite||axis.LengthSquared<1e-20||!side.IsFinite||contactCentre.HasValue&&!contactCentre.Value.IsFinite||!Vec3.Finite(profileRotation))throw new ArgumentException("Invalid closed entrance frame.");
            foreach(var point in native)if(!point.IsFinite)throw new ArgumentException("Nonfinite native entrance.");
            foreach(int i in ring)if(i<0||i>=physical.Length||!physical[i].IsFinite)throw new ArgumentException("Invalid entrance ring.");
            if(radii!=null){if(radii.Length!=ReceiverRadialField.Directions)throw new ArgumentException("Invalid directional contact profile.");foreach(double r in radii)if(!Vec3.Finite(r)||r<0)throw new ArgumentException("Invalid contact radius.");}
            dt=Math.Max(0,Math.Min(.1,dt));axis=axis.Normalized;side=(side-axis*Vec3.Dot(side,axis)).Normalized;Vec3 up=Vec3.Cross(axis,side).Normalized;
            if(side.LengthSquared<.5)throw new ArgumentException("Entrance side is parallel to its normal.");
            Vec3 centre=new Vec3(),nativeCentre=new Vec3();foreach(int i in ring)centre+=physical[i]/ring.Length;foreach(int i in boundaryLocal)nativeCentre+=native[i]/boundaryLocal.Length;
            if(contactCentre.HasValue)centre=contactCentre.Value;
            Vec3 targetCentre=centre+axis*Vec3.Dot(nativeCentre-centre,axis);
            contactProxy=active&&proxy!=null&&proxy.Active?proxy:null;ContactProjections=0;
            if(contactProxy!=null)contactProxy.RefreshBounds();
            CentreCorrection=(targetCentre-nativeCentre).Length;
            RequiredRadius=0;foreach(int i in ring){Vec3 q=physical[i]-centre;q-=axis*Vec3.Dot(q,axis);RequiredRadius=Math.Max(RequiredRadius,q.Length);}
            if(radii!=null)foreach(double radius in radii)RequiredRadius=Math.Max(RequiredRadius,radius);
            double alpha=1-Math.Exp(-Math.Max(18,Math.Max(0,response)*4)*dt);
            if(active)
            {
                engaged=true;releasing=false;releaseAge=0;
                for(int i=0;i<delta.Length;i++)
                {
                    Vec3 target=physical[d.RingA[i]]*(1-d.RingT[i])+physical[d.RingB[i]]*d.RingT[i]-centre;
                    if(boundaryAngles!=null)
                    {
                        double angle=boundaryAngles[i],r=RequiredRadius;
                        if(radii!=null&&radii.Length>2){double v=(angle-profileRotation)*radii.Length/(2*Math.PI);v-=Math.Floor(v/radii.Length)*radii.Length;int a=Math.Min(radii.Length-1,(int)v),b=(a+1)%radii.Length;double t=v-a;r=radii[a]*(1-t)+radii[b]*t;}
                        target=(side*Math.Cos(angle)+up*Math.Sin(angle))*r;
                    }
                    if(contactProxy!=null){Vec3 point=targetCentre+target+axis*Vec3.Dot(native[boundaryLocal[i]]-nativeCentre,axis);target+=contactProxy.Correction(point,1);}
                    Vec3 old=native[boundaryLocal[i]]-targetCentre;
                    // Tangential opening only: the labia are never dragged
                    // longitudinally into the channel by the source guide.
                    Vec3 desired=new Vec3(Vec3.Dot(target-old,side),Vec3.Dot(target-old,up),0);
                    desiredBoundary[i]=side*desired.X+up*desired.Y;
                    delta[i]+=(desired-delta[i])*alpha;
                }
                Admission+=(1-Admission)*alpha;
            }
            else if(engaged||releasing)
            {
                if(!releasing){Array.Copy(delta,release,delta.Length);releaseAge=0;releasing=true;engaged=false;}
                releaseAge+=dt;double weight=recovery<=0?0:1-Smooth(releaseAge/recovery);for(int i=0;i<delta.Length;i++)delta[i]=release[i]*weight;Admission=weight;
                if(weight==0)releasing=false;
            }
            else{Array.Clear(delta,0,delta.Length);Admission=0;}
            projectionAxis=axis;
            // The transfer width grows with the actual opening to keep the
            // surrounding skin gradient bounded for different card sizes.
            double width=Math.Max(.035,Math.Min(.075,RequiredRadius*3))*Math.Max(.1,materialScale);
            for(int i=0;i<proposed.Length;i++)
            {
                double influence=d.Influence==null?1-Smooth(d.Distance[i]*materialScale/width):d.Influence[i];Vec3 q=new Vec3();
                if(d.BlendOffsets!=null)for(int k=d.BlendOffsets[i];k<d.BlendOffsets[i+1];k++)
                {
                    int j=d.BlendIndices[k];q+=delta[j]*d.BlendWeights[k];
                }
                else q=delta[d.AnchorA[i]]*(1-d.AnchorT[i])+delta[d.AnchorB[i]]*d.AnchorT[i];
                proposed[i]=(side*q.X+up*q.Y)*influence;
            }
            Array.Copy(proposed,harmonic,proposed.Length);
            if(RepairEnabled)
            {
                double memoryScale=previousAdmission>1e-9?Math.Min(1,Admission/previousAdmission):0;
                for(int i=0;i<proposed.Length;i++)if(mobility[materialIdentity[i]]>0)proposed[i]+=(side*repairMemory[i].X+up*repairMemory[i].Y)*memoryScale;
                foreach(int i in boundaryLocal)proposed[materialIdentity[i]]=proposed[i];
                for(int i=0;i<proposed.Length;i++)proposed[i]=proposed[materialIdentity[i]];
            }
            AppliedScale=1;GuardBacktracks=0;LimitingTriangle=LimitingEdge=-1;RejectedArea=RejectedStrain=0;
            if(FairingEnabled&&contactProxy!=null)fairing.Apply(proposed,4,.35);
            PrepareMaterial(native,maxStrain,materialScale);
            RepairSweeps=RepairedFaces=0;
            if(RepairEnabled)Repair(native,maxStrain,materialScale);
            if(RepairEnabled)for(int i=0;i<proposed.Length;i++){Vec3 q=proposed[i]-harmonic[i];repairMemory[i]=Admission>0?Vec3.ClampLength(new Vec3(Vec3.Dot(q,side),Vec3.Dot(q,up),0),Math.Max(.01,RequiredRadius*2)):new Vec3();}
            previousAdmission=Admission;
            LimitingTriangle=LimitingEdge=-1;RejectedArea=RejectedStrain=0;
            if(!Feasible(native,1,maxStrain,materialScale))
            {
                double high=1,low=.5;
                while(!Feasible(native,low,maxStrain,materialScale)&&GuardBacktracks<20){high=low;low*=.5;GuardBacktracks++;}if(GuardBacktracks>=20)low=0;
                // Halving is a bracket, not the final deformation amplitude.
                // A slightly infeasible aperture must not lose half its width.
                for(int k=0;k<10;k++){double mid=(low+high)*.5;if(Feasible(native,mid,maxStrain,materialScale))low=mid;else high=mid;GuardBacktracks++;}
                AppliedScale=low;Feasible(native,AppliedScale,maxStrain,materialScale);
            }
            MaximumOffset=0;for(int i=0;i<Offsets.Length;i++){Offsets[i]=proposed[i]*AppliedScale;MaximumOffset=Math.Max(MaximumOffset,Offsets[i].Length);}
            MaximumBoundaryResidual=0;if(active)for(int i=0;i<boundaryLocal.Length;i++)MaximumBoundaryResidual=Math.Max(MaximumBoundaryResidual,(Offsets[boundaryLocal[i]]-desiredBoundary[i]).Length);
            MaximumContactResidual=0;if(contactProxy!=null)for(int i=0;i<Offsets.Length;i++)if(d.Distance[i]<.035)MaximumContactResidual=Math.Max(MaximumContactResidual,contactProxy.Correction(native[i]+Offsets[i],Admission).Length);
        }
        void PrepareMaterial(Vec3[] native,double strain,double materialScale)
        {
            for(int f=0;f<referenceNormals.Length;f++)
            {
                int k=f*3,a=d.Triangles[k],b=d.Triangles[k+1],c=d.Triangles[k+2];Vec3 normal=Vec3.Cross(native[b]-native[a],native[c]-native[a]);double area=normal.Length;
                referenceAreas[f]=area;referenceNormals[f]=area>1e-12?normal/area:new Vec3();
            }
            for(int e=0;e<edges.Length;e++){double length=Math.Max((native[edges[e].A]-native[edges[e].B]).Length,restLengths[e]*materialScale);edgeLimits[e]=length*(1+Math.Max(0,strain)*.96);}
        }
        Vec3 Transverse(Vec3 v){return v-projectionAxis*Vec3.Dot(v,projectionAxis);}
        void Repair(Vec3[] native,double maxStrainForRepair,double materialScale)
        {
            // Boundary displacements are prescribed by contact. Move only free
            // neighbouring material nodes when one thin triangle blocks them;
            // never reduce the entire aperture for a locally repairable face.
            for(int pass=0;pass<64;pass++)
            {
                int changed=0;
                if(contactProxy!=null)for(int i=0;i<proposed.Length;i++)if(materialIdentity[i]==i&&mobility[i]>0&&d.Distance[i]<.035){Vec3 correction=contactProxy.Correction(native[i]+proposed[i],Admission);if(correction.LengthSquared>1e-12){proposed[i]+=Vec3.ClampLength(correction,.004*Math.Max(.1,materialScale));changed++;ContactProjections++;}}
                for(int f=0;f<referenceNormals.Length;f++)
                {
                    if(referenceAreas[f]<1e-12)continue;
                    int k=f*3,ia=d.Triangles[k],ib=d.Triangles[k+1],ic=d.Triangles[k+2],a=materialIdentity[ia],b=materialIdentity[ib],c=materialIdentity[ic];Vec3 n=referenceNormals[f],pa=native[ia]+proposed[a],pb=native[ib]+proposed[b],pc=native[ic]+proposed[c];
                    double value=Vec3.Dot(Vec3.Cross(pb-pa,pc-pa),n)-referenceAreas[f]*.06;if(value>=-referenceAreas[f]*1e-3)continue;
                    Vec3 ga=Transverse(Vec3.Cross(pb-pc,n)),gb=Transverse(Vec3.Cross(pc-pa,n)),gc=Transverse(Vec3.Cross(n,pb-pa));
                    double den=mobility[a]*ga.LengthSquared+mobility[b]*gb.LengthSquared+mobility[c]*gc.LengthSquared;if(den<1e-24)continue;
                    double lambda=-value/den,limit=.004*Math.Max(.1,materialScale);
                    proposed[a]+=Vec3.ClampLength(ga*(lambda*mobility[a]),limit);proposed[b]+=Vec3.ClampLength(gb*(lambda*mobility[b]),limit);proposed[c]+=Vec3.ClampLength(gc*(lambda*mobility[c]),limit);changed++;RepairedFaces++;
                }
                for(int e=0;e<edges.Length;e++)
                {
                    int ia=edges[e].A,ib=edges[e].B,a=materialIdentity[ia],b=materialIdentity[ib];Vec3 q=native[ib]+proposed[b]-native[ia]-proposed[a];double length=q.Length;if(length<=edgeLimits[e]+1e-7*Math.Max(.1,materialScale)||length<1e-12)continue;
                    Vec3 gradient=Transverse(q/length);double den=(mobility[a]+mobility[b])*gradient.LengthSquared;if(den<1e-16)continue;Vec3 correction=gradient*((length-edgeLimits[e])/den);
                    proposed[a]+=correction*mobility[a];proposed[b]-=correction*mobility[b];changed++;
                }
                for(int i=0;i<proposed.Length;i++)proposed[i]=proposed[materialIdentity[i]];
                double residual=0;if(contactProxy!=null)for(int i=0;i<proposed.Length;i++)if(materialIdentity[i]==i&&mobility[i]>0&&d.Distance[i]<.035)residual=Math.Max(residual,contactProxy.Correction(native[i]+proposed[i],Admission).Length);
                RepairSweeps=pass+1;if(changed==0||pass>=3&&residual<.00005*Math.Max(.1,materialScale)&&Feasible(native,1,maxStrainForRepair,materialScale))break;
            }
        }
        bool Feasible(Vec3[] native,double scale,double strain,double materialScale)
        {
            MinimumAreaRatio=1;MaximumStrain=0;
            for(int f=0;f<d.Triangles.Length;f+=3)
            {
                int a=d.Triangles[f],b=d.Triangles[f+1],c=d.Triangles[f+2];Vec3 n=Vec3.Cross(native[b]-native[a],native[c]-native[a]);double den=n.LengthSquared;if(den<1e-24)continue;
                Vec3 m=Vec3.Cross(native[b]+proposed[b]*scale-native[a]-proposed[a]*scale,native[c]+proposed[c]*scale-native[a]-proposed[a]*scale);
                double ratio=Vec3.Dot(n,m)/den;MinimumAreaRatio=Math.Min(MinimumAreaRatio,ratio);if(ratio<.03){LimitingTriangle=f/3;RejectedArea=ratio;return false;}
            }
            for(int i=0;i<edges.Length;i++)
            {
                int a=edges[i].A,b=edges[i].B;double rest=Math.Max((native[a]-native[b]).Length,restLengths[i]*materialScale);if(rest<1e-9)continue;double actual=(native[a]+proposed[a]*scale-native[b]-proposed[b]*scale).Length;double value=Math.Max(0,actual/rest-1);MaximumStrain=Math.Max(MaximumStrain,value);if(value>Math.Max(0,strain)+1e-5){LimitingEdge=i;RejectedStrain=value;return false;}
            }
            return true;
        }
    }
}
