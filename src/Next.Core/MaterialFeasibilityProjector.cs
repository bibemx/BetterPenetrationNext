using System;

namespace BetterPenetrationNext.Core
{
    // Incremental coordinate projection from the last accepted material state.
    // Each vertex ray respects every incident edge, face and motion sphere before
    // committing a correction. Work is bounded; a blocked target stays residual.
    // This is a material guard, not a replacement for external/self contact CCD.
    public sealed class MaterialFeasibilityProjector
    {
        readonly EdgeConstraint[] edges;
        readonly int[] faces,edgeOffsets,vertexEdges,faceOffsets,vertexFaces;
        readonly Vec3[] accepted;
        readonly double[] maximumEdge,hardMaximumEdge,minimumArea,hardMinimumArea;
        public const double MinimumAreaRatio=.01,ProjectionAreaRatio=.010002;
        Vec3[] anchors;
        double[] inverseMass;
        double radius;
        public int LastSweeps,LimitedVertices,BaselineResets;
        public double MaximumStrain;
        readonly double[] referenceEdge;
        public bool EnforceContinuousSelfContact;
        public bool TransferSelfContactReaction;
        public TriangleMotionGuard SelfContact {get;private set;}

        public MaterialFeasibilityProjector(SimulationTopology topology)
        {
            edges=topology.Stretch;faces=topology.Triangles;int n=topology.VertexCount;
            accepted=new Vec3[n];maximumEdge=new double[edges.Length];hardMaximumEdge=new double[edges.Length];referenceEdge=new double[edges.Length];minimumArea=new double[faces.Length/3];hardMinimumArea=new double[minimumArea.Length];
            edgeOffsets=new int[n+1];faceOffsets=new int[n+1];
            foreach(var e in edges){edgeOffsets[e.A+1]++;edgeOffsets[e.B+1]++;}
            foreach(int i in faces)faceOffsets[i+1]++;
            for(int i=1;i<=n;i++){edgeOffsets[i]+=edgeOffsets[i-1];faceOffsets[i]+=faceOffsets[i-1];}
            vertexEdges=new int[edgeOffsets[n]];vertexFaces=new int[faceOffsets[n]];
            var ec=(int[])edgeOffsets.Clone();var fc=(int[])faceOffsets.Clone();
            for(int i=0;i<edges.Length;i++){vertexEdges[ec[edges[i].A]++]=i;vertexEdges[ec[edges[i].B]++]=i;}
            for(int i=0;i<faces.Length;i++)vertexFaces[fc[faces[i]]++]=i/3;
        }
        public void BeginStep(Vec3[] baseline,Vec3[] attachment,double[] mobility,double[] edgeRest,double maximumStrain,double maximumOffset)
        {
            if(baseline.Length!=accepted.Length||attachment.Length!=accepted.Length||mobility.Length!=accepted.Length||edgeRest.Length!=edges.Length||maximumOffset<=0||maximumStrain<0)throw new ArgumentException("Invalid material projection state");
            anchors=attachment;inverseMass=mobility;radius=maximumOffset;LimitedVertices=BaselineResets=0;
            // Retain the existing solver's 0.1% projection safety allowance,
            // applied to permitted extension only (zero-strain mode stays valid).
            // A transported edge must not sit exactly at its hard bound: small
            // nonuniform changes in the captured float carrier otherwise make
            // the following step discard a valid accumulated deformation.
            for(int e=0;e<edges.Length;e++){referenceEdge[e]=edgeRest[e];hardMaximumEdge[e]=edgeRest[e]*(1+maximumStrain);maximumEdge[e]=edgeRest[e]*(1+maximumStrain*.999);}
            for(int t=0;t<minimumArea.Length;t++){int k=t*3;double area=Vec3.Cross(anchors[faces[k+1]]-anchors[faces[k]],anchors[faces[k+2]]-anchors[faces[k]]).Length;minimumArea[t]=area*ProjectionAreaRatio;hardMinimumArea[t]=area*MinimumAreaRatio;}
            for(int i=0;i<accepted.Length;i++)accepted[i]=anchors[i]+Vec3.ClampLength(baseline[i]-anchors[i],mobility[i]>0?radius:0);
            if(!Feasible(accepted))
            {
                Array.Copy(anchors,accepted,accepted.Length);BaselineResets++;
                if(!Feasible(accepted))throw new InvalidOperationException("Animated material anchors are outside the requested material limits");
            }
            if(EnforceContinuousSelfContact){if(SelfContact==null)SelfContact=new TriangleMotionGuard(accepted,faces);SelfContact.BeginStep();}
        }
        bool Feasible(Vec3[] points)
        {
            for(int e=0;e<edges.Length;e++)if((points[edges[e].A]-points[edges[e].B]).Length>hardMaximumEdge[e]+1e-12)return false;
            // The projection margin is not an additional hard material limit.
            // A transported valid state can lose a few ulps of margin without
            // becoming infeasible. Keep it; closing rays below the margin stay
            // blocked until an area-increasing correction restores the margin.
            for(int t=0;t<minimumArea.Length;t++){int k=t*3;if(Vec3.Cross(points[faces[k+1]]-points[faces[k]],points[faces[k+2]]-points[faces[k]]).Length<hardMinimumArea[t]*(1-1e-8))return false;}
            return true;
        }
        public void Project(Vec3[] targets,int maximumSweeps=4)
        {
            if(anchors==null||targets.Length!=accepted.Length||maximumSweeps<1)throw new ArgumentException("BeginStep and valid targets are required");
            LastSweeps=0;
            for(int sweep=0;sweep<maximumSweeps;sweep++)
            {
                bool moved=false;LastSweeps++;
                for(int k=0;k<accepted.Length;k++)
                {
                    int i=(sweep&1)==0?k:accepted.Length-1-k;if(inverseMass[i]<=0)continue;
                    Vec3 delta=targets[i]-accepted[i];double length=delta.Length;if(length<1e-12)continue;
                    Vec3 direction=delta/length;double travel=Math.Min(length,BidirectionalContactSolver.RayLimit(accepted[i]-anchors[i],direction,radius));
                    for(int j=edgeOffsets[i];j<edgeOffsets[i+1];j++){int e=vertexEdges[j],other=edges[e].A==i?edges[e].B:edges[e].A;travel=Math.Min(travel,BidirectionalContactSolver.RayLimit(accepted[i]-accepted[other],direction,maximumEdge[e]));}
                    for(int j=faceOffsets[i];j<faceOffsets[i+1];j++)
                    {
                        int t=vertexFaces[j],a=faces[t*3],b=faces[t*3+1],c=faces[t*3+2];Vec3 ab=accepted[b]-accepted[a],ac=accepted[c]-accepted[a],normal=Vec3.Cross(ab,ac);double area=normal.Length;
                        if(area<1e-24)continue;normal/=area;
                        Vec3 gradient=i==a?Vec3.Cross(normal,ac-ab):i==b?Vec3.Cross(ac,normal):Vec3.Cross(normal,ab);
                        double slope=Vec3.Dot(gradient,direction);
                        if(slope<0)travel=Math.Min(travel,Math.Max(0,(area-minimumArea[t])/-slope));
                    }
                    if(travel<length-1e-9)LimitedVertices++;
                    if(EnforceContinuousSelfContact){travel=SelfContact.Travel(i,direction,travel);if(TransferSelfContactReaction)moved|=SelfContact.ProjectLastContact(targets,inverseMass);}
                    if(travel<=1e-13)continue;
                    accepted[i]+=direction*(travel<length?travel*.999999:travel);moved=true;
                    if(EnforceContinuousSelfContact)SelfContact.Commit(i);
                }
                if(!moved)break;
            }
            Array.Copy(accepted,targets,accepted.Length);MaximumStrain=0;
            for(int e=0;e<edges.Length;e++)if(referenceEdge[e]>1e-15)MaximumStrain=Math.Max(MaximumStrain,(accepted[edges[e].A]-accepted[edges[e].B]).Length/referenceEdge[e]-1);
        }
    }
}
