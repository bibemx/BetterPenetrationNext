"""Keep SAC Innie's native exterior; transplant only the internal receiver.

The original exterior vertices, UVs, normals and tangents are copied exactly.
Only a narrow hidden inner closure is removed. A connected mucosal collar is
added behind the apposed lips. Bone identities are not welded.
"""
from pathlib import Path
from collections import OrderedDict,defaultdict
import copy,hashlib,json,struct,zipfile,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.sparse.linalg import splu
import UnityPy
from UnityPy.helpers.MeshHelper import MeshHandler

p=Path(__file__).parent;out=p/'native-build';out.mkdir(exist_ok=True)
entry='abdata/chara/next/oo_base_bpn_1.unity3d';newentry='abdata/chara/bpn/sac_innie.unity3d';guid='org.betterpenetration.next.body.sac-innie.v1'
def encode(x):return json.dumps(x,separators=(',',':'),ensure_ascii=False).encode('utf-8')
def vector(x):return np.array([[q[k] for k in 'XYZ'] for q in x])
def vectors(x):return [dict(zip('XYZ',map(float,q))) for q in x]
def signature(v,f):return hashlib.sha256(struct.pack('<ii',len(v),f.size)+v.astype('<f4').tobytes()+f.astype('<i4').tobytes()).hexdigest().upper()
def aabb(v):
 lo=v.min(0);hi=v.max(0);return {'m_Center':dict(zip('xyz',map(float,(lo+hi)*.5))),'m_Extent':dict(zip('xyz',map(float,(hi-lo)*.5)))}
def normal_uv(h,k,count,dimension):
 a=np.array(getattr(h,k,None));return a if a.shape==(count,dimension) else np.zeros((count,dimension))

prof=p/'originals/BPN.Reciever.Profundis.zipmod';sac=p/'originals/[KKS][Body][BP] SAC Innie.zipmod'
with zipfile.ZipFile(prof) as z,zipfile.ZipFile(sac) as source:
 env=UnityPy.load(z.read(entry));senv=UnityPy.load(source.read('abdata/chara/oo_base_bp_sac_innie.unity3d'))
 ro=next(o for o in env.objects if o.path_id==-6359003101573272306);sr=next(o.read() for o in senv.objects if o.path_id==ro.path_id);mo=ro.read().m_Mesh.deref();h=MeshHandler(mo.read());h.process();sh=MeshHandler(sr.m_Mesh.deref().read());sh.process()
 tree=mo.read_typetree();original_tree=copy.deepcopy(tree);native=np.array(sh.m_Vertices);old=np.array(h.m_Vertices);N=len(old);S=len(native)
 data={name:json.loads(z.read(name)) for name in z.namelist() if name.endswith('.json')}
 support=data['bpn/reactive/support.json'];physical=data['bpn/reactive/physical.json'];guide=data['bpn/reactive/contours.json'];chart=data['bpn/reactive/chart.json']
 # The SAC surface keeps its own skin. The transplanted internal wall is
 # decoupled from the incompatible legacy genital bones and uses the existing
 # MED carrier for that part of its weights. Its MED weights remain unchanged.
 donor_bones=ro.read().m_Bones;source_bones=sr.m_Bones;names=list(support['BoneNames']);source_names=[b.deref().read().m_GameObject.deref().read().m_Name for b in source_bones]
 original_var=original_tree['m_VariableBoneCountWeights']['m_Data'];inner_vertices=np.unique(np.concatenate([np.array(x,dtype=np.int32).reshape(-1) for x in h.get_triangles()[1:]]));inner_set=set(map(int,inner_vertices));inner_controls={b['A'] for b in support['Bindings'] if b['Vertex'] in inner_set};reweighted_vertices=inner_set|{b['Vertex'] for b in support['Bindings'] if b['A'] in inner_controls};pelvis=names.index('MED_Pelvis');old_records=[original_var[original_var[i]:original_var[i+1]] for i in range(N)];reweighted=0
 for i in reweighted_vertices:
  rec=old_records[i];med=defaultdict(int);changed=False
  for word in rec:
   bone=word&65535;weight=word>>16
   if not names[bone].startswith('MED_'):bone=pelvis;changed=True
   med[bone]+=weight
  if changed:old_records[i]=[int(weight)<<16|int(bone) for bone,weight in med.items() if weight];reweighted+=1
 offsets=[N+1];payload=[]
 for record in old_records:payload+=record;offsets.append(N+1+len(payload))
 tree['m_VariableBoneCountWeights']['m_Data']=offsets+payload;original_var=tree['m_VariableBoneCountWeights']['m_Data']
 for definition in [support,physical]:
  by_control={b['A']:b['Vertex'] for b in definition['Bindings'] if b['A']==b['B']==b['C'] and b['U']==1};bone_indices=[];bone_weights=[];influence_offsets=[0]
  for i in range(len(definition['Rest'])):
   if i not in by_control:raise ValueError('Internal control has no native representative')
   rec=old_records[by_control[i]];bone_indices += [x&65535 for x in rec];bone_weights += [(x>>16)/65535 for x in rec];influence_offsets.append(len(bone_indices))
  definition['BoneIndices']=bone_indices;definition['BoneWeights']=bone_weights;definition['InfluenceOffsets']=influence_offsets
 protected_bones=set(physical['BoneIndices'])
 for i in inner_vertices:protected_bones.update(original_var[j]&65535 for j in range(original_var[i],original_var[i+1]))
 source_tree=sr.m_Mesh.deref().read_typetree();renderer_tree=ro.read_typetree();remap=[];aliases=[]
 for source_i,name in enumerate(source_names):
  i=names.index(name)
  if i in protected_bones:
   remap.append(len(names));aliases.append(name);names.append(name);tree['m_BindPose'].append(copy.deepcopy(source_tree['m_BindPose'][source_i]));tree['m_BonesAABB'].append(copy.deepcopy(source_tree['m_BonesAABB'][source_i]));renderer_tree['m_Bones'].append(copy.deepcopy(renderer_tree['m_Bones'][i]))
  else:remap.append(i);tree['m_BindPose'][i]=copy.deepcopy(source_tree['m_BindPose'][source_i])
 remap=np.array(remap,dtype=np.int32);ro.save_typetree(renderer_tree)
 bindposes=[float(x) for matrix in tree['m_BindPose'] for x in matrix.values()]
 for d in data.values():
  if isinstance(d,dict) and 'BoneNames' in d:d['BoneNames']=names.copy()
  if isinstance(d,dict) and 'Bindposes' in d:d['Bindposes']=bindposes.copy()
 # Copy the actual SAC lip-bone hierarchy/rest pose. Preserve the donor's six
 # inner attachment world poses when their SAC parent orientation changes.
 transforms={b.deref().read().m_GameObject.deref().read().m_Name:b.deref() for b in donor_bones};source_transforms={b.deref().read().m_GameObject.deref().read().m_Name:b.deref() for b in source_bones}
 def include_ancestors(mapping):
  for obj in list(mapping.values()):
   while obj.read().m_Father.path_id:
    obj=obj.read().m_Father.deref();name=obj.read().m_GameObject.deref().read().m_Name
    if name in mapping and mapping[name].path_id!=obj.path_id:raise ValueError('Ambiguous native bone ancestor '+name)
    mapping[name]=obj
 include_ancestors(transforms);include_ancestors(source_transforms)
 def world(obj):
  t=obj.read_typetree();m=np.eye(4);m[:3,:3]=Rotation.from_quat([t['m_LocalRotation'][k] for k in 'xyzw']).as_matrix()@np.diag([t['m_LocalScale'][k] for k in 'xyz']);m[:3,3]=[t['m_LocalPosition'][k] for k in 'xyz']
  parent=t['m_Father']['m_PathID'];return world(next(o for o in env.objects if o.path_id==parent))@m if parent else m
 preserved={name:world(obj) for name,obj in transforms.items() if name.startswith('cf_J_Vagina_Inner')}
 for name,src in source_transforms.items():
  if not name.startswith('cf_J_Vagina_'):continue
  target=transforms[name];t=target.read_typetree();s=src.read_typetree();source_parent=src.read().m_Father.deref().read().m_GameObject.deref().read().m_Name
  if source_parent not in transforms:raise ValueError('Unknown SAC bone parent '+source_parent)
  new_parent=transforms[source_parent];old_parent_id=t['m_Father']['m_PathID']
  if old_parent_id!=new_parent.path_id:
   old_parent=next(o for o in env.objects if o.path_id==old_parent_id);ot=old_parent.read_typetree();ot['m_Children']=[x for x in ot['m_Children'] if x['m_PathID']!=target.path_id];old_parent.save_typetree(ot)
   nt=new_parent.read_typetree();nt['m_Children'].append({'m_FileID':0,'m_PathID':target.path_id});new_parent.save_typetree(nt);t['m_Father']={'m_FileID':0,'m_PathID':new_parent.path_id}
  for key in ['m_LocalPosition','m_LocalRotation','m_LocalScale']:t[key]=copy.deepcopy(s[key])
  target.save_typetree(t)
 for name,saved_world in preserved.items():
  obj=transforms[name];t=obj.read_typetree();parent=next(o for o in env.objects if o.path_id==t['m_Father']['m_PathID']);m=np.linalg.inv(world(parent))@saved_world;scale=np.linalg.norm(m[:3,:3],axis=0);q=Rotation.from_matrix(m[:3,:3]/scale[None,:]).as_quat();t['m_LocalPosition']=dict(zip('xyz',map(float,m[:3,3])));t['m_LocalRotation']=dict(zip('xyzw',map(float,q)));t['m_LocalScale']=dict(zip('xyz',map(float,scale)));obj.save_typetree(t)
 patch=support['UvPatch'];dup=np.array(patch['Duplicates'],dtype=np.int32);D=N+len(dup);sac_start=D
 faces0=np.array(sh.get_triangles()[0],dtype=np.int32).reshape(-1,3);tri=native[faces0];centres=tri.mean(1)
 remove=(np.max(abs(tri[:,:,0]),axis=1)<.0005)&(centres[:,2]>-.034)&(centres[:,2]<-.010)&(centres[:,1]>-.216)
 if np.sum(remove)!=60:raise ValueError('Unexpected SAC inner closure; re-audit geometry')
 kept=faces0[~remove]
 # Analyze coincident seam positions; never collapse their native skin records.
 _,canonical=np.unique(np.round(native,7),axis=0,return_inverse=True);removed_edges=defaultdict(list)
 for face in faces0[remove]:
  for a,b in zip(face,np.roll(face,-1)):removed_edges[tuple(sorted((canonical[a],canonical[b])))].append((int(a),int(b)))
 boundary_keys={k for k,e in removed_edges.items() if len(e)==1 and k[0]!=k[1]};surviving={}
 for face in kept:
  for a,b in zip(face,np.roll(face,-1)):
   key=tuple(sorted((canonical[a],canonical[b])))
   if key in boundary_keys:surviving[key]=(int(a),int(b))
 if set(surviving)!=boundary_keys:raise ValueError('SAC incision does not have a matching exterior edge')
 adjacency=defaultdict(list);representative={}
 for (a,b),(va,vb) in surviving.items():adjacency[a].append(b);adjacency[b].append(a);representative[canonical[va]]=va;representative[canonical[vb]]=vb
 if any(len(x)!=2 for x in adjacency.values()):raise ValueError('Incision is not a simple closed loop')
 start=min(adjacency);order=[start];previous=None;current=start
 while True:
  choices=[x for x in adjacency[current] if x!=previous];next_node=choices[0]
  if next_node==start:break
  if next_node in order:raise ValueError('Incision loop repeats')
  order.append(next_node);previous,current=current,next_node
 if len(order)!=len(adjacency):raise ValueError('Disconnected incision loops')
 cut=np.array([representative[x] for x in order],dtype=np.int32)
 rest=vector(physical['Rest']);ring=np.array(guide['Boundaries'][0],dtype=np.int32);r0=rest[ring].mean(0);r1=(rest[guide['Contours'][1]['Vertices']]*np.array(guide['Contours'][1]['Weights'])[:,None]).sum(0);axis=r1-r0;axis/=np.linalg.norm(axis);side=np.array([1.,0,0]);up=np.cross(axis,side);cut_centre=native[cut].mean(0)
 def orient(ids,vertices,centre):
  q=vertices[ids]-centre;area=np.sum((q@side)*np.roll(q@up,-1)-np.roll(q@side,-1)*(q@up))
  if area<0:ids=ids[::-1]
  q=vertices[ids]-centre;theta=np.arctan2(q@up,q@side);first=np.argmin(abs(theta));return np.roll(ids,-first)
 cut=orient(cut,native,cut_centre);ring=orient(ring,rest,r0)
 def parameters(points):
  distances=np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1);return np.r_[0,np.cumsum(distances[:-1])]/sum(distances)
 # Four anatomical quadrants keep the slit tips on the median plane. A single
 # circumference parameter twists a thin slit when mapped to an open ellipse.
 q=native[cut]-cut_centre;u=q@up;median=np.flatnonzero(abs(native[cut,0])<1e-7);top=int(median[np.argmax(u[median])]);bottom=int(median[np.argmin(u[median])]);left=int(np.argmin(np.where(native[cut,0]<-1e-7,abs(u),np.inf)))
 marks=[0,top,left,bottom,len(cut)]
 if marks!=sorted(marks) or len(set(marks))!=5:raise ValueError('SAC slit quadrant order differs')
 cp=np.zeros(len(cut))
 for quadrant,(a,b) in enumerate(zip(marks,marks[1:])):
  ids=np.arange(a,b+1)%len(cut);lengths=np.linalg.norm(np.diff(native[cut[ids]],axis=0),axis=1);cum=np.r_[0,np.cumsum(lengths)]/sum(lengths);cp[a:b]=(quadrant+cum[:-1])/4
 q=rest[ring]-r0;theta=np.unwrap(np.arctan2((q@up)/max(abs(q@up)),(q@side)/max(abs(q@side))));rp=(theta-theta[0])/(2*np.pi)
 if np.any(np.diff(rp)<=0):raise ValueError('Donor entrance angular order differs')
 ring_control_to_visual={b['A']:b['Vertex'] for b in physical['Bindings'] if b['A']==b['B']==b['C'] and b['U']==1}
 ring_vertices=np.array([ring_control_to_visual[int(x)] for x in ring],dtype=np.int32)
 ring_a=[];ring_b=[];ring_t=[]
 for t in cp:
  a=max(0,np.searchsorted(rp,t,side='right')-1);b=(a+1)%len(ring);end=rp[b] if b else 1.;ring_a.append(int(ring[a]));ring_b.append(int(ring[b]));ring_t.append(float((t-rp[a])/(end-rp[a])))
 # Build a narrow connecting annulus; the two boundary loops remain distinct.
 bridge=[];a=b=0
 while a<len(cut) or b<len(ring):
  ta=cp[a+1] if a+1<len(cut) else 1.;tb=rp[b+1] if b+1<len(ring) else 1.
  va=sac_start+int(cut[a%len(cut)]);vb=int(ring_vertices[b%len(ring)])
  if a<len(cut) and (b>=len(ring) or ta<=tb):bridge.append([va,sac_start+int(cut[(a+1)%len(cut)]),vb]);a+=1
  else:bridge.append([va,int(ring_vertices[(b+1)%len(ring)]),vb]);b+=1
 bridge=np.array(bridge,dtype=np.int32)
 # Bake the donor's existing internal UV seam split before adding native SAC.
 donor_tri=np.array(patch['Triangles'],dtype=np.int32);old_sub=[np.array(f,dtype=np.int32).reshape(-1,3) for f in h.get_triangles()];parts=[];cursor=0
 for f in old_sub:parts.append(donor_tri[cursor:cursor+f.size].reshape(-1,3));cursor+=f.size
 parts[0]=kept+sac_start;parts[1]=np.concatenate([parts[1],bridge])
 v=np.concatenate([old,old[dup],native]);normals=np.concatenate([np.array(h.m_Normals),np.array(h.m_Normals)[dup],np.array(sh.m_Normals)]);tangents=np.concatenate([np.array(h.m_Tangents),np.array(h.m_Tangents)[dup],np.array(sh.m_Tangents)])
 uv=[]
 for k in range(6):
  first=normal_uv(h,'m_UV'+str(k),N,2);first=np.concatenate([first,first[dup]]);
  if k==0:first=np.array(patch['Coordinates']).reshape(-1,2)
  uv.append(np.concatenate([first,normal_uv(sh,'m_UV'+str(k),S,2)]))
 # Skin-space UV copies on the connecting collar do not alter SAC's native UVs.
 collar_dups=[];collar_map={};collar_uv=[]
 for j,vertex in enumerate(cut):
  source_vertex=sac_start+int(vertex);collar_map[source_vertex]=len(v)+len(collar_dups);collar_dups.append(source_vertex)
  a=ring_a[j];b=ring_b[j];t=ring_t[j];collar_uv.append(uv[0][ring_control_to_visual[a]]*(1-t)+uv[0][ring_control_to_visual[b]]*t)
 for j in range(len(bridge)):
  for k in range(3):
   if int(bridge[j,k]) in collar_map:parts[1][-len(bridge)+j,k]=collar_map[int(bridge[j,k])]
 collar_dups=np.array(collar_dups,dtype=np.int32);base_count=len(v);v=np.concatenate([v,v[collar_dups]]);normals=np.concatenate([normals,normals[collar_dups]]);tangents=np.concatenate([tangents,tangents[collar_dups]])
 for k in range(6):uv[k]=np.concatenate([uv[k],np.array(collar_uv) if k==0 else uv[k][collar_dups]])
 source_weights=np.array(sh.m_BoneWeights);source_indices=remap[np.array(sh.m_BoneIndices,dtype=np.int32)]
 old_legacy_weights=np.array(h.m_BoneWeights,dtype=np.uint16);old_legacy_indices=np.array(h.m_BoneIndices,dtype=np.uint16)
 for i in reweighted_vertices:
  old_legacy_weights[i]=0;old_legacy_indices[i]=0
  for j,word in enumerate(old_records[i][:4]):old_legacy_weights[i,j]=word>>16;old_legacy_indices[i,j]=word&65535
 legacy_weights=np.concatenate([old_legacy_weights,old_legacy_weights[dup],np.round(source_weights*65535)]).astype(np.uint16);legacy_indices=np.concatenate([old_legacy_indices,old_legacy_indices[dup],source_indices]).astype(np.uint16)
 legacy_weights=np.concatenate([legacy_weights,legacy_weights[collar_dups]]);legacy_indices=np.concatenate([legacy_indices,legacy_indices[collar_dups]])
 old_var=tree['m_VariableBoneCountWeights']['m_Data'];records=[old_var[old_var[i]:old_var[i+1]] for i in range(N)];records += [list(records[int(i)]) for i in dup]
 packed_source=[]
 for weights,indices in zip(source_weights,source_indices):
  active=np.flatnonzero(weights>0);scaled=weights[active]/sum(weights[active])*65535;q=np.floor(scaled).astype(int);q[np.argsort(-(scaled-q))[:65535-sum(q)]]+=1
  packed_source.append([int(w)<<16|int(indices[i]) for i,w in zip(active,q) if w>0])
 records+=packed_source;records += [list(records[int(i)]) for i in collar_dups]
 offsets=[len(v)+1];payload=[]
 for record in records:payload+=record;offsets.append(len(v)+1+len(payload))
 tree['m_VariableBoneCountWeights']['m_Data']=offsets+payload
 # Explicit 40/48/16/4-byte streams preserve native SAC's six UV channels/color.
 count=len(v);streams=[np.concatenate([v,normals,tangents],axis=1).astype('<f4').tobytes(),np.concatenate(uv,axis=1).astype('<f4').tobytes(),b''.join(struct.pack('<4H4H',*weights,*indices) for weights,indices in zip(legacy_weights,legacy_indices))]
 colors=np.full((count,4),255,dtype=np.uint8);sc=np.array(sh.m_Colors)
 if sc.shape==(S,4):colors[sac_start:sac_start+S]=np.round(sc*255 if np.max(sc)<=1 else sc).astype(np.uint8)
 colors[base_count:]=colors[collar_dups];streams.append(colors.tobytes());raw=b''
 for stream in streams:raw+=b'\0'*((-len(raw))%16)+stream
 channels=copy.deepcopy(tree['m_VertexData']['m_Channels']);channels[3]={'stream':3,'offset':0,'format':2,'dimension':4}
 for k in range(6):channels[4+k]={'stream':1,'offset':k*8,'format':0,'dimension':2}
 tree['m_VertexData']={'m_VertexCount':count,'m_Channels':channels,'m_DataSize':raw}
 all_faces=np.concatenate(parts);tree['m_IndexFormat']=1;tree['m_IndexBuffer']=list(all_faces.astype('<u4').tobytes());cursor=0
 for item,f in zip(tree['m_SubMeshes'],parts):
  item.update(firstByte=cursor*4,indexCount=int(f.size),baseVertex=0,firstVertex=int(f.min()) if f.size else 0,vertexCount=int(f.max()-f.min()+1) if f.size else 0)
  if f.size:item['localAABB']=aabb(v[np.unique(f)])
  cursor+=f.size
 tree['m_LocalAABB']=aabb(v)
 old_shapes=original_tree['m_Shapes'];new_records=[];new_shapes=copy.deepcopy(old_shapes)
 for shape in new_shapes['shapes']:
  begin=shape['firstVertex'];length=shape['vertexCount'];items=copy.deepcopy(old_shapes['vertices'][begin:begin+length]);lookup={x['index']:x for x in items}
  for j,source_vertex in enumerate(dup):
   if int(source_vertex) in lookup:item=copy.deepcopy(lookup[int(source_vertex)]);item['index']=N+j;items.append(item)
  shape['firstVertex']=len(new_records);shape['vertexCount']=len(items);new_records+=items
 new_shapes['vertices']=new_records;tree['m_Shapes']=new_shapes;mo.save_typetree(tree)
 # Geodesic transfer follows each lip's material surface. Euclidean proximity
 # would couple apposed opposing folds and move the wrong side of the slit.
 vertices=native;cut_points=native[cut];dist=np.full(S,np.inf);anchor_a=np.zeros(S,dtype=int);anchor_b=np.zeros(S,dtype=int);anchor_t=np.zeros(S)
 for j,a in enumerate(cut):
  b=cut[(j+1)%len(cut)];edge=vertices[b]-vertices[a];t=np.clip((vertices-vertices[a])@edge/max(edge@edge,1e-30),0,1);d=np.linalg.norm(vertices-vertices[a]-t[:,None]*edge,axis=1);mask=d<dist;dist[mask]=d[mask];anchor_a[mask]=j;anchor_b[mask]=(j+1)%len(cut);anchor_t[mask]=t[mask]
 graph_edges={}
 for face in kept:
  for a,b in zip(face,np.roll(face,-1)):
   key=tuple(sorted((int(a),int(b))));graph_edges[key]=max(1e-10,float(np.linalg.norm(native[a]-native[b])))
 skin_groups=defaultdict(list)
 for i in range(S):skin_groups[(tuple(np.round(native[i],7)),tuple(source_indices[i]),tuple(np.round(source_weights[i],7)))].append(i)
 for group in skin_groups.values():
  for i in group[1:]:graph_edges[(group[0],i)]=1e-10
 for j,i in enumerate(cut):
  for equivalent in np.flatnonzero(canonical==canonical[i]):graph_edges[(S+j,int(equivalent))]=1e-10
 rows=[];cols=[];values=[]
 for (a,b),value in graph_edges.items():rows += [a,b];cols += [b,a];values += [value,value]
 graph=coo_matrix((values,(rows,cols)),shape=(S+len(cut),S+len(cut))).tocsr();geodesic=dijkstra(graph,directed=False,indices=S+np.arange(len(cut)))[:,:S];dist=np.min(geodesic,axis=0)
 selected=np.unique(kept[np.any((dist<.075)[kept],axis=1)])
 control_start=len(support['Rest']);control_map={int(i):control_start+j for j,i in enumerate(selected)}
 if control_start+len(selected)>10000:raise ValueError('SAC reactive surface exceeds control limit')
 for name,d in data.items():
  if 'VisualMeshSignature' in d:d['VisualMeshSignature']=signature(v,all_faces)
  if 'UvPatch' in d:d['UvPatch']=None
  if 'Bindings' in d:
   by_vertex={b['Vertex']:b for b in d['Bindings']}
   for j,i in enumerate(dup):
    if int(i) in by_vertex:b=copy.deepcopy(by_vertex[int(i)]);b['Vertex']=N+j;d['Bindings'].append(b)
 # Native SAC surface controls are a separate contact-owned group. The shared
 # channel solver keeps them pinned; the closed-entrance constraint owns them.
 for source_vertex in selected:
  node=control_map[int(source_vertex)];support['Rest'].append(dict(zip('XYZ',map(float,native[source_vertex]))));support['PinWeights'].append(1)
  support['GuideCoordinates'].append(0);support['GuideWeights'].append(0);chart['Regions'].append(10);chart['SupportAxial'].append(-1);chart['SupportAngle'].append(0)
  rec=packed_source[int(source_vertex)]
  support['BoneIndices'] += [x&65535 for x in rec];support['BoneWeights'] += [(x>>16)/65535 for x in rec];support['InfluenceOffsets'].append(len(support['BoneIndices']))
  for morph in support['Morphs']:morph['Deltas'].append({'X':0.,'Y':0.,'Z':0.})
  support['Bindings'].append({'Vertex':sac_start+int(source_vertex),'A':node,'B':node,'C':node,'U':1.,'V':0.,'W':0.,'Influence':1.})
 for j,source_vertex in enumerate(collar_dups):
  native_index=int(source_vertex)-sac_start;node=control_map[native_index];support['Bindings'].append({'Vertex':base_count+j,'A':node,'B':node,'C':node,'U':1.,'V':0.,'W':0.,'Influence':1.})
 for name in ['bpn/reactive/transfer.json','bpn/reactive/reactive-transfer.json']:
  d=data[name];d['Offsets'] += [d['Offsets'][-1]]*len(selected)
 local_map={int(i):j for j,i in enumerate(selected)};closed_faces=np.array([[local_map[int(i)] for i in f] for f in kept if all(int(i) in local_map for i in f)],dtype=np.int32)
 blend_offsets=[0];blend_indices=[];blend_weights=[]
 # Harmonic extension preserves the original differential shape of the lips.
 # Coincident, identically skinned copies share analysis coefficients only;
 # all native vertex/weight identities are retained in the exported mesh.
 group_of=np.zeros(S,dtype=np.int32);groups=list(skin_groups.values())
 for j,group in enumerate(groups):group_of[group]=j
 conductance=defaultdict(float)
 for (a,b),distance in graph_edges.items():
  if a>=S or b>=S:continue
  a=int(group_of[a]);b=int(group_of[b])
  if a==b:continue
  conductance[tuple(sorted((a,b)))]+=1/max(.0001,distance)
 diagonal=np.zeros(len(groups));rows=[];cols=[];vals=[]
 for (a,b),w in conductance.items():diagonal[a]+=w;diagonal[b]+=w;rows += [a,b];cols += [b,a];vals += [-w,-w]
 rows+=list(range(len(groups)));cols+=list(range(len(groups)));vals+=diagonal.tolist();laplacian=coo_matrix((vals,(rows,cols)),shape=(len(groups),len(groups))).tocsr()
 fixed={}
 for j,i in enumerate(cut):
  for eq in np.flatnonzero(canonical==canonical[i]):fixed[int(group_of[eq])]=j
 group_distance=np.array([np.min(dist[group]) for group in groups]);unknown=np.array([i for i in range(len(groups)) if group_distance[i]<.075 and i not in fixed],dtype=np.int32);known=np.array(sorted(fixed),dtype=np.int32)
 boundary_values=np.zeros((len(known),len(cut)))
 for row,i in enumerate(known):boundary_values[row,fixed[int(i)]]=1
 basis=np.zeros((len(groups),len(cut)));basis[known]=boundary_values
 if len(unknown):basis[unknown]=splu(laplacian[unknown][:,unknown].tocsc()).solve(-laplacian[unknown][:,known]@boundary_values)
 basis=np.maximum(0,basis);influences=[]
 for i in selected:
  values=basis[group_of[i]];influence=float(sum(values));influences.append(min(1,influence));nearest=np.flatnonzero(values>1e-10)
  if len(nearest):weights=values[nearest]/sum(values[nearest])
  else:nearest=np.array([0]);weights=np.ones(1)
  blend_indices+=nearest.tolist();blend_weights+=weights.tolist();blend_offsets.append(len(blend_indices))
 closed={'Schema':1,'Controls':[control_map[int(i)] for i in selected],'Boundary':[control_map[int(i)] for i in cut],'AnchorA':anchor_a[selected].tolist(),'AnchorB':anchor_b[selected].tolist(),'AnchorT':anchor_t[selected].tolist(),'Distance':dist[selected].tolist(),'Influence':influences,'BlendOffsets':blend_offsets,'BlendIndices':blend_indices,'BlendWeights':blend_weights,'Triangles':closed_faces.reshape(-1).tolist(),'RingA':ring_a,'RingB':ring_b,'RingT':ring_t,'SourceVertexStart':sac_start,'SourceVertexCount':S,'OriginalFaceCount':len(faces0),'RemovedInnerFaces':np.flatnonzero(remove).tolist(),'NativeRestSHA256':hashlib.sha256(native.astype('<f4').tobytes()).hexdigest()}
 data['bpn/closed-entrance.json']=closed
 # Stream references and color masks belong to SAC, not Profundis.
 sac_textures={o.read().m_Name:o.read() for o in senv.objects if o.type.name=='Texture2D'}
 for o in env.objects:
  if o.type.name=='Texture2D':
   tex=o.read()
   if tex.m_Name in ['cf_body_00_mc','cf_body_00_mc_low']:tex.set_image(sac_textures[tex.m_Name].image,target_format=4,mipmap_count=tex.m_MipCount);tex.save()
 oldasset='p_cf_body_bpn';newasset='p_cf_body_bpn_sac_innie'
 for o in env.objects:
  if o.type.name=='AssetBundle':
   ab=o.read_typetree();ab['m_Name']=newentry.removeprefix('abdata/');ab['m_Container']=[(key.replace(oldasset,newasset) if oldasset in key else key,value) for key,value in ab['m_Container']];o.save_typetree(ab)
  elif o.type.name=='GameObject':
   go=o.read_typetree()
   if go['m_Name']==oldasset:go['m_Name']=newasset;o.save_typetree(go)
 oldcab=next(k for k in env.file.files if not k.endswith('.resS'));newcab='CAB-'+hashlib.md5(b'BPN.SAC.Innie.Native.Closed.1.0.9').hexdigest()
 for o in env.objects:
  if o.type.name=='Texture2D':
   t=o.read_typetree();stream=t.get('m_StreamData')
   if stream and stream['path']:stream['path']=stream['path'].replace(oldcab,newcab);o.save_typetree(t)
 env.file.files=OrderedDict((k.replace(oldcab,newcab),f) for k,f in env.file.files.items())
 for k,f in env.file.files.items():f.name=k
 bundle=env.file.save(packer='lz4');check=UnityPy.load(bundle);readback=MeshHandler(next(o for o in check.objects if o.path_id==mo.path_id).read());readback.process()
 if not np.array_equal(np.array(readback.m_Vertices)[sac_start:sac_start+S],native):raise ValueError('Native SAC surface changed')
 if not np.array_equal(np.array(readback.m_Vertices),v.astype('<f4')):raise ValueError('Serialized positions differ')
 if signature(np.array(readback.m_Vertices),np.concatenate([np.array(x,dtype=np.int32).reshape(-1,3) for x in readback.get_triangles()]))!=signature(v,all_faces):raise ValueError('Serialized signature differs')
 manifest=ET.fromstring(source.read('manifest.xml'));manifest.find('guid').text=guid;manifest.find('name').text='[BPN] SAC Innie';manifest.find('version').text='1.0.9';body=manifest.find('KK_UncensorSelector/body');body.find('guid').text=guid;body.find('displayName').text='[BPN] SAC Innie';body.find('allowRandom').text='false';body.find('oo_base/file').text=newentry.removeprefix('abdata/');body.find('oo_base/asset').text=newasset
 descriptor={'Schema':1,'Guid':guid,'Name':'[BPN] SAC Innie','Bundle':newentry,'Asset':newasset,'VisualVertices':count,'VisualMeshSignature':signature(v,all_faces),'BakedUv':True,'NativeExterior':True}
 data['bpn/receiver.json']=descriptor
 destination=out/'[KKS][Body][BPN] SAC Innie.zipmod'
 with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as result:
  result.writestr('manifest.xml',ET.tostring(manifest,encoding='utf-8',xml_declaration=True));result.writestr(newentry,bundle)
  for name in z.namelist():
   if name in [entry,'manifest.xml'] or name.endswith('.json'):continue
   result.writestr(name,z.read(name))
  for name,value in data.items():result.writestr(name,encode(value))
 report={'NativeSACVertices':S,'NativeSACPositionsUnchanged':True,'NativeSACUVsPreserved':True,'NativeSACLipBonePoseCopied':True,'SkinPaletteAliases':aliases,'SkinPaletteLength':len(names),'NewBoneTransforms':0,'InternalVerticesDecoupledFromLegacyRig':reweighted,'RemovedHiddenClosureFaces':int(sum(remove)),'BoundaryVertices':len(cut),'MucosalBridgeTriangles':len(bridge),'SurfaceControls':len(selected),'TotalControls':len(support['Rest']),'TotalVertices':count,'MeshSignature':descriptor['VisualMeshSignature'],'OriginalArchivesUnchanged':True,'Installed':False,'UserAccepted':False}
 (out/'build.json').write_bytes(encode(report));print(json.dumps(report,indent=2))
