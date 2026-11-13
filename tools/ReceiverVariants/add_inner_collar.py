"""Complete the existing two-sided channel with an inward-facing entry collar.

Keep every existing position, skin record, UV and triangle. Add independent
normal/tangent identities for the inner lip edge; never weld skin identities.
"""
import argparse,copy,hashlib,json,struct,zipfile,itertools
from collections import OrderedDict
from pathlib import Path
import numpy as np
import UnityPy
from UnityPy.helpers.MeshHelper import MeshHandler

def encode(value):return json.dumps(value,separators=(',',':'),ensure_ascii=False).encode('utf-8')
def signature(vertices,faces):return hashlib.sha256(struct.pack('<ii',len(vertices),faces.size)+vertices.astype('<f4').tobytes()+faces.astype('<i4').tobytes()).hexdigest().upper()

def build(source,destination):
 source,destination=Path(source),Path(destination)
 if source.resolve()==destination.resolve() or destination.exists():raise ValueError('Use a new experimental destination; originals are never overwritten')
 with zipfile.ZipFile(source) as archive:
  descriptor=json.loads(archive.read('bpn/receiver.json'));support=json.loads(archive.read('bpn/reactive/support.json'));closed=json.loads(archive.read('bpn/closed-entrance.json'))
  if 'bpn/entry-junction.json' in archive.namelist():raise ValueError('The package already contains an inner collar')
  env=UnityPy.load(archive.read(descriptor['Bundle']));obj=next(o for o in env.objects if o.path_id==-3792098243900159189);tree=obj.read_typetree();h=MeshHandler(obj.read());h.process()
  vertices=np.array(h.m_Vertices);parts=[np.array(f,dtype=np.int32).reshape(-1,3) for f in h.get_triangles()];count=len(vertices)
  if count!=68555 or len(closed['Boundary'])!=20:raise ValueError('This patch requires the verified native 60-face SAC package')
  outer=parts[1][-108:].copy();sources=np.unique(outer[outer>=closed['SourceVertexStart']+closed['SourceVertexCount']]);
  if len(sources)!=20:raise ValueError('The outer collar vertex identities differ')
  # Both wall layers have a coincident material rim but independent skin/UV
  # identities. Existing inner endpoints retain their own complete skin record;
  # only the added normal copies inherit the exact source vertex record.
  data=tree['m_VariableBoneCountWeights']['m_Data'];records=[data[data[i]:data[i+1]] for i in range(count)]
  inner_vertices=np.unique(parts[2]);boundary_sources=set(map(int,sources));choices={}
  uv=np.array(h.m_UV0);physical=json.loads(archive.read('bpn/reactive/physical.json'));physical_rest=np.array([[q[k] for k in 'XYZ'] for q in physical['Rest']])
  def rim_uv_options(node):
   ids=inner_vertices[np.linalg.norm(vertices[inner_vertices]-physical_rest[node],axis=1)<1e-8]
   if not len(ids):raise ValueError('No baked UV binding on channel rim')
   return np.unique(np.round(uv[ids],9),axis=0)
  seam_shifts=[];rim_values=[]
  for node in set(closed['RingA']+closed['RingB']):
   values=rim_uv_options(node);rim_values.extend(values)
   for x,y in itertools.combinations(values,2):
    if np.linalg.norm(x-y)>.01:seam_shifts.extend([x-y,y-x])
  seam_shifts=np.unique(np.round(seam_shifts,9),axis=0) if len(seam_shifts) else []
  uv_min=np.min(rim_values,axis=0);uv_max=np.max(rim_values,axis=0)
  for i in np.unique(outer):
   i=int(i)
   if i not in boundary_sources:
    candidates=inner_vertices[np.linalg.norm(vertices[inner_vertices]-vertices[i],axis=1)<1e-9]
    candidates=[int(j) for j in candidates]
    if not candidates:raise ValueError('No independently skinned inner-wall rim identity for '+str(i))
    choices[i]=[(j,uv[j]) for j in candidates]
   else:
    j=i-(closed['SourceVertexStart']+closed['SourceVertexCount']);a=closed['RingA'][j];b=closed['RingB'][j];t=closed['RingT'][j];av=rim_uv_options(a);bv=rim_uv_options(b)
    values=[x*(1-t)+y*t for x,y in itertools.product(av,bv) if np.linalg.norm(x-y)<.01]
    if t<1e-7:values+=list(av)
    if t>1-1e-7:values+=list(bv)
    if not values:raise ValueError('UV seam intersects an unsplit collar boundary')
    # Keep two atlas identities at the lining's actual wrap seam. An endpoint
    # on a neighbouring chart uses its translated edge value, not a triangle
    # interpolated across the full atlas. Padding is limited to the rim band.
    values += [np.clip(q+shift,uv_min,uv_max) for q in list(values) for shift in seam_shifts]
    choices[i]=[(None,q) for q in np.unique(np.round(values,9),axis=0)]
  pool={};source_rows=[];inner_uv=[];inner=[]
  for face in outer:
   options=list(itertools.product(*(choices[int(i)] for i in face)))
   selected=min(options,key=lambda option:np.max(np.ptp(np.array([q[1] for q in option]),axis=0)))
   triangle=[]
   for i,(target,q) in zip(face,selected):
    if target is None:
     key=(int(i),tuple(np.round(q,9)))
     if key not in pool:pool[key]=count+len(source_rows);source_rows.append(int(i));inner_uv.append(q)
     target=pool[key]
    triangle.append(target)
   inner.append(triangle[::-1])
  sources=np.array(source_rows,dtype=np.int32);inner=np.array(inner,dtype=np.int32);parts[2]=np.concatenate([parts[2],inner]);new_vertices=np.concatenate([vertices,vertices[sources]])
  raw=bytes(tree['m_VertexData']['m_DataSize']);cursor=0;new_raw=b''
  for stream,stride in enumerate([40,48,16,4]):
   cursor=(cursor+15)//16*16;block=np.frombuffer(raw[cursor:cursor+count*stride],dtype=np.uint8).reshape(count,stride);cursor+=count*stride
   block=block.copy();appended=block[sources].copy()
   if stream==0:
    floats=appended.view('<f4').reshape(len(sources),10);floats[:,3:6]*=-1;floats[:,9]*=-1
   if stream==1:
    for j,i in enumerate(sources):appended[j,:8]=np.frombuffer(np.array(inner_uv[j],dtype='<f4').tobytes(),dtype=np.uint8)
   new_raw+=b'\0'*((-len(new_raw))%16)+block.tobytes()+appended.tobytes()
  if cursor!=len(raw):raise ValueError('Unexpected packed vertex stream layout')
  tree['m_VertexData']['m_VertexCount']=len(new_vertices);tree['m_VertexData']['m_DataSize']=new_raw
  records += [list(records[int(i)]) for i in sources];offsets=[len(new_vertices)+1];payload=[]
  for rec in records:payload+=rec;offsets.append(len(new_vertices)+1+len(payload))
  tree['m_VariableBoneCountWeights']['m_Data']=offsets+payload
  faces=np.concatenate(parts);tree['m_IndexBuffer']=list(faces.astype('<u4').tobytes());cursor=0
  for submesh,part in zip(tree['m_SubMeshes'],parts):
   submesh.update(firstByte=cursor*4,indexCount=int(part.size),baseVertex=0,firstVertex=int(part.min()) if part.size else 0,vertexCount=int(part.max()-part.min()+1) if part.size else 0);cursor+=part.size
  shapes=tree['m_Shapes'];shape_records=[];old_shapes=copy.deepcopy(shapes['vertices'])
  for shape in shapes['shapes']:
   rows=copy.deepcopy(old_shapes[shape['firstVertex']:shape['firstVertex']+shape['vertexCount']]);lookup={row['index']:row for row in rows}
   for j,i in enumerate(sources):
    if int(i) in lookup:
     row=copy.deepcopy(lookup[int(i)]);row['index']=count+j
     for key in ['normal']:
      if key in row:row[key]={k:-v for k,v in row[key].items()}
     rows.append(row)
   shape['firstVertex']=len(shape_records);shape['vertexCount']=len(rows);shape_records+=rows
  shapes['vertices']=shape_records;obj.save_typetree(tree)
  new_signature=signature(new_vertices,faces);resources={}
  for name in archive.namelist():
   if not name.endswith('.json'):continue
   value=json.loads(archive.read(name))
   if 'VisualMeshSignature' in value:value['VisualMeshSignature']=new_signature
   if name=='bpn/receiver.json':value['VisualVertices']=len(new_vertices)
   if 'Bindings' in value:
    by_vertex={b['Vertex']:b for b in value['Bindings']}
    for j,i in enumerate(sources):
     if int(i) in by_vertex:
      b=copy.deepcopy(by_vertex[int(i)]);b['Vertex']=count+j;value['Bindings'].append(b)
   resources[name]=value
  resources['bpn/entry-junction.json']={'Schema':1,'OuterSubmesh':1,'InnerSubmesh':2,'OuterFaceStart':len(parts[1])-108,'InnerFaceStart':len(parts[2])-108,'FaceCount':108,'OriginalVertices':count,'Sources':sources.tolist(),'InnerTriangles':inner.reshape(-1).tolist(),'BakedRimUV':True}
  oldcab=next(k for k in env.file.files if not k.endswith('.resS'));newcab='CAB-'+hashlib.md5(b'BPN.SAC.Innie.Inner.Collar.1.0.9').hexdigest()
  for o in env.objects:
   if o.type.name=='Texture2D':
    t=o.read_typetree();stream=t.get('m_StreamData')
    if stream and stream['path']:stream['path']=stream['path'].replace(oldcab,newcab);o.save_typetree(t)
  env.file.files=OrderedDict((k.replace(oldcab,newcab),f) for k,f in env.file.files.items())
  for k,f in env.file.files.items():f.name=k
  bundle=env.file.save(packer='lz4');check=UnityPy.load(bundle);readback=MeshHandler(next(o for o in check.objects if o.path_id==obj.path_id).read());readback.process()
  if not np.array_equal(np.array(readback.m_Vertices),new_vertices.astype('<f4')):raise ValueError('Vertex positions changed during serialization')
  if signature(np.array(readback.m_Vertices),np.concatenate([np.array(f,dtype=np.int32).reshape(-1,3) for f in readback.get_triangles()]))!=new_signature:raise ValueError('Serialized mesh signature differs')
  final_uv=np.array(readback.m_UV0)
  for face_set in [parts[2][-108:]]:
   spans=np.max(np.ptp(final_uv[face_set],axis=1),axis=1)
   if np.max(spans)>.05:
    i=int(np.argmax(spans));raise ValueError('Collar still crosses unrelated texture islands: '+str(face_set[i].tolist())+' UV='+str(final_uv[face_set[i]].tolist()))
  destination.parent.mkdir(parents=True,exist_ok=True)
  with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as result:
   for name in archive.namelist():
    if name==descriptor['Bundle']:result.writestr(name,bundle)
    elif name in resources:result.writestr(name,encode(resources[name]))
    else:result.writestr(name,archive.read(name))
   result.writestr('bpn/entry-junction.json',encode(resources['bpn/entry-junction.json']))
  return {'NewVertices':len(sources),'InnerCollarTriangles':len(inner),'VisualVertices':len(new_vertices),'MeshSignature':new_signature,'OriginalGeometryUnchanged':True,'NewBones':0}

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('source');parser.add_argument('destination');args=parser.parse_args();print(json.dumps(build(args.source,args.destination),indent=2))
