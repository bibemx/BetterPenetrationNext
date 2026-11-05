using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
using Core_BetterPenetration;
using UnityEngine;

namespace BetterPenetrationNext
{
    // The original exterior soft-tissue bones must see the corrected Source,
    // rather than colliders that still follow its uncorrected animation.
    // Only the selected receiver's audited entrance DynamicBones are touched.
    internal sealed class NativeEntranceContactRig : IDisposable
    {
        readonly List<DynamicBoneCollider> proxies=new List<DynamicBoneCollider>();
        readonly Dictionary<DynamicBone,List<DynamicBoneCollider>> removed=new Dictionary<DynamicBone,List<DynamicBoneCollider>>();
        readonly Dictionary<DynamicBone,bool> suspended=new Dictionary<DynamicBone,bool>();
        CollisionAgent target;GameObject root;bool legacyMode;
        public int BoundBones {get{return removed.Count+suspended.Count;}}
        public int SuspendedBones {get{return suspended.Count;}}
        public bool UsingLegacyDynamics {get{return legacyMode;}}
        public int ProxyCount {get{return proxies.Count;}}
        public void Synchronize(CollisionAgent receiver,DanAgent source,Transform sourceRoot,InextensibleSourceRig rod,Vec3[] vertices,bool legacyDynamics)
        {
            if(receiver==null||source==null||sourceRoot==null){Release();return;}
            if(target!=receiver || legacyMode!=legacyDynamics){Release();target=receiver;legacyMode=legacyDynamics;}
            if(!legacyDynamics)
            {
                // One owner for this prepared entrance: its section field and
                // material collar. The old particle colliders would add a
                // second, delayed opening to the very same weighted vertices.
                // Suspend only the agent's explicit entrance list; preserve
                // other DynamicBones, IK, card sizes and their saved states.
                foreach(var bone in receiver.m_kokanDynamicBones)
                {
                    if(bone==null)continue;
                    if(!suspended.ContainsKey(bone))suspended.Add(bone,bone.enabled);
                    bone.enabled=false;
                }
                return;
            }
            if(root==null)
            {
                root=new GameObject("BPN native entrance contact rig");root.transform.SetParent(sourceRoot,false);
                for(int i=0;i<8;i++){var go=new GameObject("BPN surface section "+i);go.transform.SetParent(root.transform,false);var c=go.AddComponent<DynamicBoneCollider>();c.m_Direction=DynamicBoneCollider.Direction.Y;c.m_Bound=DynamicBoneCollider.Bound.Outside;c.m_Center=Vector3.zero;proxies.Add(c);}
            }
            root.SetActive(true);rod.MeasureRadii(vertices);
            for(int i=0;i<proxies.Count;i++)
            {
                int a=i*(rod.Count-1)/proxies.Count,b=(i+1)*(rod.Count-1)/proxies.Count;Vec3 p=rod.Solved[a],q=rod.Solved[b],edge=q-p;double radius=0;
                for(int s=a;s<=b;s++){Vec3 d=rod.Solved[s]-p,normal=edge.Normalized;radius=Math.Max(radius,rod.SectionRadii[s]+(d-normal*Vec3.Dot(d,normal)).Length);}
                var c=proxies[i];c.transform.position=AnimationAdapter.ToUnity((p+q)*.5);if(edge.LengthSquared>1e-16)c.transform.rotation=Quaternion.FromToRotation(Vector3.up,AnimationAdapter.ToUnity(edge.Normalized));
                var scale=c.transform.lossyScale;double radial=Math.Max(1e-6,(Math.Abs(scale.x)+Math.Abs(scale.z))*.5),axial=Math.Max(1e-6,Math.Abs(scale.y));c.m_Radius=(float)(radius/radial);c.m_Height=(float)(edge.Length/axial+2*c.m_Radius);
            }
            foreach(var bone in receiver.m_kokanDynamicBones)
            {
                if(bone==null)continue;List<DynamicBoneCollider> saved;if(!removed.TryGetValue(bone,out saved)){saved=new List<DynamicBoneCollider>();removed.Add(bone,saved);}
                RemoveNative(bone,source.m_danColliders,saved);RemoveNative(bone,source.m_virtualDanColliders,saved);
                foreach(var c in proxies)if(!bone.m_Colliders.Contains(c))bone.m_Colliders.Add(c);
            }
        }
        static void RemoveNative(DynamicBone bone,List<DynamicBoneCollider> colliders,List<DynamicBoneCollider> saved)
        {if(colliders==null)return;foreach(var c in colliders)if(c!=null&&bone.m_Colliders.Remove(c)&&!saved.Contains(c))saved.Add(c);}
        public void Release()
        {
            foreach(var pair in suspended)if(pair.Key!=null){pair.Key.enabled=pair.Value;if(pair.Value)pair.Key.ResetParticlesPosition();}
            suspended.Clear();
            foreach(var pair in removed)if(pair.Key!=null){foreach(var c in proxies)pair.Key.m_Colliders.Remove(c);foreach(var c in pair.Value)if(c!=null&&!pair.Key.m_Colliders.Contains(c))pair.Key.m_Colliders.Add(c);}
            removed.Clear();target=null;if(root!=null)root.SetActive(false);
        }
        public void Dispose(){Release();if(root!=null)UnityEngine.Object.Destroy(root);root=null;proxies.Clear();}
    }
}
