using System;
using HarmonyLib;
using UnityEngine;
namespace BetterPenetrationNext
{
    public sealed partial class NextRuntime
    {
        public int ExternalMeshCopyReleases { get; private set; }
        void RegisterExternalMeshCompatibility()
        {
            // These exact installed entry points copy sharedMesh before adding
            // an external morph. Copying our current output would turn contact
            // displacement into permanent rest data and break the next bind.
            var controller=Type.GetType("KK_PregnancyPlus.BlendShapeController, KKS_PregnancyPlus",false);
            var helper=Type.GetType("KK_PregnancyPlus.PregnancyPlusHelper, KKS_PregnancyPlus",false);
            var add=controller==null?null:AccessTools.DeclaredMethod(controller,"AddBlendShapeToMesh",new[]{typeof(SkinnedMeshRenderer)});
            var copy=helper==null?null:AccessTools.DeclaredMethod(helper,"CopyMesh",new[]{typeof(Mesh)});
            if(add!=null)hooks.Patch(add,prefix:new HarmonyMethod(typeof(NextRuntime),nameof(BeforePregnancyRendererCopy)){priority=Priority.First});
            if(copy!=null)hooks.Patch(copy,prefix:new HarmonyMethod(typeof(NextRuntime),nameof(BeforePregnancyMeshCopy)){priority=Priority.First});
            if(add!=null||copy!=null)log.LogInfo("NEXT_COMPAT PregnancyPlus mesh-copy baseline hooks: renderer="+(add!=null)+" mesh="+(copy!=null));
        }
        static void BeforePregnancyRendererCopy(SkinnedMeshRenderer __0)
        {
            var runtime=Instance;if(runtime==null||__0==null)return;
            int key=0;bool found=false;
            foreach(var item in runtime.bodies)if(item.Value.Binding.Renderer==__0&&item.Value.Binding.Alive){key=item.Key;found=true;break;}
            if(!found)return;
            // Retire restores the authored mesh and all current morph weights
            // before the external routine reads and clones sharedMesh.
            runtime.Retire(key);runtime.nextDiscovery=0;runtime.ExternalMeshCopyReleases++;
            runtime.log.LogInfo("NEXT_MESH_COPY clean baseline supplied to PregnancyPlus.AddBlendShapeToMesh renderer="+__0.name);
        }
        static void BeforePregnancyMeshCopy(ref Mesh __0)
        {
            var runtime=Instance;if(runtime==null||__0==null)return;
            int key=0;Mesh original=null;bool found=false;
            foreach(var item in runtime.bodies)if(item.Value.Binding.OwnsMesh(__0)){key=item.Key;original=item.Value.Binding.Original;found=true;break;}
            if(!found)return;
            __0=original;runtime.Retire(key);runtime.nextDiscovery=0;runtime.ExternalMeshCopyReleases++;
            runtime.log.LogInfo("NEXT_MESH_COPY clean baseline supplied to PregnancyPlusHelper.CopyMesh");
        }
    }
}
