using System;
using System.Collections;
using BetterPenetrationNext.Core;
using UnityEngine;
using UnityEngine.Rendering;
using Object=UnityEngine.Object;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        IEnumerator RunXray()
        {
            root.transform.localScale=Vector3.one;
            var panelData=PrimitiveMeshes.Grid(8,8,1.2,1.5); for(int i=0;i<panelData.Triangles.Length;i+=3) { int v=panelData.Triangles[i+1]; panelData.Triangles[i+1]=panelData.Triangles[i+2]; panelData.Triangles[i+2]=v; }
            var panel=Skin("BPN neutral pixel panel",panelData,new Vector3(-0.45f,0,0)); panel.gameObject.layer=30;
            var sphere=Skin("BPN neutral pixel sphere",PrimitiveMeshes.Sphere(48,24,0.45),new Vector3(0.1f,0,0.55f)); sphere.gameObject.layer=30;
            var outer=panel.sharedMaterial; outer.color=new Color(0.55f,0.22f,0.09f,1); outer.EnableKeyword("_EMISSION"); outer.SetColor("_EmissionColor",outer.color);
            var original=sphere.sharedMaterial; original.shader=Shader.Find("Unlit/Texture");
            var sourceTexture=new Texture2D(2,1,TextureFormat.RGBA32,false); sourceTexture.SetPixels(new[]{new Color(0.9f,0.2f,0.4f),new Color(0.2f,0.6f,0.9f)}); sourceTexture.Apply(); assets.Add(sourceTexture); original.mainTexture=sourceTexture;
            var uv=new Vector2[sphere.sharedMesh.vertexCount]; var positions=sphere.sharedMesh.vertices; for(int i=0;i<uv.Length;i++) uv[i]=new Vector2(positions[i].x/0.9f+0.5f,positions[i].y/0.9f+0.5f); sphere.sharedMesh.uv=uv;
            var weights=new double[sphere.sharedMesh.vertexCount]; for(int i=0;i<weights.Length;i++) weights[i]=1;
            var geometry=new ContactDetection(sphere,weights); var panelPose=new Mesh(); assets.Add(panelPose); panel.BakeMesh(panelPose);
            var target=new RenderTexture(640,400,24,RenderTextureFormat.ARGB32); target.Create(); assets.Add(target);
            var go=new GameObject("BPN pixel camera"); go.transform.SetParent(root.transform,false); go.transform.localPosition=new Vector3(0,0,-4);
            var camera=go.AddComponent<Camera>(); camera.orthographic=true; camera.orthographicSize=0.9f; camera.nearClipPlane=0.1f; camera.farClipPlane=20; camera.cullingMask=1<<30;
            camera.clearFlags=CameraClearFlags.SolidColor; camera.backgroundColor=new Color(0.07f,0.08f,0.1f,1); camera.allowHDR=false; camera.allowMSAA=false; camera.targetTexture=target;
            XRayRenderer effect=null;
            try
            {
                effect=new XRayRenderer { DotScale=10,DotDensity=0.65f,Target=XRayTarget.Source }; effect.Register(sphere,sphere,true,geometry.XRaySnapshot); effect.Register(panel,panel,false,()=>panelPose);
                int buffers=camera.GetCommandBuffers(CameraEvent.BeforeImageEffects).Length;
                yield return null; yield return null; yield return null; var baseline=Pixels(camera,target); effect.Enabled=true; yield return null; yield return null; yield return null; var hidden=Pixels(camera,target);
                SaveNeutralPixels(baseline,target,"next-xray-neutral-off.png"); SaveNeutralPixels(hidden,target,"next-xray-neutral-on.png");
                int changed=ChangedPixels(baseline,hidden,camera,target,-0.29f,0.09f,-0.27f,0.27f);
                Require(changed>40,"Next hidden-only shader changes occluded neutral-sphere pixels","pixels="+changed);
                int visibleChanges=ChangedPixels(baseline,hidden,camera,target,0.24f,0.45f,-0.2f,0.2f);
                Require(visibleChanges<=2,"unoccluded source fragments keep their standard rendering","pixels="+visibleChanges);
                Require(ChangedPixels(baseline,hidden,camera,target,-0.95f,-0.52f,-0.5f,0.5f)<=2,"outer panel away from the working region remains unchanged");
                Require(panel.sharedMaterial==outer&&sphere.sharedMaterial==original,"X-Ray does not replace either renderer's original materials");
                effect.OriginalTextureBlend=0; var untextured=Pixels(camera,target); Require(ChangedPixels(hidden,untextured,camera,target,-0.29f,0.09f,-0.27f,0.27f)>40,"X-Ray preserves the original sampled texture when texture blend is enabled"); effect.OriginalTextureBlend=1;
                effect.SourceColor=Color.red; var tinted=Pixels(camera,target); Require(ChangedPixels(hidden,tinted,camera,target,-0.29f,0.09f,-0.27f,0.27f)>40,"source tint changes only the independent diagnostic image"); effect.SourceColor=Color.cyan;
                effect.SourceOpacity=0; var zero=Pixels(camera,target); Require(ChangedPixels(baseline,zero,camera,target,-0.29f,0.09f,-0.27f,0.27f)==0,"zero opacity removes the diagnostic pass"); effect.SourceOpacity=0.45f;
                effect.DotDensity=0; zero=Pixels(camera,target); Require(ChangedPixels(baseline,zero,camera,target,-0.29f,0.09f,-0.27f,0.27f)==0,"zero density removes the dot pattern"); effect.DotDensity=0.65f;
                effect.DotPatternEnabled=false; var solid=Pixels(camera,target); Require(ChangedPixels(baseline,solid,camera,target,-0.29f,0.09f,-0.27f,0.27f)>changed,"disabling dots restores a continuous occluded overlay"); effect.DotPatternEnabled=true;
                effect.Target=XRayTarget.Both; var both=Pixels(camera,target); Require(ChangedPixels(hidden,both,camera,target,-0.29f,0.09f,-0.27f,0.27f)<=2,"both-target mode does not draw the source twice"); effect.Target=XRayTarget.Source;
                var secondKey=new object(); effect.Register(secondKey,sphere,true,geometry.XRaySnapshot); var duplicate=Pixels(camera,target);
                Require(ChangedPixels(hidden,duplicate,camera,target,-0.29f,0.09f,-0.27f,0.27f)<=2,"multiple pair registrations do not double-render one source mesh"); effect.Unregister(secondKey);
                effect.IncludeHiddenSource=false; sphere.enabled=false; effect.Enabled=false; effect.Tick(); var hiddenOff=Pixels(camera,target); effect.Enabled=true; var hiddenOn=Pixels(camera,target);
                Require(ChangedPixels(hiddenOff,hiddenOn,camera,target,-0.4f,0.6f,-0.45f,0.45f)==0,"hiding the selected Renderer clears previously recorded overlay commands");
                effect.IncludeHiddenSource=true; var forcedHidden=Pixels(camera,target);
                SaveNeutralPixels(forcedHidden,target,"next-xray-neutral-hidden-source.png");
                Require(ChangedPixels(hiddenOff,forcedHidden,camera,target,-0.29f,0.09f,-0.27f,0.27f)>40,"selected hidden Source receives its own pixel overlay when explicitly enabled");
                sphere.gameObject.SetActive(false); yield return null; var inactiveSource=Pixels(camera,target);
                SaveNeutralPixels(inactiveSource,target,"next-xray-neutral-inactive-source.png");
                Require(ChangedPixels(forcedHidden,inactiveSource,camera,target,-0.29f,0.09f,-0.27f,0.27f)<=2,"inactive Source hierarchy retains the same private X-Ray geometry");
                sphere.gameObject.layer=29; var maskedSource=Pixels(camera,target);
                Require(ChangedPixels(hiddenOff,maskedSource,camera,target,-0.29f,0.09f,-0.27f,0.27f)<=2,"hidden Source rendering respects each camera layer mask"); sphere.gameObject.layer=30;
                effect.Enabled=false; effect.Tick(); var hiddenDisabled=Pixels(camera,target);
                Require(ChangedPixels(hiddenOff,hiddenDisabled,camera,target,-0.4f,0.6f,-0.45f,0.45f)<=2,"disabling X-Ray removes the forced hidden Source pass");
                sphere.gameObject.SetActive(true); effect.Enabled=true;
                sphere.enabled=true; var visibleAgain=Pixels(camera,target); Require(ChangedPixels(hidden,visibleAgain,camera,target,-0.29f,0.09f,-0.27f,0.27f)<=2,"showing the Renderer resumes X-Ray without re-registration");
                panel.enabled=false; effect.Enabled=false; effect.Tick(); var clear=Pixels(camera,target); effect.Enabled=true; var unoccluded=Pixels(camera,target);
                Require(ChangedPixels(clear,unoccluded,camera,target,-0.4f,0.6f,-0.45f,0.45f)<=2,"no occluder produces no hidden-only overlay"); panel.enabled=true;
                var transparentShader=Shader.Find("Unlit/Transparent"); Require(transparentShader!=null,"native transparent fixture shader exists");
                var texture=new Texture2D(1,1,TextureFormat.RGBA32,false); texture.SetPixel(0,0,new Color(0.55f,0.22f,0.09f,0.35f)); texture.Apply(); assets.Add(texture);
                var transparent=new Material(transparentShader); transparent.mainTexture=texture; assets.Add(transparent); panel.sharedMaterial=transparent;
                effect.Enabled=false; effect.Tick(); var transparentOff=Pixels(camera,target); effect.Enabled=true; var transparentOn=Pixels(camera,target);
                Require(ChangedPixels(transparentOff,transparentOn,camera,target,-0.29f,0.09f,-0.27f,0.27f)>40,"private depth detects a transparent occluding panel");
                Require(panel.sharedMaterial==transparent,"private depth leaves transparent original material intact");
                var secondObject=new GameObject("BPN second pixel camera"); secondObject.transform.SetParent(root.transform,false); secondObject.transform.localPosition=new Vector3(0,0,-4);
                var second=secondObject.AddComponent<Camera>(); second.CopyFrom(camera); second.orthographic=false; second.fieldOfView=26;
                var perspectiveOn=Pixels(second,target); effect.Enabled=false; effect.Tick(); var perspectiveOff=Pixels(second,target);
                Require(ChangedPixels(perspectiveOff,perspectiveOn,second,target,-0.29f,0.09f,-0.27f,0.27f)>20,"second perspective camera receives its own occlusion pass");
                Require(camera.GetCommandBuffers(CameraEvent.BeforeImageEffects).Length==buffers&&second.GetCommandBuffers(CameraEvent.BeforeImageEffects).Length==0,"disabling X-Ray releases both camera command buffers");
                second.enabled=false;
                var shell=Skin("Neutral surrounding diagnostic shell",PrimitiveMeshes.Sphere(48,24,.6),new Vector3(.1f,0,.55f));shell.gameObject.layer=30;
                var shellPose=new Mesh();assets.Add(shellPose);shell.BakeMesh(shellPose);
                effect.Register(shell,shell,false,()=>shellPose);effect.Target=XRayTarget.Both;effect.Occlusion=OcclusionMode.AllSelected;effect.DotPatternEnabled=false;effect.OriginalTextureBlend=0;effect.SourceColor=Color.cyan;effect.ReceiverColor=Color.red;effect.SourceOpacity=effect.ReceiverOpacity=.9f;effect.Enabled=true;
                var nested=Pixels(camera,target);var centre=camera.WorldToScreenPoint(root.transform.position+new Vector3(.1f,0,.55f));var pixel=nested[(int)centre.y*target.width+(int)centre.x];
                Require(pixel.g+pixel.b>pixel.r*2+30,"nested Source stays distinguishable inside a Receiver overlay");
                SaveNeutralPixels(nested,target,"next-xray-neutral-nested-source.png");
                effect.Enabled=false;effect.Tick();var sharedShader=effect.DiagnosticShader;Require(sharedShader!=null,"native diagnostic shader remains available");
                using(var reversed=new XRayRenderer(sharedShader){Target=XRayTarget.Both,Occlusion=OcclusionMode.AllSelected,DotPatternEnabled=false,OriginalTextureBlend=0,SourceColor=Color.cyan,ReceiverColor=Color.red,SourceOpacity=.9f,ReceiverOpacity=.9f,Enabled=true})
                {
                    reversed.Register(shell,shell,false,()=>shellPose);reversed.Register(sphere,sphere,true,geometry.XRaySnapshot);var reversePixels=Pixels(camera,target);
                    Require(ChangedPixels(nested,reversePixels,camera,target,-.29f,.09f,-.27f,.27f)<=2,"nested overlay visibility is independent of registration order");
                }
                shell.enabled=false;
                effect.Dispose(); effect=null; geometry.Dispose(); yield return null; yield return null;
                int owned=0; foreach(var m in Resources.FindObjectsOfTypeAll<Material>()) if(m.name.StartsWith("BPN diagnostic surface",StringComparison.Ordinal)) owned++;
                Require(owned==0,"disposal releases all diagnostic surface materials");
            }
            finally { if(effect!=null) effect.Dispose(); geometry.Dispose(); target.Release(); Object.Destroy(go); panel.enabled=false; sphere.enabled=false; }
        }
        Color32[] Pixels(Camera camera,RenderTexture target)
        {
            camera.Render(); var previous=RenderTexture.active; RenderTexture.active=target; var image=new Texture2D(target.width,target.height,TextureFormat.RGB24,false);
            try { image.ReadPixels(new Rect(0,0,target.width,target.height),0,0); image.Apply(); return image.GetPixels32(); }
            finally { RenderTexture.active=previous; Object.Destroy(image); }
        }
        void SaveNeutralPixels(Color32[] pixels,RenderTexture target,string name)
        { var image=new Texture2D(target.width,target.height,TextureFormat.RGB24,false); try { image.SetPixels32(pixels); image.Apply(); System.IO.File.WriteAllBytes(System.IO.Path.Combine(System.IO.Path.GetDirectoryName(output),name),image.EncodeToPNG()); } finally { Object.Destroy(image); } }
        int ChangedPixels(Color32[] before,Color32[] after,Camera camera,RenderTexture target,float left,float right,float bottom,float top)
        {
            var a=camera.WorldToScreenPoint(root.transform.position+new Vector3(left,bottom,0.55f)); var b=camera.WorldToScreenPoint(root.transform.position+new Vector3(right,top,0.55f)); int count=0;
            for(int y=Mathf.Max(0,(int)a.y);y<Mathf.Min(target.height,(int)b.y);y++) for(int x=Mathf.Max(0,(int)a.x);x<Mathf.Min(target.width,(int)b.x);x++)
            { var p=before[y*target.width+x]; var q=after[y*target.width+x]; if(Math.Abs(p.r-q.r)+Math.Abs(p.g-q.g)+Math.Abs(p.b-q.b)>12) count++; } return count;
        }
    }
}
