using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Object=UnityEngine.Object;
namespace BetterPenetrationNext
{
    // Camera-local diagnostic geometry; never changes any model material.
    public sealed class ContactDebugOverlay : IDisposable
    {
        readonly List<Vector3> points=new List<Vector3>(2048);
        readonly List<Color> colors=new List<Color>(2048);
        readonly Material material;
        public ContactDebugOverlay()
        {
            var shader=Shader.Find("Hidden/Internal-Colored"); if(shader==null) return;
            material=new Material(shader) { hideFlags=HideFlags.HideAndDontSave };
            material.SetInt("_SrcBlend",(int)BlendMode.SrcAlpha); material.SetInt("_DstBlend",(int)BlendMode.OneMinusSrcAlpha);
            material.SetInt("_Cull",(int)CullMode.Off); material.SetInt("_ZWrite",0); material.SetInt("_ZTest",(int)CompareFunction.Always);
            Camera.onPostRender+=Render;
        }
        public void Clear() { points.Clear(); colors.Clear(); }
        public void Line(Vector3 a,Vector3 b,Color color) { if(points.Count>=32768) return; points.Add(a); points.Add(b); colors.Add(color); colors.Add(color); }
        void Render(Camera camera)
        {
            if(material==null||points.Count==0||camera==null||camera.cameraType==CameraType.Preview) return;
            material.SetPass(0); GL.PushMatrix(); GL.MultMatrix(Matrix4x4.identity); GL.Begin(GL.LINES);
            for(int i=0;i<points.Count;i++) { GL.Color(colors[i]); GL.Vertex(points[i]); }
            GL.End(); GL.PopMatrix();
        }
        public void Dispose() { Camera.onPostRender-=Render; if(material!=null) Object.Destroy(material); points.Clear(); colors.Clear(); }
    }
}
