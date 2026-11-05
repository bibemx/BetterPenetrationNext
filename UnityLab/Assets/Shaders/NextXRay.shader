Shader "Hidden/BetterPenetrationNext/PreparedXRay"
{
    Properties
    {
        _MainTex("Authored texture",2D)="white"{}
        _Tint("Tint",Color)=(0,1,1,1)
        _Opacity("Opacity",Range(0,1))=0.45
        _TextureBlend("Original texture blend",Range(0,1))=1
        _Dots("Dot mask",Float)=1
        _DotScale("Cell size pixels",Float)=8
        _DotDensity("Occupied cells",Range(0,1))=0.5
    }
    SubShader
    {
        Tags { "Queue"="Overlay" "RenderType"="Transparent" }
        CGINCLUDE
        #include "UnityCG.cginc"
        sampler2D_float _CameraDepthTexture;
        sampler2D _MainTex,_BPNSelfDepth,_BPNOtherDepth,_BPNExtraDepth;
        float4 _MainTex_ST,_Tint;
        float _Opacity,_TextureBlend,_Dots,_DotScale,_DotDensity,_BPNFar,_BPNFlip;
        struct appdata { float4 vertex:POSITION; float2 uv:TEXCOORD0; };
        struct v2f { float4 position:SV_POSITION; float4 screen:TEXCOORD0; float4 clip:TEXCOORD1; float eye:TEXCOORD2; float2 uv:TEXCOORD3; };
        v2f vert(appdata v)
        { v2f o; o.position=UnityObjectToClipPos(v.vertex); o.clip=o.position; o.screen=ComputeScreenPos(o.position); o.eye=-UnityObjectToViewPos(v.vertex).z; o.uv=TRANSFORM_TEX(v.uv,_MainTex); return o; }
        float2 PrivateUV(v2f i)
        {
            float2 uv=i.clip.xy/i.clip.w*0.5+0.5;
            #if UNITY_UV_STARTS_AT_TOP
            uv.y=1-uv.y;
            #endif
            if(_BPNFlip<0) uv.y=1-uv.y;
            return uv;
        }
        float SceneEye(v2f i)
        {
            float raw=SAMPLE_DEPTH_TEXTURE_PROJ(_CameraDepthTexture,UNITY_PROJ_COORD(i.screen));
            if(unity_OrthoParams.w>0.5)
            {
                #if defined(UNITY_REVERSED_Z)
                raw=1-raw;
                #endif
                return lerp(_ProjectionParams.y,_ProjectionParams.z,raw);
            }
            return LinearEyeDepth(raw);
        }
        float Bias(v2f i) { return max(0.0001,i.eye*0.00001); }
        float PrivateEye(v2f i)
        { float2 uv=PrivateUV(i); return min(tex2D(_BPNOtherDepth,uv).r,tex2D(_BPNExtraDepth,uv).r)*_BPNFar; }
        fixed4 Overlay(v2f i)
        {
            clip(tex2D(_BPNSelfDepth,PrivateUV(i)).r*_BPNFar+Bias(i)-i.eye);
            float coverage=1;
            if(_Dots>0.5)
            {
                clip(_DotDensity-0.000001);
                float2 cells=i.screen.xy/i.screen.w*_ScreenParams.xy/max(2,_DotScale),id=floor(cells),p=frac(cells)-0.5;
                float hash=frac(sin(dot(id,float2(127.1,311.7)))*43758.5453); clip(_DotDensity-hash);
                float d=length(p),edge=max(fwidth(d),0.5/max(2,_DotScale)); coverage=1-smoothstep(0.24-edge,0.24+edge,d); clip(coverage-0.01);
            }
            fixed3 textureColor=tex2D(_MainTex,i.uv).rgb;
            return fixed4(lerp(fixed3(1,1,1),textureColor,_TextureBlend)*_Tint.rgb,_Opacity*_Tint.a*coverage);
        }
        float4 fragDepth(v2f i):SV_Target { return i.eye/max(0.001,_BPNFar); }
        fixed4 fragOpaque(v2f i):SV_Target { clip(i.eye-SceneEye(i)-Bias(i)); return Overlay(i); }
        fixed4 fragTransparent(v2f i):SV_Target { clip(i.eye-PrivateEye(i)-Bias(i)); clip(SceneEye(i)+Bias(i)-i.eye); return Overlay(i); }
        fixed4 fragAll(v2f i):SV_Target { return Overlay(i); }
        ENDCG
        Pass
        {
            Name "PRIVATE_DEPTH" ZTest LEqual ZWrite On Cull Off Blend Off
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment fragDepth
            ENDCG
        }
        Pass
        {
            Name "OPAQUE_OCCLUDED" ZTest Greater ZWrite Off Cull Off Blend SrcAlpha OneMinusSrcAlpha
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment fragOpaque
            ENDCG
        }
        Pass
        {
            Name "TRANSPARENT_OCCLUDED" ZTest Always ZWrite Off Cull Off Blend SrcAlpha OneMinusSrcAlpha
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment fragTransparent
            ENDCG
        }
        Pass
        {
            Name "ALL_SELECTED_DIAGNOSTIC" ZTest Always ZWrite Off Cull Off Blend SrcAlpha OneMinusSrcAlpha
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment fragAll
            ENDCG
        }
    }
    Fallback Off
}
