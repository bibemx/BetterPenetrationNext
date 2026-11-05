using System;
using BepInEx.Configuration;
using UnityEngine;
namespace BetterPenetrationNext
{
    public enum OperatingMode { BPN,BP }
    // Field names are the ConfigurationManager's documented tag contract.
    public sealed class ConfigurationManagerAttributes { public bool? Browsable; }
    public sealed partial class NextRuntime
    {
        ConfigEntry<OperatingMode> operatingMode;
        ConfigEntry<KeyboardShortcut> xrayShortcut,menuShortcut;
        bool menuOpen,changingOperatingMode;Rect menuRect=new Rect(80,80,520,640);Vector2 menuScroll;
        static bool CurrentSetting(string section,string key)
        {
            if(section=="Next.Controls"||section=="Next.Source"||section=="Next.XRay")return true;
            if(section=="Next.Receiver")return key=="CervixMaxDisplacement"||key=="CervixFollowStrength"||key=="CanalLengthScale";
            if(section=="Next.Contact")return key=="RecoveryTime"||key=="ContactDamping"||key=="MaxTissueStrain";
            if(section=="Next.Performance")return key=="BackgroundRig";
            return false;
        }
        internal static ConfigDescription CurrentDescription(string section,string key,ConfigDescription description)
        {
            if(!section.StartsWith("Next",StringComparison.Ordinal)||CurrentSetting(section,key))return description;
            var tags=new object[description.Tags.Length+1];Array.Copy(description.Tags,tags,description.Tags.Length);tags[tags.Length-1]=new ConfigurationManagerAttributes{Browsable=false};
            return new ConfigDescription(description.Description,description.AcceptableValues,tags);
        }
        void InitializeControlWindow()
        {
            operatingMode=configuration.Bind("Next.Controls","Mode",mode.Value==NextMode.Original?OperatingMode.BP:OperatingMode.BPN,"BPN: current reactive rig. BP: original retained BetterPenetration controller.");
            xrayShortcut=configuration.Bind("Next.Controls","ToggleXRay",new KeyboardShortcut(KeyCode.B),"Toggle X-Ray directly (default B).");
            menuShortcut=configuration.Bind("Next.Controls","OpenSettings",new KeyboardShortcut(KeyCode.N),"Open this plugin's settings window directly (default N).");
            operatingMode.SettingChanged+=(s,e)=>ApplyOperatingMode();
            mode.SettingChanged+=(s,e)=>{if(!changingOperatingMode){changingOperatingMode=true;operatingMode.Value=mode.Value==NextMode.Original?OperatingMode.BP:OperatingMode.BPN;changingOperatingMode=false;}};
            ApplyOperatingMode();
        }
        void ApplyOperatingMode()
        {
            if(changingOperatingMode)return;changingOperatingMode=true;
            try{if(operatingMode.Value==OperatingMode.BP){contactMode.Value=Core.ContactMode.Original;mode.Value=NextMode.Original;}
                else{anatomicalReceiver.Value=true;reactiveRig.Value=true;wholeSource.Value=true;pathMode.Value=Core.PathMode.SourceFollowsPath;contactMode.Value=Core.ContactMode.Bidirectional;mode.Value=NextMode.SoftBody;}
                transition.Request((int)mode.Value);dirty=true;
            }finally{changingOperatingMode=false;}
        }
        void Update()
        {
            if(xrayShortcut==null)return;
            if(xrayShortcut.Value.IsDown())xrayEnabled.Value=!xrayEnabled.Value;
            if(menuShortcut.Value.IsDown())menuOpen=!menuOpen;
        }
        void DrawControlWindow()
        {
            if(!menuOpen||operatingMode==null)return;
            menuRect.width=Math.Min(540,Screen.width-20);menuRect.height=Math.Min(650,Screen.height-20);
            menuRect=GUI.Window(109826,menuRect,ControlWindow,"BetterPenetrationNext 1.0.9");
        }
        void ControlWindow(int id)
        {
            GUILayout.BeginHorizontal();if(GUILayout.Toggle(operatingMode.Value==OperatingMode.BPN,"BPN",GUI.skin.button))operatingMode.Value=OperatingMode.BPN;if(GUILayout.Toggle(operatingMode.Value==OperatingMode.BP,"BP",GUI.skin.button))operatingMode.Value=OperatingMode.BP;if(GUILayout.Button("Закрыть",GUILayout.Width(90)))menuOpen=false;GUILayout.EndHorizontal();
            menuScroll=GUILayout.BeginScrollView(menuScroll);
            GUILayout.Label("BPN — реактивный риг. BP — оригинальный BetterPenetration.");
            if(operatingMode.Value==OperatingMode.BPN){
                GUILayout.Space(8);GUILayout.Label("Размер источника");Slider("Длина",sourceLengthScale,.5f,1.5f);Slider("Ширина",sourceWidthScale,.5f,1.5f);
                GUILayout.Space(8);GUILayout.Label("Получатель");Slider("Исходная длина канала",canalLengthScale,1,1.5f);Slider("Предел сжатия шейки",cervixMaxDisplacement,0,.08f);Slider("Реакция шейки",cervixFollowStrength,0,1);Slider("Плавность контакта",contactDamping,1,24);Slider("Восстановление после контакта, с",recoveryTime,0,10);
                GUILayout.Label("Контакт сжимает шейку в пределах её геометрии; матка и длина канала от нагрузки не меняются.");
                backgroundRig.Value=GUILayout.Toggle(backgroundRig.Value,"Расчёт на отдельном потоке");
            }
            GUILayout.Space(8);GUILayout.Label("X-Ray");xrayEnabled.Value=GUILayout.Toggle(xrayEnabled.Value,"Включён");
            GUILayout.BeginHorizontal();if(GUILayout.Toggle(xrayTarget.Value==XRayTarget.Both,"Оба меша",GUI.skin.button))xrayTarget.Value=XRayTarget.Both;if(GUILayout.Toggle(xrayTarget.Value==XRayTarget.Source,"Источник",GUI.skin.button))xrayTarget.Value=XRayTarget.Source;if(GUILayout.Toggle(xrayTarget.Value==XRayTarget.Receiver,"Получатель",GUI.skin.button))xrayTarget.Value=XRayTarget.Receiver;GUILayout.EndHorizontal();
            Slider("Текстуры",textureBlend,0,1);Slider("Видимость источника",sourceOpacity,0,1);Slider("Видимость получателя",receiverOpacity,0,1);dots.Value=GUILayout.Toggle(dots.Value,"Точечный индикатор X-Ray");
            GUILayout.Space(8);GUILayout.Label("X-Ray: "+xrayShortcut.Value+"    Настройки: "+menuShortcut.Value);GUILayout.Label("Изменить клавиши можно в Next.Controls в Plugin Settings.");
            GUILayout.EndScrollView();GUI.DragWindow(new Rect(0,0,10000,22));
        }
        static void Slider(string label,ConfigEntry<float> entry,float minimum,float maximum)
        {
            GUILayout.Label(label+": "+entry.Value.ToString("0.###"));float before=entry.Value,value=GUILayout.HorizontalSlider(before,minimum,maximum);if(Math.Abs(value-before)>1e-6f)entry.Value=value;
        }
    }
}
