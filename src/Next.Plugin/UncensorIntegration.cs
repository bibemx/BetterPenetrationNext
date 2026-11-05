using System;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using BepInEx;
using UnityEngine;
using BetterPenetrationNext.Core;

namespace BetterPenetrationNext
{
    public static class UncensorIntegration
    {
        public const string PreparedGuid="org.betterpenetration.next.body.v1";
        static Type controllerType;
        static PropertyInfo dataProperty;
        static FieldInfo guidField;
        public static void Initialize()
        {
            controllerType=Type.GetType("KK_Plugins.UncensorSelector+UncensorSelectorController, KKS_UncensorSelector",false);
            if(controllerType==null) throw new MissingMemberException("Installed KKS UncensorSelector controller is absent.");
            dataProperty=controllerType.GetProperty("BodyData",BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic);
            if(dataProperty==null) throw new MissingMemberException("Installed selector has no effective BodyData property.");
            guidField=dataProperty.PropertyType.GetField("BodyGUID",BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic);
            if(guidField==null) throw new MissingMemberException("Effective body resource identity cannot be read.");
        }
        public static string EffectiveGuid(ChaControl character)
        {
            if(controllerType==null||character==null) return null;
            var controller=character.GetComponent(controllerType); if(controller==null) return null;
            object data=dataProperty.GetValue(controller,null); return data==null?null:guidField.GetValue(data) as string;
        }
        public static SimulationDefinition LoadDefinition(string path)
        {
            if(!File.Exists(path)) return null;
            using(var file=File.OpenRead(path)) using(var zip=new ZipArchive(file,ZipArchiveMode.Read))
            {
                var entry=zip.GetEntry("bpn/simulation.json");
                if(entry==null||entry.Length>32*1024*1024) throw new InvalidDataException("Simulation metadata is absent or too large.");
                using(var input=entry.Open()) using(var reader=new StreamReader(input))
                { var value=JsonCodec.Deserialize<SimulationDefinition>(reader.ReadToEnd()); if(value==null||value.BodyGuid!=PreparedGuid||value.Schema!=1) throw new InvalidDataException("Unrecognized prepared resource identity."); return value; }
            }
        }
        public static string DefaultPackage { get { return ReleaseDefaults.LoadEmbedded().Value<string>("Next.Assets","PreparedZipmod"); } }
        public static ChaControl Character(object controller)
        {
            var component=controller as Component; if(component==null) return null;
            var property=controller.GetType().GetProperty("ChaControl",BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic);
            return property==null?component.GetComponent<ChaControl>():property.GetValue(controller,null) as ChaControl;
        }
    }
}
