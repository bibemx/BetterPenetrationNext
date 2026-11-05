using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using BepInEx;
using BepInEx.Configuration;
using BetterPenetrationNext.Core;

namespace BetterPenetrationNext
{
    // Read once at initialization. The immutable resource is also the installer's profile.
    public sealed class ReleaseDefaults
    {
        public readonly ReleaseDefaultProfile Profile;
        public readonly string Sha256;
        readonly Dictionary<string,ReleaseDefaultSetting> entries;
        readonly string gameRoot;

        public ReleaseDefaults(string json,string root)
        {
            Profile=JsonCodec.Deserialize<ReleaseDefaultProfile>(json);
            if(Profile==null) throw new InvalidDataException("Missing release defaults.");
            entries=Profile.Index(); gameRoot=root;
            using(var hash=SHA256.Create()) Sha256=BitConverter.ToString(hash.ComputeHash(System.Text.Encoding.UTF8.GetBytes(json))).Replace("-","");
        }

        public static ReleaseDefaults LoadEmbedded()
        {
            using(var stream=typeof(ReleaseDefaults).Assembly.GetManifestResourceStream("BetterPenetrationNext.release-defaults.json"))
            {
                if(stream==null) throw new InvalidDataException("Release defaults were not embedded in the DLL.");
                using(var reader=new StreamReader(stream)) return new ReleaseDefaults(reader.ReadToEnd(),Paths.GameRootPath);
            }
        }

        public T Value<T>(string section,string key)
        {
            ReleaseDefaultSetting row;
            if(!entries.TryGetValue(ReleaseDefaultProfile.Identity(section,key),out row))
                throw new InvalidDataException("Release default is absent: "+section+"/"+key);
            if(row.Type!=typeof(T).Name) throw new InvalidDataException("Release default type mismatch: "+section+"/"+key);
            string serialized=ReleaseDefaultProfile.Expand(row.Value,gameRoot);
            var value=(T)TomlTypeConverter.ConvertToValue(serialized,typeof(T));
            if(typeof(T).IsEnum&&!Enum.IsDefined(typeof(T),value)) throw new InvalidDataException("Undefined enum in release defaults: "+key);
            if(value is float&&(float.IsNaN((float)(object)value)||float.IsInfinity((float)(object)value)))
                throw new InvalidDataException("Non-finite release default: "+key);
            if(value is double&&(double.IsNaN((double)(object)value)||double.IsInfinity((double)(object)value)))
                throw new InvalidDataException("Non-finite release default: "+key);
            return value;
        }

        public ConfigEntry<T> Bind<T>(ConfigFile config,string section,string key,string description)
        { return config.Bind(section,key,Value<T>(section,key),NextRuntime.CurrentDescription(section,key,new ConfigDescription(description))); }
        public ConfigEntry<T> Bind<T>(ConfigFile config,string section,string key,ConfigDescription description)
        { return config.Bind(section,key,Value<T>(section,key),NextRuntime.CurrentDescription(section,key,description)); }

        public ConfigEntry<T> BindLegacy<T>(ConfigFile config,string section,string key,T legacyTypeHint,ConfigDescription description)
        { return config.Bind(section,key,Value<T>(section,key),NextRuntime.CurrentDescription(section,key,description)); }

        public ConfigEntry<T> BindLegacy<T>(ConfigFile config,string section,string key,T legacyTypeHint,string description)
        { return Bind<T>(config,section,key,description); }

        public ConfigEntry<T> BindAssetDefault<T>(ConfigFile config,string section,string key,T assetDefault,string description)
        {
            // Third-party assets can add shape controls absent from the supported release asset.
            return entries.ContainsKey(ReleaseDefaultProfile.Identity(section,key))?
                Bind<T>(config,section,key,description):config.Bind(section,key,assetDefault,description);
        }

        public int VerifyEffectiveConfiguration(ConfigFile config,bool includeAssetControls)
        {
            int checkedCount=0;
            foreach(var row in Profile.Settings)
            {
                if(!includeAssetControls&&row.Section=="Next.PreparedMorphs") continue;
                var definition=new ConfigDefinition(row.Section,row.Key);
                if(!config.ContainsKey(definition)) throw new InvalidDataException("Unbound release setting: "+definition);
                var entry=config[definition];
                if(entry.SettingType.Name!=row.Type) throw new InvalidDataException("Bound setting has wrong type: "+definition);
                object expected=typeof(ReleaseDefaults).GetMethod("Value").MakeGenericMethod(entry.SettingType).Invoke(this,new object[]{row.Section,row.Key});
                if(!Equals(entry.BoxedValue,expected)||!Equals(entry.DefaultValue,expected))
                    throw new InvalidDataException("Effective setting differs from release defaults: "+definition);
                checkedCount++;
            }
            return checkedCount;
        }
    }
}
