using System;
using System.IO;
using System.Linq;
using System.Reflection;
using BepInEx.Configuration;
using BetterPenetrationNext;
using BetterPenetrationNext.Core;

public static class DefaultProfileChecks
{
    static int count;
    static bool colors;
    static void Check(bool value,string reason) { if(!value) throw new Exception(reason); }
    static void Pass(string name,Action test) { test(); count++; Console.WriteLine("PASS "+name); }
    static Type SettingType(string name)
    {
        switch(name)
        {
            case "Single": return typeof(float);
            case "Boolean": return typeof(bool);
            case "Int32": return typeof(int);
            case "String": return typeof(string);
            case "Color": return typeof(UnityEngine.Color);
            default: return typeof(ReleaseDefaults).Assembly.GetType("BetterPenetrationNext."+name,false)??
                typeof(ReleaseDefaults).Assembly.GetType("BetterPenetrationNext.Core."+name,true);
        }
    }
    static ConfigFile BindAll(string path,ReleaseDefaults defaults)
    {
        var file=new ConfigFile(path,false) { SaveOnConfigSet=false };
        var bind=typeof(ReleaseDefaults).GetMethods().Single(m=>m.Name=="Bind"&&m.GetParameters()[3].ParameterType==typeof(string));
        foreach(var row in defaults.Profile.Settings)
        {
            if(!colors&&row.Type=="Color") continue;
            bind.MakeGenericMethod(SettingType(row.Type)).Invoke(defaults,new object[]{file,row.Section,row.Key,"Default profile test"});
        }
        return file;
    }
    static string Snapshot(ConfigFile file)
    {
        return string.Join("\n",file.Select(pair=>pair.Key.Section+"/"+pair.Key.Key+"="+pair.Value.GetSerializedValue()).OrderBy(s=>s,StringComparer.Ordinal));
    }
    static void SameValuesAndDefaults(ConfigFile a,ConfigFile b)
    {
        Check(Snapshot(a)==Snapshot(b),"Effective settings differ");
        foreach(var pair in a)
        {
            var other=b[pair.Key];
            Check(Equals(pair.Value.BoxedValue,other.DefaultValue),"Value and Reset default differ: "+pair.Key);
        }
    }
    static void Rejected(Action action,string reason)
    {
        bool rejected=false;
        try { action(); } catch { rejected=true; }
        Check(rejected,reason);
    }
    static int Main(string[] args)
    { return Run(args,false); }

    public static int Run(string[] args,bool includeNativeColors)
    {
        try
        {
            count=0; colors=includeNativeColors;
            if(!colors) Console.WriteLine("SCOPE: managed settings; the two native Unity color converters are checked only in the Unity host.");
            string profilePath=Path.GetFullPath(args[0]),root=Path.GetFullPath(args[1]);
            Directory.CreateDirectory(root);
            string json=File.ReadAllText(profilePath);
            var defaults=new ReleaseDefaults(json,root);
            string clean=Path.Combine(root,"clean.cfg"),updated=Path.Combine(root,"updated.cfg");
            ConfigFile first=null;
            Pass("profile bytes equal the resource in the target net46 DLL",()=>
            {
                using(var stream=typeof(ReleaseDefaults).Assembly.GetManifestResourceStream("BetterPenetrationNext.release-defaults.json"))
                using(var reader=new StreamReader(stream)) Check(reader.ReadToEnd()==json,"Embedded profile is stale");
            });
            Pass("clean BepInEx config binds every typed profile setting",()=>
            {
                Check(!File.Exists(clean),"Choose a new test directory");
                first=BindAll(clean,defaults); first.Save();
                Check(first.Count==defaults.Profile.Settings.Count(r=>colors||r.Type!="Color"),"Incomplete binding");
                foreach(var pair in first) Check(Equals(pair.Value.BoxedValue,pair.Value.DefaultValue),"Wrong initial default: "+pair.Key);
                Check(first[new ConfigDefinition("Next.Contact","RecoveryTime")].BoxedValue.Equals(4.5f),"RecoveryTime changed");
            });
            Pass("deleting the saved config regenerates identical values and Reset defaults",()=>
            {
                File.Delete(clean);
                var recreated=BindAll(clean,new ReleaseDefaults(json,root)); recreated.Save();
                SameValuesAndDefaults(first,recreated);
            });
            Pass("saving and reloading preserves every value",()=>SameValuesAndDefaults(first,BindAll(clean,defaults)));
            Pass("existing user values survive ordinary plugin startup",()=>
            {
                File.WriteAllText(updated,"[Next.Contact]\nContactDamping = 3\n[Other]\nUntouched = user value\n");
                var loaded=BindAll(updated,defaults);
                var entry=loaded[new ConfigDefinition("Next.Contact","ContactDamping")];
                Check(entry.BoxedValue.Equals(3f),"Startup overwrote a user setting");
                Check(entry.DefaultValue.Equals(defaults.Value<float>("Next.Contact","ContactDamping")),"Reset uses an old default");
                entry.BoxedValue=entry.DefaultValue;
                Check(entry.BoxedValue.Equals(defaults.Value<float>("Next.Contact","ContactDamping")),"Reset did not use the profile");
            });
            Pass("profile omissions, duplicate keys, wrong types and invalid enums are rejected",()=>
            {
                Rejected(()=>defaults.Value<float>("Missing","Entry"),"Missing default accepted");
                Rejected(()=>defaults.Value<int>("Next.Contact","ContactDamping"),"Wrong default type accepted");
                var p=JsonCodec.Deserialize<ReleaseDefaultProfile>(json);
                p.Settings=p.Settings.Concat(new[]{p.Settings[0]}).ToArray();
                Rejected(()=>new ReleaseDefaults(JsonCodec.Serialize(p),root),"Duplicate accepted");
                p=JsonCodec.Deserialize<ReleaseDefaultProfile>(json);
                p.Settings.Single(r=>r.Key=="ContactMode").Value="12345";
                var invalid=new ReleaseDefaults(JsonCodec.Serialize(p),root);
                Rejected(()=>invalid.Value<ContactMode>("Next.Contact","ContactMode"),"Undefined enum accepted");
            });
            Pass("non-finite numbers and game-root path escapes are rejected",()=>
            {
                var p=JsonCodec.Deserialize<ReleaseDefaultProfile>(json);
                p.Settings.Single(r=>r.Key=="ContactDamping").Value="NaN";
                var invalid=new ReleaseDefaults(JsonCodec.Serialize(p),root);
                Rejected(()=>invalid.Value<float>("Next.Contact","ContactDamping"),"NaN accepted");
                Rejected(()=>ReleaseDefaultProfile.Expand("@game/../outside.cfg",root),"Path escape accepted");
            });
            Pass("installer-generated config equals clean BepInEx defaults",()=>
            {
                string installed=Path.Combine(root,"installer.cfg");
                Check(File.Exists(installed),"Run the installer merge test first");
                SameValuesAndDefaults(first,BindAll(installed,defaults));
            });
            File.WriteAllText(Path.Combine(root,"effective-settings.txt"),Snapshot(first));
            Console.WriteLine("RESULT "+count+" defaults tests passed; settings="+first.Count+"; scope="+(colors?"isolated Unity player configuration":"managed BepInEx, not KKS Runtime"));
            return 0;
        }
        catch(Exception e) { Console.Error.WriteLine(e); return 1; }
    }
}
