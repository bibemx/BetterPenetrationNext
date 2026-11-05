using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;

namespace BetterPenetrationNext.Core
{
    [DataContract]
    public sealed class ReleaseDefaultSetting
    {
        [DataMember] public string Section;
        [DataMember] public string Key;
        [DataMember] public string Type;
        [DataMember] public string Value;
    }

    [DataContract]
    public sealed class ReleaseDefaultProfile
    {
        [DataMember] public int Schema;
        [DataMember] public string Id;
        [DataMember] public string Version;
        [DataMember] public string Status;
        [DataMember] public ReleaseDefaultSetting[] Settings;

        public Dictionary<string,ReleaseDefaultSetting> Index()
        {
            if(Schema!=1||string.IsNullOrEmpty(Id)||string.IsNullOrEmpty(Version)||
                (Status!="candidate"&&Status!="validated")||Settings==null||Settings.Length==0)
                throw new InvalidDataException("Invalid release defaults header.");
            var result=new Dictionary<string,ReleaseDefaultSetting>(StringComparer.Ordinal);
            foreach(var row in Settings)
            {
                if(row==null||string.IsNullOrEmpty(row.Section)||string.IsNullOrEmpty(row.Key)||
                    string.IsNullOrEmpty(row.Type)||row.Value==null||
                    row.Section.IndexOfAny(new[]{'\r','\n','[',']','\0'})>=0||
                    row.Key.IndexOfAny(new[]{'\r','\n','[',']','=','\0'})>=0||
                    row.Value.IndexOfAny(new[]{'\r','\n','\0'})>=0)
                    throw new InvalidDataException("Invalid release default entry.");
                string identity=Identity(row.Section,row.Key);
                if(result.ContainsKey(identity)) throw new InvalidDataException("Duplicate release default: "+row.Section+"/"+row.Key);
                result.Add(identity,row);
            }
            return result;
        }

        public static string Identity(string section,string key) { return section+"\0"+key; }

        public static string Expand(string value,string gameRoot)
        {
            const string prefix="@game/";
            if(!value.StartsWith(prefix,StringComparison.Ordinal)) return value;
            string relative=value.Substring(prefix.Length).Replace('/',Path.DirectorySeparatorChar);
            string root=Path.GetFullPath(gameRoot).TrimEnd(Path.DirectorySeparatorChar,Path.AltDirectorySeparatorChar);
            string path=Path.GetFullPath(Path.Combine(root,relative));
            if(!path.StartsWith(root+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))
                throw new InvalidDataException("A release default path leaves the game directory.");
            return path;
        }
    }
}
