using System.IO;
using System.Runtime.Serialization.Json;
using System.Text;

namespace BetterPenetrationNext.Core
{
    public static class JsonCodec
    {
        // BepInEx-loaded user struct arrays are omitted by native JsonUtility in
        // this KKS player. Use the same managed codec in Editor and runtime.
        static DataContractJsonSerializer Serializer<T>()
        { return new DataContractJsonSerializer(typeof(T),new DataContractJsonSerializerSettings { MaxItemsInObjectGraph=4000000 }); }
        public static string Serialize<T>(T value)
        { using(var stream=new MemoryStream()) { Serializer<T>().WriteObject(stream,value); return Encoding.UTF8.GetString(stream.ToArray()); } }
        public static T Deserialize<T>(string value)
        { using(var stream=new MemoryStream(Encoding.UTF8.GetBytes(value))) return (T)Serializer<T>().ReadObject(stream); }
    }
}
