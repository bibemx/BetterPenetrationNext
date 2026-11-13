using System;
using System.IO;
using System.IO.Compression;
using System.Xml.Linq;
using BetterPenetrationNext.Core;

namespace BetterPenetrationNext
{
    // An immutable, loading-time contract. Each character owns a reference to
    // its selected package; switching bodies cannot reuse another body's map.
    public sealed class PreparedReceiverPackage
    {
        [Serializable] public sealed class Descriptor
        {
            public int Schema,VisualVertices;
            public string Guid,Name,Bundle,Asset,VisualMeshSignature;
        }
        public string FilePath,Guid,Bundle,Asset;
        public int VisualVertices;
        public SimulationDefinition Definition,Physical,Support;
        public ReceiverSupportTransfer Transfer,ReactiveTransfer;
        public ReceiverGuideDefinition Contours;
        public AnimatedPathProfile Profile;
        public ReactiveMaterialChart Chart;
        public ClosedEntranceDefinition ClosedEntrance;

        static T Read<T>(ZipArchive zip,string name)
        {
            var entry=zip.GetEntry(name);
            if(entry==null||entry.Length>32*1024*1024)throw new InvalidDataException("Missing/oversized receiver data: "+name);
            using(var reader=new StreamReader(entry.Open()))return JsonCodec.Deserialize<T>(reader.ReadToEnd());
        }
        public static PreparedReceiverPackage Load(string path)
        {
            using(var file=File.OpenRead(path))using(var zip=new ZipArchive(file,ZipArchiveMode.Read))
            {
                var manifest=zip.GetEntry("manifest.xml");
                if(manifest==null||manifest.Length>1024*1024)throw new InvalidDataException("Missing receiver manifest.");
                XDocument xml;using(var input=manifest.Open())xml=XDocument.Load(input);
                var body=xml.Root.Element("KK_UncensorSelector").Element("body");
                var result=new PreparedReceiverPackage{FilePath=Path.GetFullPath(path),Guid=(string)body.Element("guid"),Bundle="abdata/"+(string)body.Element("oo_base").Element("file"),Asset=(string)body.Element("oo_base").Element("asset"),VisualVertices=56130};
                result.Definition=Read<SimulationDefinition>(zip,"bpn/simulation.json");
                result.Physical=Read<SimulationDefinition>(zip,"bpn/reactive/physical.json");
                result.Support=Read<SimulationDefinition>(zip,"bpn/reactive/support.json");
                result.Transfer=Read<ReceiverSupportTransfer>(zip,"bpn/reactive/transfer.json");
                result.ReactiveTransfer=Read<ReceiverSupportTransfer>(zip,"bpn/reactive/reactive-transfer.json");
                result.Contours=Read<ReceiverGuideDefinition>(zip,"bpn/reactive/contours.json");
                result.Profile=Read<AnimatedPathProfile>(zip,"bpn/reactive/path.json");
                result.Chart=Read<ReactiveMaterialChart>(zip,"bpn/reactive/chart.json");
                if(zip.GetEntry("bpn/closed-entrance.json")!=null)result.ClosedEntrance=Read<ClosedEntranceDefinition>(zip,"bpn/closed-entrance.json");
                if(zip.GetEntry("bpn/receiver.json")!=null)
                {
                    var descriptor=Read<Descriptor>(zip,"bpn/receiver.json");
                    if(descriptor.Schema!=1||descriptor.Guid!=result.Guid||descriptor.Bundle!=result.Bundle||descriptor.Asset!=result.Asset||descriptor.VisualMeshSignature!=result.Definition.VisualMeshSignature||descriptor.VisualVertices<3||descriptor.VisualVertices>200000)throw new InvalidDataException("Receiver descriptor/manifest mismatch.");
                    result.VisualVertices=descriptor.VisualVertices;
                }
                else if(result.Guid!="org.betterpenetration.next.body.v1")throw new InvalidDataException("Variant requires a receiver descriptor.");
                if(zip.GetEntry(result.Bundle)==null||result.Definition.VisualMeshSignature!=result.Physical.VisualMeshSignature||result.Definition.VisualMeshSignature!=result.Support.VisualMeshSignature)throw new InvalidDataException("Receiver geometry identities differ.");
                result.Definition.Validate(result.VisualVertices);result.Physical.Validate(result.VisualVertices);result.Support.Validate(result.VisualVertices);result.Profile.Validate(result.Physical);
                if(result.ClosedEntrance!=null)result.ClosedEntrance.Validate(result.Support,result.Physical);
                return result;
            }
        }
    }
}
