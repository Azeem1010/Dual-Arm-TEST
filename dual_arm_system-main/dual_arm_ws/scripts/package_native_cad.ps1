$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$interop='E:/Program Files/SOLIDWORKS Crop/SOLIDWORKS/api/redist/SolidWorks.Interop.sldworks.dll'
Add-Type -Path $interop
Add-Type -ReferencedAssemblies $interop -TypeDefinition @'
using System;
using System.IO;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
public static class NativeCadPack {
 public static string Run(string input,string folder) {
  ISldWorks sw;
  try { sw=(ISldWorks)Marshal.GetActiveObject("SldWorks.Application"); }
  catch(COMException) { sw=(ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application")); }
  int error=0,warning=0;
  IModelDoc2 doc=(IModelDoc2)sw.OpenDoc6(input,2,1,"",ref error,ref warning);
  if(doc==null) throw new Exception("Open failed "+error);
  if(Directory.Exists(folder) && Directory.GetFiles(folder,"*.SLDASM").Length!=0) throw new Exception("Output folder already contains CAD output");
  Directory.CreateDirectory(folder);
  IModelDocExtension extension=(IModelDocExtension)doc.Extension;
  IPackAndGo pack=(IPackAndGo)extension.GetPackAndGo();
  pack.IncludeDrawings=false;
  object raw;
  if(!pack.GetDocumentNames(out raw)) throw new Exception("Cannot enumerate references");
  string[] names=(string[])raw;
  File.WriteAllLines(Path.Combine(folder,"source_references.txt"),names);
  string[] destinations=new string[names.Length];
  for(int i=0;i<names.Length;i++) {
   // 3D Interconnect reference identifiers append |source.step to the document path.
   // Keep the original identifier in PackAndGo's input order; only the output
   // filename uses the document path before the source separator.
   string clean=names[i].TrimEnd('\0').Split('|')[0];
   string filename=String.Equals(clean,input,StringComparison.OrdinalIgnoreCase)?"dual_arm_corrected.SLDASM":("ref_"+i.ToString("D4")+"_"+Path.GetFileName(clean));
   destinations[i]=Path.Combine(folder,filename);
  }
  pack.FlattenToSingleFolder=false;
  if(!pack.SetSaveToName(true,folder)) throw new Exception("Cannot set portable output folder");
  object saved=extension.SavePackAndGo((PackAndGo)pack);
  if(saved==null) throw new Exception("Pack returned no status");
  int[] statuses=(int[])saved;
  foreach(int status in statuses) if(status!=0) throw new Exception("Pack status "+status);
  File.WriteAllLines(Path.Combine(folder,"FILES.txt"),destinations);
  return "PASS files="+destinations.Length;
 }
}
'@
[NativeCadPack]::Run((Join-Path $root 'cad/native_corrected/dual_arm_corrected_geometry.SLDASM'),(Join-Path $root 'cad/native_portable'))
