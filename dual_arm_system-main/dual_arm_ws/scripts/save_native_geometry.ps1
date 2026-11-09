$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$interop='E:/Program Files/SOLIDWORKS Crop/SOLIDWORKS/api/redist/SolidWorks.Interop.sldworks.dll'
Add-Type -Path $interop
Add-Type -ReferencedAssemblies $interop -TypeDefinition @'
using System;
using System.IO;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
public static class StandaloneCad {
 public static string Run(string input,string output) {
  ISldWorks sw;
  try { sw=(ISldWorks)Marshal.GetActiveObject("SldWorks.Application"); }
  catch(COMException) { sw=(ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application")); }
  int error=0,warning=0;
  IModelDoc2 doc=(IModelDoc2)sw.OpenDoc6(input,2,1,"",ref error,ref warning);
  if(doc==null) throw new Exception("Open failed "+error);
  if(File.Exists(output)) throw new Exception("Output already exists");
  if(!((IModelDocExtension)doc.Extension).SaveAs(output,0,1,null,ref error,ref warning))
   throw new Exception("Save part failed "+error);
  IModelDoc2 part=(IModelDoc2)sw.OpenDoc6(output,1,1,"",ref error,ref warning);
  if(part==null) throw new Exception("Reopen failed "+error);
  object[] bodies=(object[])((IPartDoc)part).GetBodies2(0,false);
  return "Saved/reopened native geometry part; solid bodies="+(bodies==null?0:bodies.Length)+" warnings="+warning;
 }
}
'@
[StandaloneCad]::Run((Join-Path $root 'cad/native_corrected/dual_arm_corrected_geometry.SLDASM'),(Join-Path $root 'cad/dual_arm_corrected_geometry.SLDPRT'))
