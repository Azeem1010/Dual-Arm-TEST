$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$interop = 'E:/Program Files/SOLIDWORKS Crop/SOLIDWORKS/api/redist/SolidWorks.Interop.sldworks.dll'
Add-Type -Path $interop
Add-Type -ReferencedAssemblies $interop -TypeDefinition @'
using System;
using System.IO;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
public static class CorrectedStepImport {
    public static string Run(string input, string outputDirectory) {
        ISldWorks sw;
        try { sw=(ISldWorks)Marshal.GetActiveObject("SldWorks.Application"); }
        catch(COMException) { sw=(ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application")); }
        int error=0, warning=0;
        IImportStepData data=(IImportStepData)sw.GetImportFileData(input);
        if(data!=null) data.MapConfigurationData=false;
        IModelDoc2 doc=(IModelDoc2)sw.LoadFile4(input,"r",data,ref error);
        if(doc==null) throw new Exception("STEP import failed: "+error);
        Directory.CreateDirectory(outputDirectory);
        string output=Path.Combine(outputDirectory,"dual_arm_corrected_geometry"+(doc.GetType()==2 ? ".SLDASM" : ".SLDPRT"));
        if(File.Exists(output)) throw new Exception("Refusing to overwrite existing native output: "+output);
        bool saved=((IModelDocExtension)doc.Extension).SaveAs(output,0,1,null,ref error,ref warning);
        if(!saved || error!=0) throw new Exception("Native save failed: "+error+" warnings="+warning);
        return "PASS imported geometry only; original feature tree/mates not preserved. Output="+output+" warnings="+warning;
        // Never close or hide the application or user documents.
    }
}
'@
[CorrectedStepImport]::Run((Join-Path $root 'cad/dual_arm_corrected.STEP'), (Join-Path $root 'cad/native_corrected'))
