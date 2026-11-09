$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
Add-Type -Path 'E:/Program Files/SOLIDWORKS Crop/SOLIDWORKS/api/redist/SolidWorks.Interop.sldworks.dll'
Add-Type -ReferencedAssemblies 'E:/Program Files/SOLIDWORKS Crop/SOLIDWORKS/api/redist/SolidWorks.Interop.sldworks.dll' -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
public class CadComponentRow {
    public string name;
    public string path;
    public bool fixedComponent;
    public bool virtualComponent;
    public double[] transform;
}
public static class CadInspect {
    public static CadComponentRow[] Run(string path) {
        ISldWorks sw;
        try { sw = (ISldWorks)Marshal.GetActiveObject("SldWorks.Application"); }
        catch (COMException) { sw = (ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application")); }
        int errors=0, warnings=0;
        IModelDoc2 doc = (IModelDoc2)sw.OpenDoc6(path,2,1,"",ref errors,ref warnings);
        if (doc == null) throw new Exception("Open failed: "+errors+" / "+warnings);
        var rows = new List<CadComponentRow>();
        foreach (object item in (object[])((IAssemblyDoc)doc).GetComponents(false)) {
            IComponent2 c = (IComponent2)item;
            rows.Add(new CadComponentRow {name=c.Name2,path=c.GetPathName(),fixedComponent=c.IsFixed(),virtualComponent=c.IsVirtual,
                transform=(double[])((IMathTransform)c.Transform2).ArrayData});
        }
        // Leave application and documents open; never close user-owned work.
        return rows.ToArray();
    }
}
'@
$path = Join-Path $root 'cad/working/完成_雙臂總組合.sldasm'
if ($env:DUAL_ARM_CAD_INSPECT) { $path = [System.IO.Path]::GetFullPath($env:DUAL_ARM_CAD_INSPECT) }
$items = [CadInspect]::Run($path)
$report = if ($env:DUAL_ARM_CAD_INSPECT) { 'solidworks_corrected_components.json' } else { 'solidworks_components.json' }
$items | ConvertTo-Json -Depth 5 | Set-Content (Join-Path $root ('docs/evidence/' + $report)) -Encoding utf8
"components=$($items.Count)"
