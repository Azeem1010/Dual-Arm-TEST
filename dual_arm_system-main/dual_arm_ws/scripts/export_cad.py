"""Extract STEP leaf geometry and placements with OpenCascade, without inventing joints.

Run using .cad-venv/bin/python after installing cadquery-ocp.
STL files are in millimetres; consumers must use scale 0.001.
"""
import json
import math
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_WIRE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.XCAFDoc import XCAFDoc_ColorGen

root = Path(__file__).resolve().parents[1]
step = next((root/'third_party/dual_arm_3d').rglob('雙臂總組合.STEP'))
out = root/'src/dual_arm_description/meshes/cad'
out.mkdir(parents=True,exist_ok=True)
doc = TDocStd_Document(TCollection_ExtendedString('dual_arm'))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
if reader.ReadFile(str(step)) != IFSelect_RetDone or not reader.Transfer(doc):
    raise RuntimeError('STEP transfer failed')
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
leaves = []
corrections = json.loads((root/'cad/corrections.json').read_text(encoding='utf-8-sig'))
corrected_doc = TDocStd_Document(TCollection_ExtendedString('corrected_dual_arm'))
corrected_tool = XCAFDoc_DocumentTool.ShapeTool_s(corrected_doc.Main())
corrected_colors = XCAFDoc_DocumentTool.ColorTool_s(corrected_doc.Main())

def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else 'unnamed'

def visit(label, placement, ancestry):
    label_name = name(label)
    if tool.IsReference_s(label):
        target = TDF_Label()
        tool.GetReferredShape_s(label,target)
        visit(target, placement.Multiplied(tool.GetLocation_s(label)), ancestry+[label_name])
        return
    if tool.IsAssembly_s(label):
        children = TDF_LabelSequence()
        tool.GetComponents_s(label,children)
        for i in range(1,children.Length()+1):
            visit(children.Value(i),placement,ancestry+[label_name])
        return
    shape = tool.GetShape_s(label)
    if shape.IsNull():
        return
    for correction in corrections['rotations']:
        if len(leaves) in correction['component_indices']:
            rotation = gp_Trsf()
            rotation.SetRotation(gp_Ax1(gp_Pnt(*correction['pivot_cad_mm']),gp_Dir(*correction['axis_cad'])),
                                 math.radians(correction['angle_deg']))
            placement = TopLoc_Location(rotation).Multiplied(placement)
    transformed = shape.Moved(placement)
    corrected_label = corrected_tool.AddShape(transformed,False)
    TDataStd_Name.Set_s(corrected_label,TCollection_ExtendedString(f'component_{len(leaves):03d}_{label_name}'))
    BRepMesh_IncrementalMesh(transformed,0.15,False,0.2,True).Perform()
    filename = f'component_{len(leaves):03d}.stl'
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(transformed,str(out/filename)):
        raise RuntimeError('STL export failed')
    box = Bnd_Box()
    BRepBndLib.Add_s(transformed,box)
    tr = placement.Transformation()
    low, high = box.CornerMin(), box.CornerMax()
    cylinders = []
    apertures = []
    wire_bounds = []
    if '平台' in label_name:
        explorer = TopExp_Explorer(transformed,TopAbs_WIRE)
        while explorer.More():
            wb = Bnd_Box()
            BRepBndLib.Add_s(explorer.Current(),wb)
            wl, wh = wb.CornerMin(), wb.CornerMax()
            dx, dy, dz = wh.X()-wl.X(), wh.Y()-wl.Y(), wh.Z()-wl.Z()
            wire_bounds.append([wl.X(),wl.Y(),wl.Z(),wh.X(),wh.Y(),wh.Z()])
            if 10 < dx < 80 and dy < .5 and 10 < dz < 80:
                apertures.append(dict(center_mm=[(wh.X()+wl.X())/2,(wh.Y()+wl.Y())/2,(wh.Z()+wl.Z())/2],size_mm=[dx,dy,dz]))
            explorer.Next()
    if 'ax12' in label_name.lower() or 'ax-12' in label_name.lower():
        explorer = TopExp_Explorer(transformed,TopAbs_FACE)
        while explorer.More():
            surface = BRepAdaptor_Surface(TopoDS.Face(explorer.Current()))
            if surface.GetType()==GeomAbs_Cylinder:
                cylinder = surface.Cylinder()
                if cylinder.Radius()>=5:
                    axis = cylinder.Axis()
                    point, direction = axis.Location(), axis.Direction()
                    cylinders.append(dict(radius_mm=cylinder.Radius(),point_mm=[point.X(),point.Y(),point.Z()],axis=[direction.X(),direction.Y(),direction.Z()]))
            explorer.Next()
    leaves.append(dict(file=filename,name=label_name,ancestry=ancestry,
                       wire_bounds=wire_bounds, apertures=apertures, cylinders=cylinders, bounds_mm=[low.X(),low.Y(),low.Z(),high.X(),high.Y(),high.Z()], transform=[[tr.Value(i,j) for j in range(1,5)] for i in range(1,4)]))

free = TDF_LabelSequence()
tool.GetFreeShapes(free)
for i in range(1,free.Length()+1):
    visit(free.Value(i),TopLoc_Location(),[])
(root/'docs/evidence/cad_inventory.json').write_text(json.dumps(leaves,indent=2,ensure_ascii=False)+'\n')
# Eight 25 mm white pads fill the original recesses to world z=14 mm.
# CAD -> world: x=x/1000, y=.03-z/1000, z=y/1000.
for row in range(4):
    for col in range(2):
        x=-167.5+col*35
        y=-15+row*35
        pad=BRepPrimAPI_MakeBox(gp_Pnt(x-12.5,0,30-y-12.5),25,14,25).Shape()
        label=corrected_tool.AddShape(pad,False)
        TDataStd_Name.Set_s(label,TCollection_ExtendedString(f'C_white_pad_{row*2+col+1}'))
        corrected_colors.SetColor(label,Quantity_Color(1.,1.,1.,Quantity_TOC_RGB),XCAFDoc_ColorGen)
writer=STEPCAFControl_Writer()
writer.SetNameMode(True)
writer.SetColorMode(True)
if not writer.Transfer(corrected_doc,STEPControl_AsIs):
    raise RuntimeError('Corrected STEP transfer failed')
if writer.Write(str(root/'cad/dual_arm_corrected.STEP')) != IFSelect_RetDone:
    raise RuntimeError('Corrected STEP write failed')
print(f'Exported {len(leaves)} CAD components. Joint identification and dynamic validation remain required.')
