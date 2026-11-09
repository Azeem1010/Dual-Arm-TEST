"""Engineering rendering of corrected CAD meshes; no image synthesis/editing."""
from pathlib import Path
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer, vtkRenderWindow, vtkWindowToImageFilter
from vtkmodules.vtkIOImage import vtkPNGWriter
import vtkmodules.vtkRenderingOpenGL2

root=Path(__file__).resolve().parents[1]
renderer=vtkRenderer()
renderer.SetBackground(.93,.94,.96)
for index in [8,9,10,11,12,19]:
    reader=vtkSTLReader()
    reader.SetFileName(str(root/f'src/dual_arm_description/meshes/cad/component_{index:03d}.stl'))
    reader.Update()
    mapper=vtkPolyDataMapper()
    mapper.SetInputData(reader.GetOutput())
    actor=vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*((.85,.38,.55) if index in [10,11] else (.08,.08,.09) if index==12 else (.65,.68,.72)))
    renderer.AddActor(actor)
camera=renderer.GetActiveCamera()
camera.SetPosition(400,185,-50)
camera.SetFocalPoint(100,185,-50)
camera.SetViewUp(0,0,1)
camera.ParallelProjectionOn()
camera.SetParallelScale(85)
window=vtkRenderWindow()
window.SetOffScreenRendering(1)
window.SetSize(1000,800)
window.AddRenderer(renderer)
window.Render()
image=vtkWindowToImageFilter()
image.SetInput(window)
image.Update()
writer=vtkPNGWriter()
writer.SetFileName(str(root/'docs/evidence/camera_mount_corrected_side.png'))
writer.SetInputConnection(image.GetOutputPort())
writer.Write()
