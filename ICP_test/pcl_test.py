import pcl
import vtk
import VTKPointCloud as vpc

p = pcl.PointCloud()
p.from_file(bytes(b'/home/haeyeon/cocel/data1/1.pcd'))
p.to_list()
print(p.size)

pointCloud = vpc.VtkPointCloud()
for i in range(p.size):
    pointCloud.addPoint(p[i],-1)

renderer = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)

        # Interactor
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)
renderer.AddActor(pointCloud.point_vtkActor)
renderer.SetBackground(0.0, 0.0, 0.0)
renderWindow.Render()
renderWindowInteractor.Initialize()

renderWindowInteractor.Start()

