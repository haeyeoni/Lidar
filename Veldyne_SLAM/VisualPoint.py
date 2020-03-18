import vtk
import numpy as np
import GetData
import time


class VisualPoint():
    def __init__(self):
        self.point_cloud = VtkPointCloud()
        self.POINT = []

    def render(self):
        x_thresh = [-1.1, 2.1]
        y_thresh = [1.5, 4.5]
        z_thresh = [-0.4, 2.1]
        for i in range(len(self.POINT)):
            point_coords = self.POINT[i]
            if (point_coords[0] > x_thresh[0]) and (point_coords[0] < x_thresh[1]) and \
                    (point_coords[1] > y_thresh[0]) and (point_coords[1] < y_thresh[1]) and \
                    (point_coords[2] > z_thresh[0]) and (point_coords[2] < z_thresh[1]):
                color_num = 0.7
            else:
                color_num = -1
            self.point_cloud.addPoint(self.POINT[i], color_num)
        # Add the velodyne plane# Add the velodyne plane
        for x in np.linspace(-4, 4, 100):
            for y in np.linspace(0, 2, 25):
                tmp_coords = np.array([x, 0, y])
                self.point_cloud.addPoint(tmp_coords, 1)
        # Add the floor plane
        plane_center = (-4, -4, -0.55)
        normal = (0, 0, 1)
        point1 = ([-4, 10, -0.55])
        point2 = ([4, -4, -0.55])
        self.point_cloud.addPlane(plane_center, normal, point1, point2)
        # Renderer
        renderer = vtk.vtkRenderer()
        # Render Window
        
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)

        # Interactor
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        renderer.AddActor(self.point_cloud.point_vtkActor)
        renderer.AddActor(self.point_cloud.plane_vtkActor)
        renderer.SetBackground(0.0, 0.0, 0.0)
        renderWindow.Render()
        renderWindowInteractor.Initialize()

        #callback 
        #cb = vtkTimerCallback(duration=250, VtkPointCloud=self.point_cloud)
        renderWindowInteractor.AddObserver('TimerEvent', self.callback(renderWindowInteractor))
        renderWindowInteractor.CreateTimer(0)
        renderWindowInteractor.Start()

    def callback(self, obj):
        iren = obj
        while True:
            try:
                print(len(self.POINT))
                for p in self.POINT:
                    self.point_cloud.addPoint(p,1)
                iren.CreateTimer(1)
                iren.GetRenderWindow().Render()
            except KeyboardInterrupt:
                    iren.DestroyTimer()

class VtkPointCloud:
    def __init__(self, zMin=-1.0, zMax=1.0, maxNumPoints=1e6):
        self.init_planes()
        self.init_points(zMin, zMax, maxNumPoints)

    def addPoint(self, point, color_num):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:3])
            self.vtkDepth.InsertNextValue(color_num)
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = np.random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:3])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def addPlane(self, plane_center, normal, x_axis, y_axis):
        self.vtkPlanes.SetCenter(plane_center)
        self.vtkPlanes.SetNormal(normal)
        self.vtkPlanes.SetPoint1(x_axis)
        self.vtkPlanes.SetPoint2(y_axis)

    def init_points(self, zMin=-1.0, zMax=1.0, maxNumPoints=1e6):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()

        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetCellData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetCellData().SetActiveScalars('DepthArray')
        point_mapper = vtk.vtkPolyDataMapper()
        point_mapper.SetInputData(self.vtkPolyData)
        point_mapper.SetColorModeToDefault()
        point_mapper.SetScalarRange(zMin, zMax)
        self.point_vtkActor = vtk.vtkActor()
        self.point_vtkActor.SetMapper(point_mapper)

    def init_planes(self):
        self.vtkPlanes = vtk.vtkPlaneSource()
        plane_mapper = vtk.vtkPolyDataMapper()
        plane_mapper.SetInputData(self.vtkPlanes.GetOutput())
        self.plane_vtkActor = vtk.vtkActor()
        self.plane_vtkActor.SetMapper(plane_mapper)
