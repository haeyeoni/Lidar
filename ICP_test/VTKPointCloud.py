import vtk
import numpy as np

class VtkPointCloud:
    def __init__(self, zMin=-1.0, zMax=1.0, maxNumPoints=1e6):
        self.init_planes()
        self.init_points(zMin, zMax, maxNumPoints)

    def addPoint(self, point, color_num):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
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