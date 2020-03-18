import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc



def main():

    ##load cloud point
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()
    cloud_in.from_file(bytes(b'/home/haeyeon/cocel/data1/1.pcd'))
    cloud_out.from_file(bytes(b'/home/haeyeon/cocel/data1/2.pcd'))

    ## ICP algorithm ##
    icp = cloud_in.make_IterativeClosestPoint()
    converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
    rotation = transf[0:3,0:3]
    translation = transf[3,0:3]
    inArray = np.array(np.asarray(cloud_in))
    outArray = np.array(np.asarray(cloud_out))

    ## Transfrom the input point cloud
    converted = np.add(inArray.dot(rotation),translation)
    print(str(transf))

    ## Render with vtk

    pointCloud = vpc.VtkPointCloud()
    innumPoints = cloud_in.size
    outnumPoints = cloud_out.size
    for i in range(innumPoints):
        pointCloud.addPoint(converted[i,0:3],-1)
    for i in range(innumPoints):
        pointCloud.addPoint(cloud_in[i],0)
    for i in range(outnumPoints):
        pointCloud.addPoint(cloud_out[i],1)

    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)

    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    renderer.AddActor(pointCloud.point_vtkActor)
    renderer.SetBackground(0.0, 0.0, 0.0)
    renderWindow.Render()
    renderWindowInteractor.Initialize()

    renderWindowInteractor.Start()




if __name__ == "__main__":
    main()