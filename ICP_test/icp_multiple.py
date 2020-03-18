import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc



def main():

    ##load cloud point
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()
    result_point = np.array([])
    point_size = 0
    pointCloud = vpc.VtkPointCloud()
    
    cloud_in.from_file(bytes(b'/home/haeyeon/Lidar/data1/1.pcd'))
    for i in range(2,20,1000):

    #cloud_in.from_file(bytes(b'/home/haeyeon/cocel/data1/1.pcd'))
        filename = '/home/haeyeon/cocel/data1/{}.pcd'.format(i)
        cloud_out.from_file(filename.encode())
        icp = cloud_in.make_IterativeClosestPoint()
    ## ICP algorithm ##
        converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
        
        #rotation = transf[0:3,0:3]
        #translation = transf[3,0:3]

        outArray = np.array(np.asarray(cloud_out))
        cloud_in = cloud_out
        for j in range(cloud_in.size):
            if(not outArray[j,0:3] in result_point):
                result_point = np.append(result_point, outArray[j,0:3])
                point_size +=1
    # ## Render with vtk

    for i in range(point_size):
        #print(result_point[i*3:(i+1)*3])
        pointCloud.addPoint(result_point[i*3:(i+1)*3],-1)

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