import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys

def main(file_path, file_number, steps):
    ##load cloud point
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()
    filename = file_path + '/1.pcd'
    cloud_in.from_file(filename.encode())
    pointCloud = vpc.VtkPointCloud()
    for i in range(cloud_in.size):
        pointCloud.addPoint(cloud_in[i],1)
    
    ## Iteratively load the pcd data 
    for i in range(2,int(file_number),int(steps)):
        filename = file_path + '/{}.pcd'.format(i)
        print(filename)
        cloud_out.from_file(filename.encode())
        icp = cloud_in.make_IterativeClosestPoint()
        ## ICP algorithm ##
        converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
        ## Transfrom the input point cloud
        rotation = transf[0:3,0:3]
        translation = transf[3,0:3]  
        converted = np.add(np.matmul(estimate, rotation),translation)
        if(converged):
            for j in range(estimate.size):
                new_points = converted[j,0:3]
                pointCloud.addPoint(new_points,0)


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
    assert len(sys.argv) == 4, "Command Format: python3 icp_multiple.py {filepath} {file_number} {steps}"
    main(sys.argv[1], sys.argv[2], sys.argv[3])