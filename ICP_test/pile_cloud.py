import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys
import time
import os

file_path = '/home/haeyeon/Downloads/data0407_1size'
file_number = '50'
steps = '1'

def main(file_path, file_number, steps):
    viewer = pcl.pcl_visualization.PCLVisualizering()
    ##load cloud point
    cloud = pcl.PointCloud()
    ## Iteratively load the pcd data
    num_lidar_data = 0
    for path in os.listdir(file_path):
        if os.path.isfile(os.path.join(file_path, path)):
            num_lidar_data += 1
    result = []
    for i in range(1,num_lidar_data,int(steps)):
        ## read file
        filename = file_path + '/{}.pcd'.format(i)
        print(filename)
        cloud.from_file(filename.encode())
        for j in range(cloud.size):
            result.append(cloud[j])
    cloud_final = pcl.PointCloud(result)
    cloud_final = ransac(cloud_final)
    viewer.AddPointCloud(cloud_final)
    viewer.Spin()
    # start_time = time.time()
    
def render(cloud):
    pointCloud = vpc.VtkPointCloud()

    for i in range(cloud.size):
        pointCloud.addPoint(cloud[i],0)
    
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    renderer.AddActorstart_time = time.time()
    ()
    renderWindowInteractor.Initialize()
    renderWindowInteractor.Start()



def ransac(cloud):
    model_p = pcl.SampleConsensusModelPlane(cloud)
    ransac = pcl.RandomSampleConsensus(model_p)
    ransac.set_DistanceThreshold(0.1)
    ransac.computeModel()
    plane = ransac.get_Inliers()
    
    final = []
    find_plane = 0 ## index of inlier inside cloud
    for i in range(0, cloud.size):
        plane_index = plane[find_plane]
        if plane_index == i:
            if find_plane < len(plane) - 1:
                find_plane += 1
        else:
            final.append(cloud[i])
    
    return pcl.PointCloud(final)

if __name__ == "__main__":
    #assert len(sys.argv) == 4, "Command Format: python3 icp_multiple.py {filepath} {file_number} {steps}"
    #main(sys.argv[1], sys.argv[2], sys.argv[3])
    main(file_path, file_number, steps)