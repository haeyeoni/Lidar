import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys
import time

file_path = '/home/haeyeon/Lidar/ICP_test/data3'
file_number = '385'
steps = '1'

def main(file_path, file_number, steps):
    start_time = time.time()
    ##load cloud point
    cloud_source = pcl.PointCloud()
    cloud_target = pcl.PointCloud()
    result = []
    ## Iteratively load the pcd data 
    for i in range(1,int(file_number),int(steps)):
        filename = file_path + '/{}.pcd'.format(i)
        print(filename)
        cloud_source.from_file(filename.encode())
        
        sor = cloud_source.make_voxel_grid_filter()
        sor.set_leaf_size(0.05, 0.05, 0.05)
        cloud_source = sor.filter()
        cloud_source = ransac(cloud_source)
        if cloud_target.size is 0:
            cloud_target = cloud_source
            cloud_target = ransac(cloud_target)
            for j in range(cloud_target.size):
                result.append(cloud_target[j])
                continue
        icp = cloud_source.make_IterativeClosestPoint()
        ## ICP algorithm ##
        converged, transf, estimate, fitness = icp.icp(cloud_source, cloud_target)
        ## Transfrom the input point cloud
        #rotation = transf[0:3,0:3]
        #translation = transf[3,0:3]  
        #converted = np.add(np.matmul(estimate, rotation),translation)
        if(converged):
            for j in range(estimate.size):
                result.append(estimate[j])
            
    cloud_final = pcl.PointCloud(result)
    print("---{}s seconds---".format(time.time()-start_time))
    render(cloud_final)


def render(cloud):
    pointCloud = vpc.VtkPointCloud()

    for i in range(cloud.size):
        pointCloud.addPoint(cloud[i],0)
    
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



def ransac(cloud):
    model_p = pcl.SampleConsensusModelPlane(cloud)
    ransac = pcl.RandomSampleConsensus(model_p)
    ransac.set_DistanceThreshold(0.1)
    ransac.computeModel()
    plane = ransac.get_Inliers()
    
    final = []
    print(len(plane))

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