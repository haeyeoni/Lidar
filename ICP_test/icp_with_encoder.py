import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys
import time
import os

file_path = '/home/haeyeon/Downloads/driving_velodyne'
ce30_path = '/home/haeyeon/Downloads/driving_ce30'
file_number = '50'
steps = '1'

def main(file_path, file_number, steps):
    viewer = pcl.pcl_visualization.PCLVisualizering()
    # start_time = time.time()
    # encoder = np.load(file_path + "/encoder.npy")
    # yaw = []
    # transition = []
    # for c,x,y,theta in encoder:
    #     yaw.append(theta)
    #     transition.append([x,y])   
    # num_yaw_data = len(yaw)

    ##load cloud point
    cloud_source = pcl.PointCloud()
    cloud_target = pcl.PointCloud()
    result = []
    ## Iteratively load the pcd data
    num_lidar_data = 0
    for path in os.listdir(file_path):
        if os.path.isfile(os.path.join(file_path, path)):
            num_lidar_data += 1
    # step = num_yaw_data / num_lidar_data

    for i in range(1,num_lidar_data,int(steps)):
        ## read file
        filename = file_path + '/{}.pcd'.format(i)
        print(filename)
        cloud_source.from_file(filename.encode())
        ## filtering
        sor = cloud_source.make_voxel_grid_filter()
        sor.set_leaf_size(0.05, 0.05, 0.05)
        cloud_source = sor.filter()
        ## remove ground floor
        cloud_source = ransac(cloud_source)
        
        ##initialize the target only at the first time
        if cloud_target.size is 0:
            cloud_target = cloud_source
            for j in range(cloud_target.size):
                result.append(cloud_target[j])
                continue
        ## rotates the source cloud according to the yaw data
        # step = num_yaw_data/num_lidar_data
        # if round(step*i) < len(yaw):
        #     angle = np.radians(-yaw[round(step*i)])
        #     c, s = np.cos(angle), np.sin(angle)
        #     R = np.array(((c,-s,0),(s,c,0),(0,0,1)))
        #     print(round(step*i))
        #     x,y = transition[round(step*i)][0], transition[round(step*i)][1]
        #     t = np.array((-x/1000,-y/1000,0))
        # #render(cloud_source)
        # ## running ICP algorithm
            
        # cloud_source_matrix = cloud_source.to_array()
        # cloud_source_rotated_matrix = np.add(np.dot(cloud_source_matrix,R),t)
        
        # #cloud_source_rotated_matrix = np.dot(cloud_source_matrix,R)
        # cloud_source_rotated = pcl.PointCloud(cloud_source_rotated_matrix.astype(np.float32))
        
    #render(cloud_source_rotated)
        icp = cloud_source.make_IterativeClosestPoint()
        converged, transf, estimate, fitness = icp.icp(cloud_source, cloud_target, max_iter = 100)
            
        ## Transfrom the input poistart_time = time.time()
            # rotation = transf[0:3,0:3]
            # translation = transf[3,0:3]  
        #converted = np.add(np.matmul(estimate, rotation),translation)
        if(converged):
            cloud_name = "cloud" + str(i)
            viewer.AddPointCloud(estimate,cloud_name.encode())
            viewer.SpinOnce()
                # # cloud_target_matrix = np.add(np.dot(cloud_source_matrix,rotation),translation)
                # # cloud_target = pcl.PointCloud(cloud_target_matrix.astype(np.float32))
                # # cloud_target = estimate
                # viewer.AddPointCloud(estimate,cloud_name.encode())
                # viewer.SpinOnce()
                # for j in range(estimate.size):
                #     result.append(estimate[j])
            
            
    # cloud_final = pcl.PointCloud(result)
    # print("---{}s seconds---".format(time.time()-start_time))
    # print("pcd file",num_lidar_data,"encoder data",num_yaw_data)
    # render(cloud_final)
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