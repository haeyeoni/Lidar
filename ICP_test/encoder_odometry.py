import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys
import time
import os

file_path = '/home/haeyeon/Lidar/ICP_test/ce30data_1'
file_number = '50'
steps = '1'

def main(file_path, file_number, steps):
    viewer = pcl.pcl_visualization.PCLVisualizering()
    encoder = np.load(file_path + "/encoder.npy")
    yaw = []
    transition = []
    for c,x,y,theta in encoder:
        yaw.append(theta)
        transition.append([x,y])

    num_yaw_data = len(yaw)

    ##load cloud point
    cloud_source = pcl.PointCloud()
    cloud_target = pcl.PointCloud()
    ## calculate the number of lidar data set
    num_lidar_data = 0
    for path in os.listdir(file_path):
        if os.path.isfile(os.path.join(file_path, path)):
            num_lidar_data += 1

    step = num_lidar_data/num_yaw_data

    for i in range(1,num_yaw_data):
        lcount = round(step * i)
        ## read file
        filename = file_path + '/{}.pcd'.format(lcount)
        print(filename)
        cloud_source.from_file(filename.encode())
        ## filtering
        # sor = cloud_source.make_voxel_grid_filter()
        # sor.set_leaf_size(0.05, 0.05, 0.05)
        # cloud_source = sor.filter()
        ## remove ground floor
        #cloud_source = ransac(cloud_source)
        
        ##initialize the target only at the first time
        if cloud_target.size is 0:
            cloud_target = cloud_source
            continue
        ## rotates the source cloud according to the yaw data
        
        angle = np.radians(-yaw[i])
        c, s = np.cos(angle), np.sin(angle)
        R = np.array(((c,-s,0),(s,c,0),(0,0,1)))
        print(i,"th encoder data")
        x,y = transition[i][0], transition[i][1]
        t = np.array((-y/1000,-x/1000,0))
        # #render(cloud_source)
        # ## running ICP algorithm
            
        cloud_source_matrix = cloud_source.to_array()
        cloud_source_rotated_matrix = np.add(np.dot(cloud_source_matrix,R),t)
            
            #cloud_source_rotated_matrix = np.dot(cloud_source_matrix,R)
        cloud_source_rotated = pcl.PointCloud(cloud_source_rotated_matrix.astype(np.float32))
        # cloud_name = "cloud" + str(i)
        # viewer.AddPointCloud(cloud_source_rotated,cloud_name.encode())
        # viewer.SpinOnce()

        #render(cloud_source_rotated)
        icp = cloud_source_rotated.make_IterativeClosestPoint()
        converged, transf, estimate, fitness = icp.icp(cloud_source_rotated, cloud_target, max_iter = 100, axis=1)
        
        
        ## Transfrom the input poistart_time = time.time()
            # rotation = transf[0:3,0:3]
            # translation = transf[3,0:3]  
        #converted = np.add(np.matmul(estimate, rotation),translation)
        if(converged):
            cloud_name = "cloud" + str(i)
            viewer.AddPointCloud(estimate,cloud_name.encode())
            cloud_target = estimate
            viewer.SpinOnce()
        
    viewer.Spin()

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