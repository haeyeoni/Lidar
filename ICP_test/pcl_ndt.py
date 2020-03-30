# -*- coding: utf-8 -*-
# How to use Normal Distributions Transform
# http://pointclouds.org/documentation/tutorials/normal_distributions_transform.php#normal-distributions-transform

import pcl
import pcl.pcl_visualization
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys

# int main (int argc, char** argv)


def main():
    target_cloud = pcl.load('/home/haeyeon/Lidar/RIST_data/1.pcd')
    print('Loaded ' + str(target_cloud.size) +
          ' data points from room_scan1.pcd')


    input_cloud = pcl.load('/home/haeyeon/Lidar/RIST_data/500.pcd')
    print('Loaded ' + str(input_cloud.size) +
          ' data points from room_scan.pcd')


    approximate_voxel_filter = input_cloud.make_ApproximateVoxelGrid()
    approximate_voxel_filter.set_leaf_size(0.2, 0.2, 0.2)
    filtered_cloud = approximate_voxel_filter.filter()

    ndt = pcl.NormalDistributionsTransform(input_cloud,target_cloud)
    ndt.convertTransform()
    ndt.
    print('Filtered cloud contains ' + str(filtered_cloud.size) +
          ' data points from room_scan2.pcd')
          
    print(filtered_cloud.size, input_cloud.size, target_cloud.size)
    render(target_cloud, input_cloud, filtered_cloud)

def render(cloud1, cloud2, cloud3):
    pointCloud = vpc.VtkPointCloud()
    
    # for i in range(cloud1.size):
    #     pointCloud.addPoint(cloud1[i],-1)
    # for i in range(cloud2.size):
    #     pointCloud.addPoint(cloud2[i],0)
    for i in range(cloud3.size):
        pointCloud.addPoint(cloud3[i],1)
    
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
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()