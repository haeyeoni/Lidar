import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
import sys

file_path = '/home/haeyeon/Lidar/RIST_data/'
file_number = '1249'
steps = '50'

#def main(file_path, file_number, steps):
def main():
    ##load cloud point
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()
    result = []
    filename = file_path + '/1.pcd'
    cloud_in.from_file(filename.encode())
    cloud_in = ransac(cloud_in)
    # for i in range(cloud_in.size):
    #     pointCloud.addPoint(cloud_in[i],1)
    
    ## Iteratively load the pcd data 
    for i in range(2,int(file_number),int(steps)):
        filename = file_path + '{}.pcd'.format(i)
        cloud_out.from_file(filename.encode())
        print(filename)
        
        cloud_out = ransac(cloud_out)
        icp = cloud_in.make_IterativeClosestPoint()
        ## ICP algorithm ##
        converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
        ## Transfrom the input point cloud
        rotation = transf[0:3,0:3]
        translation = transf[3,0:3]  
        converted = np.add(np.matmul(estimate, rotation),translation)
        if(converged):
            for j in range(converted.shape[0]):
                new_points = converted[j,0:3]
                result.append(new_points)
        cloud_in.from_array(converted)
    cloud_out.from_list(result)
    #ransac_render(cloud_out)
    render(cloud_out)

def render(cloud):
    pointCloud = vpc.VtkPointCloud()
    
    for i in range(cloud.size):
        pointCloud.addPoint(cloud[i],-1)

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
    cnt = 0
    for i in range(0, cloud.size):
        plane_index = plane[find_plane]
        if plane_index == i:
            if find_plane < len(plane) - 1:
                find_plane += 1
        else:
            final.append(cloud[i])
            cnt += 1
    print(cnt)   
    print(cloud.size)   

    return pcl.PointCloud(final)

def ransac_render(cloud):

    pointCloud = vpc.VtkPointCloud()
    model_p = pcl.SampleConsensusModelPlane(cloud)
    ransac = pcl.RandomSampleConsensus(model_p)
    ransac.set_DistanceThreshold(1)
    ransac.computeModel()
    plane = ransac.get_Inliers()
    print(len(plane))

    find_plane = 0 ## index of inlier inside cloud
    cnt = 0
    for i in range(0, cloud.size):
        plane_index = plane[find_plane]
        if plane_index == i:
            if find_plane < len(plane) - 1:
                find_plane += 1
        else:
            pointCloud.addPoint(cloud[i],-1)
            cnt += 1
    print(cnt)   
    print(cloud.size)     
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
    #assert len(sys.argv) == 4, "Command Format: python3 icp_multiple.py {filepath} {file_number} {steps}"
    #main(sys.argv[1], sys.argv[2], sys.argv[3])
    main()