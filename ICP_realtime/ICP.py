import pcl
import random
import numpy as np
import vtk

class ICP():
    def __init__(self):
        self.previous = []

    def icp_algorithm(self, point_cloud):
        if len(self.previous) is 0:
            return point_cloud
        else:
            cloud_in = pcl.PointCloud()
            cloud_out = pcl.PointCloud()
            cloud_in.from_list(self.previous)
            cloud_out.from_list(point_cloud)
            icp = cloud_in.make_IterativeClosestPoint()

            ## ICP algorithm ##
            converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
            rotation = transf[0:3,0:3]
            translation = transf[3,0:3] 
            transformed = np.add(np.matmul(estimate, rotation),translation)
            self.previous = transformed
            return transformed

# def main():
#     ##load cloud point
#     cloud_in = pcl.PointCloud()
#     cloud_out = pcl.PointCloud()
#     cloud_in.from_file(bytes(b'/home/haeyeon/Lidar/data1/1.pcd'))
#     pointCloud = vpc.VtkPointCloud()
#     for i in range(cloud_in.size):
#         pointCloud.addPoint(cloud_in[i],1)
    
#     ## Iteratively load the pcd data 
#     for i in range(2,1200,100):
#         filename = '/home/haeyeon/cocel/data1/{}.pcd'.format(i)
#         print(filename)
#         cloud_out.from_file(filename.encode())
#         icp = cloud_in.make_IterativeClosestPoint()
#         ## ICP algorithm ##
#         converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
#         ## Transfrom the input point cloud
#         rotation = transf[0:3,0:3]
#         translation = transf[3,0:3]  
#         converted = np.add(np.matmul(estimate, rotation),translation)
#         if(converged):
#             for j in range(estimate.size):
#                 new_points = converted[j,0:3]
#                 pointCloud.addPoint(new_points,0)


#     renderer = vtk.vtkRenderer()
#     renderWindow = vtk.vtkRenderWindow()
#     renderWindow.AddRenderer(renderer)

#     renderWindowInteractor = vtk.vtkRenderWindowInteractor()
#     renderWindowInteractor.SetRenderWindow(renderWindow)
#     renderer.AddActor(pointCloud.point_vtkActor)
#     renderer.SetBackground(0.0, 0.0, 0.0)
#     renderWindow.Render()
#     renderWindowInteractor.Initialize()

#     renderWindowInteractor.Start()




# if __name__ == "__main__":
#     main()