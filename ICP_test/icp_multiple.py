import pcl
import random
import numpy as np
import vtk
import VTKPointCloud as vpc
import pcl.pcl_visualization
def main():

    ##load cloud point
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()
    cloud_in.from_file(bytes(b'/home/haeyeon/Lidar/data1/1.pcd'))
    point_size = 0
    pointCloud = vpc.VtkPointCloud()
    rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
    translation = np.array([0,0,0])
    result_point = [np.array(np.asarray(cloud_in))]
    for i in range(result_point[0].shape[0]):
        pointCloud.addPoint(result_point[0][i,0:3],1)

    
    for i in range(2,1000,50):
    #cloud_in.from_file(bytes(b'/home/haeyeon/cocel/data1/1.pcd'))
        filename = '/home/haeyeon/cocel/data1/{}.pcd'.format(i)
        print(filename)
        cloud_out.from_file(filename.encode())
        icp = cloud_in.make_IterativeClosestPoint()
    ## ICP algorithm ##
        #transform_out = np.array(np.asarray(cloud_out))
        #transform_out = np.add(transform_out.dot(rotation),translation)
        #cloud_out.from_array(transform_out)
        converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
        ## Transfrom the input point cloud
        rotation = transf[0:3,0:3]
        translation = transf[3,0:3]
        outArray = np.array(np.asarray(cloud_out))
        # for j in range(outArray.shape[0]):
        #     pointCloud.addPoint(outArray[j,0:3],-1)

    
        converted = np.add(np.matmul(estimate, rotation),translation)
        #cloud_in.from_array(converted)
        if(converged):
            for j in range(estimate.size):
                new_points = converted[j,0:3]
                # if(not new_points in result_point[]):
                #result_point.append(new_points)
                pointCloud.addPoint(new_points,0)
#     for j in range(len(result_point)):
#         if j!=0:
#             cloud_out.from_array(result_point[j])
# #print(str(transf))
#         visual = pcl.pcl_visualization.CloudViewing()
# # PointXYZ
#         visual.ShowMonochromeCloud(cloud_out, b'cloud')
#     v = True
#     while v:
#         v = not(visual.WasStopped())
    ## Render with vtk

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