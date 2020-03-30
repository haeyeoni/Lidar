#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
#include <pcl/registration/icp.h>
int main (int argc, char** argv)
{
  // Initialize ROS

  std::cout << "Lidar node activated." << std::endl;
  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  std::stringstream s;
  //VeloSubscribetoPCL velsub;
  pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_done (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pretrans_icp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
     
     
      pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
      voxelGrid.setInputCloud(Initial_cloud);
      voxelGrid.setLeafSize(0.1, 0.1, 0.1);
      voxelGrid.filter(*cloudVoxel);
      std::cout<<"Saved Initial frame"<<std::endl;

      pcl::fromPCLPointCloud2(*cloudVoxel, *Initial_frame);
      // Optional
      seg.setOptimizeCoefficients (true);

      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.2);
      seg.setInputCloud (Initial_frame);
      seg.segment (*inliers, coefficients);
      ROS_INFO("extract the inliers"); 
      extract.setInputCloud (Initial_frame);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*Initial_frame);

      icp.setInputSource(1);
      icp.setInputTarget(1);

      viewer.addPointCloud (Initial_frame, "initial");
      s.str("");
      s.clear();
      continue;
    } 
  
  viewer.spin();
  return 0;
}
