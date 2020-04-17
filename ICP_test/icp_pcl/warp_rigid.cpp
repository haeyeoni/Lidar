
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using Cloud2 = pcl::PCLPointCloud2;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;

int
main (int argc, char **argv)
{
  double dist = 0.05;
  double rans = 0.05;
  int iter = 100;
  Cloud2::Ptr source_cloud (new Cloud2 ());
  Cloud2::Ptr source_filtered (new Cloud2 ());
  Cloud2::Ptr target_cloud (new Cloud2 ());
  Cloud2::Ptr target_filtered (new Cloud2 ());
  Cloud::Ptr cloud_icp_done (new Cloud);
  Cloud::Ptr source (new Cloud);
  Cloud::Ptr target (new Cloud);
  pcl::PCDReader reader;
  pcl::VoxelGrid<Cloud2> sor;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ExtractIndices<PointType> extract;
  pcl::SACSegmentation<PointType> seg;
  pcl::ModelCoefficients coefficients;
  Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());
  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  seg.setOptimizeCoefficients (true);

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.2);


   
  for(int i = 1; i < 743; i = i+1){  

    std::string path = "/home/haeyeon/Lidar/ICP_test/data1/";
    std::string ext = ".pcd";
    std::string filename = path + std::to_string(i) + ext;

    reader.read(filename, *source_cloud);
    sor.setInputCloud (source_cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter(*source_filtered);
    pcl::fromPCLPointCloud2(*source_filtered, *source);

    //remove ground plane
    seg.setInputCloud (source);
    seg.segment (*inliers, coefficients);
    extract.setInputCloud (source);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*source);

    // initialize the target cloud
    if(target->empty()) {
      target = source;
      continue;
    }
    //icp
    //pcl::transformPointCloud(*source, *source, transformation_matrix);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 
    pcl::registration::WarpPointRigid3D<PointType, PointType>::Ptr warp_fcn 
      (new pcl::registration::WarpPointRigid3D<PointType, PointType>);
    pcl::registration::TransformationEstimationLM<PointType, PointType>::Ptr te (new pcl::registration::TransformationEstimationLM<PointType, PointType>);
    te->setWarpFunction (warp_fcn);
    icp.setTransformationEstimation (te);
    //icp.setMaximumIterations (iter);
    // icp.setMaxCorrespondenceDistance (dist);
    // icp.setRANSACOutlierRejectionThreshold (rans);
    icp.setInputSource(source);
    icp.setInputTarget(target);
    //icp.setMaximumIterations (500);
    icp.align(*target);
    if(icp.hasConverged()){
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << std::to_string(i) << std::endl;
        viewer.addPointCloud (target, "cloud"+std::to_string(i));
        viewer.spinOnce();
      }
    //transformation_matrix = icp.getFinalTransformation();
    std::cout << icp.getFinalTransformation () << std::endl;
  }

  viewer.spin();
  return(0);
}