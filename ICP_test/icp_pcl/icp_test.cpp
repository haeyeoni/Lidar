#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/filters/uniform_sampling.h>

int main ( int argc, char** argv )
{
  pcl::PCLPointCloud2::Ptr source_cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr source_filtered (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr target_cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr target_filtered (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_done (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  pcl::CorrespondencesPtr rejected_correspondences(new pcl::Correspondences);
  pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, double> transformation;
  pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, double>::Matrix4 trs_matrix;
  pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, double>::Matrix4 res_matrix;
  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> initial_estimation;
  pcl::registration::CorrespondenceRejectorDistance rejecter;
  
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

    if(target->empty()) {
      target = source;
      continue;
    }

    //icp
    //pcl::transformPointCloud(*source, *source, transformation_matrix);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 
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
  }

  viewer.spin();
  return(0);
}

