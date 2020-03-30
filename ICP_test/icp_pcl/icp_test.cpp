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
  // int iteration;
  // pcl::registration::DefaultConvergenceCriteria<double> criteria(iteration, trs_matrix, *rejected_correspondences);
  // criteria.setMaximumIterations(100);
  // criteria.setMaximumIterationsSimilarTransforms(10);
  // criteria.setTranslationThreshold(0.1);
  // criteria.setRotationThreshold(std::cos(0.5 * 3.141592 / 180.0));
  // criteria.setRelativeMSE(0.001);
  
  seg.setOptimizeCoefficients (true);

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.2);

  for(int i = 1; i < 1200; i = i+100){  

    std::string path = "/home/haeyeon/Lidar/RIST_data/";
    std::string ext = ".pcd";
    std::string filename = path + std::to_string(i) + ext;

    reader.read(filename, *source_cloud);
    sor.setInputCloud (source_cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter(*source_filtered);
    pcl::fromPCLPointCloud2(*source_filtered, *source);

    //uniform sampling
    uniform_sampling.setInputCloud(target);
    uniform_sampling.setRadiusSearch(0.03f);
    uniform_sampling.filter(*target);

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
    // iteration = 0;
    // res_matrix.setIdentity();
    // initial_estimation.setInputSource(source);
    // initial_estimation.setInputTarget(target);
    // initial_estimation.determineCorrespondences(*correspondences, 0.1);
    // rejecter.setInputSource<pcl::PointXYZ>(source);
    // rejecter.setInputTarget<pcl::PointXYZ>(target);
    // rejecter.setInputCorrespondences(correspondences);
    // rejecter.setMaximumDistance(0.1);
    // rejecter.getCorrespondences(*rejected_correspondences);
    // transformation.estimateRigidTransformation(*source, *target, *rejected_correspondences, trs_matrix);
    // pcl::transformPointCloud(*source, *source, trs_matrix);
    // res_matrix = res_matrix * trs_matrix;
    // pcl::transformPointCloud(result, result, res_matrix);
    // result += *source;
    // viewer.addPointCloud (source, "cloud"+std::to_string(i));
    // viewer.spinOnce();
    //icp
    pcl::transformPointCloud(*source, *source, transformation_matrix);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations (200);
    icp.align(*source);
    if(icp.hasConverged()){
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << std::to_string(i) << std::endl;
        viewer.addPointCloud (source, "cloud"+std::to_string(i));
        viewer.spinOnce();
      }
    transformation_matrix = icp.getFinalTransformation();
  }

  viewer.spin();
  return(0);
}

