#include <ros/ros.h>
#include <sstream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
class VeloSubscribetoPCL
{
  public:
      VeloSubscribetoPCL()
      {
        this->subscriber = this->nh.subscribe("/ce30_points", 10, &VeloSubscribetoPCL::SubVelodyne, this);
        this->publisher = this->nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
      }
      
    void SubVelodyne(const sensor_msgs::PointCloud &cloud_msg)
    {
        sensor_msgs::PointCloud2 output;
        sensor_msgs::convertPointCloudToPointCloud2(cloud_msg, output);
        this->publisher.publish(output);
    }

  private:
      ros::NodeHandle nh;
      ros::Subscriber subscriber;
      ros::Publisher publisher;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  VeloSubscribetoPCL VeloSubscribetoPCL;
  ros::spin();
}