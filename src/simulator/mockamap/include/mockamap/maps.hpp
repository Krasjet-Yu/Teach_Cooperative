#ifndef MAPS_HPP
#define MAPS_HPP 

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

namespace mocka {

  class Maps {
  public:
    typedef struct BasicInfo
    {
      ros::NodeHandle *nh_private;
      int sizeX;
      int sizeY;
      int sizeZ;
      int seed;
      double scale;
      sensor_msgs::PointCloud2 *output;
      pcl::PointCloud<pcl::PointXYZ> *cloud;
    } BasicInfo;
    
    BasicInfo getInfo() const;
    void setInfo(const BasicInfo &value);
  

  public:
    Maps();

  public:
    void generate(int type);
    void addMap(const geometry_msgs::PoseStamped::ConstPtr& msg);

  private:
    BasicInfo info;

  private:
    void pcl2ros();
    void randomMapGenerate();
    void addGround();
  };
}
#endif