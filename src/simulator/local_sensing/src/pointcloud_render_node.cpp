#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher sense_cloud_pub;
ros::Publisher local_cloud_pub;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

std::string sensor;
Matrix4d laser2body;
Matrix4d laser2world;
Eigen::Quaterniond laser2world_quat;
nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;

ros::Time last_odom_stamp = ros::TIME_MAX;

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Eigen::Matrix4d body_pose = Pose_receive;
  //convert to laser pose
  laser2world = body_pose * laser2body;
  // laser2world_quat = laser2world.block<3, 3>(0, 0);
  // laser2world_rot = laser2world.block<3, 3>(0, 0);
  // laser2world_pos = Eigen::Vector3d(laser2world(0, 3), laser2world{1, 3}, laser2world(2, 3));

  last_odom_stamp = odom.header.stamp;

  // last_pose_world(0) = odom.pose.pose.position.x;
  // last_pose_world(1) = odom.pose.pose.position.y;
  // last_pose_world(2) = odom.pose.pose.position.z;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map, _sense_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd, _sense_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;

  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  pcl::PointXYZ pt_in_laser;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      if ((fabs(pt.z - _odom.pose.pose.position.z) / (sensing_horizon)) >
          tan(M_PI / 12.0))
        continue;

      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      if (pt_vec.dot(yaw_vec) < 0) continue;

      // cloud point in world coordinate for visualization
      _local_map.points.push_back(pt);
      // cloud point to laser coordinate
      Eigen::Vector4d pose_in_world(pt.x, pt.y, pt.z, 1.0);
      Eigen::Vector4d pose_in_laser = laser2world.inverse() * pose_in_world;
      pt_in_laser.x = pose_in_laser(0);
      pt_in_laser.y = pose_in_laser(1);
      pt_in_laser.z = pose_in_laser(2);
      _sense_map.points.push_back(pt_in_laser);
    }
  } else {
    return;
  }

  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "world";

  local_cloud_pub.publish(_local_map_pcd);

  _sense_map.width = _sense_map.points.size();
  _sense_map.height = 1;
  _sense_map.is_dense = true;

  pcl::toROSMsg(_sense_map, _sense_map_pcd);
  _sense_map_pcd.header.frame_id = sensor;

  sense_cloud_pub.publish(_sense_map_pcd);
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);
  nh.getParam("sensor", sensor);

  laser2body << 0.0, 0.0, 1.0, 0.0,
                -1.0, 0.0, 0.0, 0.0,
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  sense_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("sense_cloud", 10);
  local_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 10);
  

  double sensing_duration = 1.0 / sensing_rate * 2.5;

  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
