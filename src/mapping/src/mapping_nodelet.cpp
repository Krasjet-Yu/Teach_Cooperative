#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

#include <atomic>
#include <thread>

namespace mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    SyncPolicyImageOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
    SyncPolicyImagePose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    SyncPolicyPointCloudOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>
    SyncPolicyPointCloudPose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyPointCloudOdom>> SynchronizerPointCloudOdom;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyPointCloudPose>> SynchronizerPointCloudPose;

struct CamConfig {
  // camera paramters
  double rate;
  double range;
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scaling_factor;
};

struct LidarConfig {
  double rate;
  double range;
  double height;
};

enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;

  // map param
  Eigen::Vector3d map_size;
  Eigen::Vector3d map_origin;
  Eigen::Vector3d map_min_boundary;
  Eigen::Vector3d map_max_boundary;
  Eigen::Vector3i local_bound_max_;
  Eigen::Vector3i local_bound_min_;

  std::string sensor_type_;
  // laser param

  // camera param
  CamConfig camConfig_;  // just store the parameters of camera
  LidarConfig laserConfig_;

  int down_sample_factor_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;

  Eigen::Matrix3d laser2body_R_;
  Eigen::Vector3d laser2body_p_;

  // just for depth filter
  Eigen::Vector3d last_cam_p_;
  Eigen::Quaterniond last_cam_q_;
  bool get_first_frame_ = false;
  cv::Mat last_depth_;
  double depth_filter_tolerance_;
  double depth_filter_mindist_;
  int depth_filter_margin_;

  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  // camera
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  // laser
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> local_pc_sub_;

  // camera
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;
  // laser
  SynchronizerPointCloudPose sync_pc_pose_;
  SynchronizerPointCloudOdom sync_pc_odom_;

  ros::Timer vis_timer_;
  ros::Publisher gridmap_inflate_pub_, local_pc_pub_, pcl_pub_;
  ros::Publisher vis_map_pub_, vis_map_inf_pub_;

  // NOTE just for global map in simulation
  ros::Timer global_map_timer_;
  ros::Subscriber map_pc_sub_;
  bool map_recieved_ = false;
  int pose_type_;

  // NOTE for mask target
  bool use_mask_ = false;
  ros::Subscriber target_odom_sub_;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d target_odom_;

  OccGridMap gridmap_;
  int inflate_size_;

  void depthOdomCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg) {
    if (callback_lock_.test_and_set()) {
      return;
    }
    ros::Time t1, t2;
    // t1 = ros::Time::now();
    // body coordinate -> odom（world） coordinate
    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    // Rotation Matrix: camera coordinate -> body coordinate -> odom coordinate
    Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, camConfig_.depth_scaling_factor);
    }
    cv::Mat depth_img = depth_ptr->image;

    // pub target

    int nr = depth_img.rows;
    int nc = depth_img.cols;
    std::vector<Eigen::Vector3d> obs_pts;
    // put the points of the depth into the list of obs_points

    // TODO depth filter

    // t1 = ros::Time::now();
    // camera pixel -> obs points with depth in odom(world) coordinate
    for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
      for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
        // (x,y,z) in camera frame
        double z = (depth_img.at<uint16_t>(i, j)) / camConfig_.depth_scaling_factor;
        if (depth_img.at<uint16_t>(i, j) == 0) {
          z = camConfig_.range + 0.5;
        }
        if (std::isnan(z) || std::isinf(z))
          continue;
        if (z < depth_filter_mindist_) {
          continue;
        }
        // pixel coordinate -> camera coordinate
        double y = (i - camConfig_.cy) * z / camConfig_.fy;
        double x = (j - camConfig_.cx) * z / camConfig_.fx;
        Eigen::Vector3d p(x, y, z); // in camera coordinate
        p = cam_q * p + cam_p;      // in odom(world) coordinate
        bool good_point = true;
        if (get_first_frame_) {
          // NOTE depth filter:
          Eigen::Vector3d p_rev_proj =
              last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
          double vv = p_rev_proj.y() * camConfig_.fy / p_rev_proj.z() + camConfig_.cy;
          double uu = p_rev_proj.x() * camConfig_.fx / p_rev_proj.z() + camConfig_.cx;
          if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
            double drift_dis = fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / camConfig_.depth_scaling_factor - p_rev_proj.z());
            if (drift_dis > depth_filter_tolerance_) {
              good_point = false;
            }
          }
        }
        if (good_point) {
          obs_pts.push_back(p);
        }
      }
    }

    last_depth_ = depth_img;
    last_cam_p_ = cam_p;
    last_cam_q_ = cam_q;
    get_first_frame_ = true;
    gridmap_.updateMap(cam_p, obs_pts);

    // NOTE use mask
    if (use_mask_) {  // mask target
      while (target_lock_.test_and_set())
        ;
      Eigen::Vector3d ld = target_odom_;
      Eigen::Vector3d ru = target_odom_;
      ld.x() -= 0.5;
      ld.y() -= 0.5;
      ld.z() -= 1.0;
      ru.x() += 0.5;
      ru.y() += 0.5;
      ru.z() += 1.0;
      gridmap_.setFree(ld, ru);
      target_lock_.clear();
    }

    // TODO pub local map
    // sensor_msgs::PointCloud2 pc_msg;
    // pcl::PointCloud<pcl::PointXYZ> pcd;
    // pcl::PointXYZ pt;
    // std::vector<Eigen::Vector3d> mask_pts_ =  obs_pts;
    // for (const auto p : mask_pts_) {
    //   pt.x = p.x();
    //   pt.y = p.y();
    //   pt.z = p.z();
    //   pcd.push_back(pt);
    // }
    // pcd.width = pcd.points.size();
    // pcd.height = 1;
    // pcd.is_dense = true;
    // pcl::toROSMsg(pcd, pc_msg);
    // pc_msg.header.frame_id = "world";
    // local_pc_pub_.publish(pc_msg);
    // std::cout << "publish local points " << std::endl;

    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
    // std::cout << "publish gridmap inflate " << std::endl;

    callback_lock_.clear();
  }

  void PointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                              const nav_msgs::OdometryConstPtr& odom_msg) {
    if (callback_lock_.test_and_set()) {
      return;
    }
    ros::Time t1, t2;
    // t1 = ros::Time::now();
    // body coordinate -> odom（world） coordinate
    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    // Rotation Matrix: camera coordinate -> body coordinate -> odom coordinate
    Eigen::Vector3d laser_p = body_q.toRotationMatrix() * laser2body_p_ + body_p;
    Eigen::Quaterniond laser_q = body_q * Eigen::Quaterniond(laser2body_R_);
    std::vector<Eigen::Vector3d> obs_pts;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*cloud_msg, point_cloud);
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      // gridmap_.setOcc(p);
      p = laser_q * p + laser_p;
      if (p(3) < laserConfig_.height) continue;
      obs_pts.push_back(p);
    }
    // gridmap_.inflate(inflate_size_);
    gridmap_.updateMap(laser_p, obs_pts);

    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
    // std::cout << "publish gridmap inflate " << std::endl;

    callback_lock_.clear();
  }

  // NOTE
  void target_odom_callback(const nav_msgs::OdometryConstPtr& msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_odom_.x() = msgPtr->pose.pose.position.x;
    target_odom_.y() = msgPtr->pose.pose.position.y;
    target_odom_.z() = msgPtr->pose.pose.position.z;
    target_lock_.clear();
  }

  // NOTE just for global map in simulation
  void map_call_back(const sensor_msgs::PointCloud2ConstPtr& msgPtr) {
    if (map_recieved_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      gridmap_.setOcc(p);
    }
    gridmap_.inflate(inflate_size_);
    ROS_WARN("[mapping] GLOBAL MAP REVIEVED!");
    map_recieved_ = true;
    return;
  }
  
  void global_map_timer_callback(const ros::TimerEvent& event) {
    if (!map_recieved_) {
      return;
    }
    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
  }

  void visCallback(const ros::TimerEvent & /*event*/)
  {
    publishMapInflate(true);
    publishMap();
  }

  void publishMapInflate(bool all_info) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = map_max_boundary(0);
    min_y = map_max_boundary(1);
    min_z = map_max_boundary(2);

    max_x = map_min_boundary(0);
    max_y = map_min_boundary(1);
    max_z = map_min_boundary(2);

    local_bound_max_ = gridmap_.pos2idx(Eigen::Vector3d(max_x, max_y, max_z));
    local_bound_min_ = gridmap_.pos2idx(Eigen::Vector3d(min_x, min_y, min_z));
    Eigen::Vector3i min_cut = local_bound_min_;
    Eigen::Vector3i max_cut = local_bound_max_;

    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
          if (gridmap_.isOccupied(Eigen::Vector3i(x, y, z))) {
            Eigen::Vector3d pos;
            pos = gridmap_.idx2pos(Eigen::Vector3i(x, y, z));
            // if (pos(2) > mp_.visualization_truncate_height_)
            //   continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    vis_map_inf_pub_.publish(cloud_msg);
  }

  void publishMap() {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = map_max_boundary(0);
    min_y = map_max_boundary(1);
    min_z = map_max_boundary(2);

    max_x = map_min_boundary(0);
    max_y = map_min_boundary(1);
    max_z = map_min_boundary(2);

    local_bound_max_ = gridmap_.pos2idx(Eigen::Vector3d(max_x, max_y, max_z));
    local_bound_min_ = gridmap_.pos2idx(Eigen::Vector3d(min_x, min_y, min_z));
    Eigen::Vector3i min_cut = local_bound_min_;
    Eigen::Vector3i max_cut = local_bound_max_;

    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
          if (gridmap_.isProbOccupied(Eigen::Vector3i(x, y, z))) {
            Eigen::Vector3d pos;
            pos = gridmap_.idx2pos(Eigen::Vector3i(x, y, z));
            // if (pos(2) > mp_.visualization_truncate_height_)
            //   continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    vis_map_pub_.publish(cloud_msg);
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of mapping
    // cam2body_R_ << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    // cam2body_p_.setZero();
    std::vector<double> tmp;
    if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
    }
    if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
      cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
    }
    if (nh.param<std::vector<double>>("laser2body_R", tmp, std::vector<double>())) {
      laser2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
    }
    if (nh.param<std::vector<double>>("laser2body_p", tmp, std::vector<double>())) {
      laser2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
    }
    // std::cout << "R: \n" << cam2body_R_ << std::endl;
    // std::cout << "p: \n" << cam2body_p_ << std::endl;
    double res;

    // NOTE whether to use global map
    nh.getParam("sensor_type", sensor_type_);
    nh.getParam("pose_type", pose_type_);
    // mapping parameters
    nh.getParam("down_sample_factor", down_sample_factor_);
    nh.getParam("resolution", res);
    nh.getParam("local_x", map_size.x());
    nh.getParam("local_y", map_size.y());
    nh.getParam("local_z", map_size.z());
    nh.getParam("inflate_size", inflate_size_);

    map_origin = Eigen::Vector3d(-map_size.x()/2.0, -map_size.y()/2.0, 0.0);
    map_min_boundary = map_origin;
    map_max_boundary = map_origin + map_size;

    if(sensor_type_ == std::string("camera")) {
      // camera parameters
      nh.getParam("camera_rate", camConfig_.rate);
      nh.getParam("camera_range", camConfig_.range);
      nh.getParam("cam_width", camConfig_.width);
      nh.getParam("cam_height", camConfig_.height);
      nh.getParam("cam_fx", camConfig_.fx);
      nh.getParam("cam_fy", camConfig_.fy);
      nh.getParam("cam_cx", camConfig_.cx);
      nh.getParam("cam_cy", camConfig_.cy);
      nh.getParam("depth_scaling_factor", camConfig_.depth_scaling_factor);
      // depth filter parameters
      nh.getParam("depth_filter_tolerance", depth_filter_tolerance_);
      nh.getParam("depth_filter_mindist", depth_filter_mindist_);
      nh.getParam("depth_filter_margin", depth_filter_margin_);
      gridmap_.setup(res, map_size, camConfig_.range);
    }
    else if(sensor_type_ == std::string("laser")) {
      // laser parameters
      nh.getParam("laser_rate", laserConfig_.rate);
      nh.getParam("laser_range", laserConfig_.range);
      nh.getParam("laser_height", laserConfig_.height);
      gridmap_.setup(res, map_size, laserConfig_.range);
    }
    
    // raycasting parameters
    int p_min, p_max, p_hit, p_mis, p_occ, p_def;
    nh.getParam("p_min", p_min);
    nh.getParam("p_max", p_max);
    nh.getParam("p_hit", p_hit);
    nh.getParam("p_mis", p_mis);
    nh.getParam("p_occ", p_occ);
    nh.getParam("p_def", p_def);

    gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
    gridmap_.inflate_size = inflate_size_;
    // use mask parameter
    nh.getParam("use_mask", use_mask_);

    gridmap_inflate_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1);
    vis_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_occupancy", 10);
    vis_map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_occupancy_inflate", 10);

    local_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 1);
    pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("mask_cloud", 10);
    if (sensor_type_ == std::string("camera")) {
      if (pose_type_ == POSE_STAMPED) {
        // TODO: pose and odom
        map_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("global_map", 1, &Nodelet::map_call_back, this);
        global_map_timer_ = nh.createTimer(ros::Duration(1.0), &Nodelet::global_map_timer_callback, this);
      } 
      else if (pose_type_ == ODOMETRY) {
        depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "depth", 50));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 100, ros::TransportHints().tcpNoDelay()));
        sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
        sync_image_odom_->registerCallback(boost::bind(&Nodelet::depthOdomCallback, this, _1, _2));
      }
    }
    else if (sensor_type_ == std::string("laser")) {
      if (pose_type_ == POSE_STAMPED) {
        // TODO: pose and odom
        map_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("global_map", 1, &Nodelet::map_call_back, this);
        global_map_timer_ = nh.createTimer(ros::Duration(1.0), &Nodelet::global_map_timer_callback, this);
      } 
      else if (pose_type_ == ODOMETRY) {
        local_pc_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "sense_map", 50));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 100, ros::TransportHints().tcpNoDelay()));
        sync_pc_odom_.reset(new message_filters::Synchronizer<SyncPolicyPointCloudOdom>(
            SyncPolicyPointCloudOdom(100), *local_pc_sub_, *odom_sub_));
        sync_pc_odom_->registerCallback(boost::bind(&Nodelet::PointCloudOdomCallback, this, _1, _2));
      }
    }

    vis_timer_ = nh.createTimer(ros::Duration(0.11), &Nodelet::visCallback, this);

    if (use_mask_) {
      target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 1, &Nodelet::target_odom_callback, this, ros::TransportHints().tcpNoDelay());
    }
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapping::Nodelet, nodelet::Nodelet);