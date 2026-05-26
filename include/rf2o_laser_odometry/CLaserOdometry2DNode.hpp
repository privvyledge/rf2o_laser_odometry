#ifndef RF2O_LASER_ODOMETRY__CLASERODOMETRY2DNODE_HPP_
#define RF2O_LASER_ODOMETRY__CLASERODOMETRY2DNODE_HPP_

#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <vector>

namespace rf2o {

class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  explicit CLaserOdometry2DNode(const rclcpp::NodeOptions & options);
  void process();
  void publish();
  bool setLaserPoseFromTf();
  bool scan_available();

  // Params & vars
  CLaserOdometry2D    rf2o_ref;
  bool                publish_tf, new_scan_available;
  bool                publish_covariance, use_rf2o_twist_covariance;
  double              freq;
  double              covariance_scale;
  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;
  std::vector<double> pose_covariance_diagonal;
  std::vector<double> twist_covariance_diagonal;

  sensor_msgs::msg::LaserScan::SharedPtr          last_scan;
  bool                                            GT_pose_initialized;
  std::shared_ptr<tf2_ros::Buffer>                buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;  
  std::unique_ptr<tf2_ros::TransformBroadcaster>  odom_broadcaster;
  nav_msgs::msg::Odometry                         initial_robot_pose;

  // Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      initPose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  // CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);

  void fillConfiguredCovariance(std::array<double, 36>& covariance,
                                const std::vector<double>& diagonal);
  void fillTwistCovariance(nav_msgs::msg::Odometry& odom);
  bool covarianceDiagonalValid(const std::vector<double>& diagonal,
                               const char* name);
};

} // namespace rf2o

#endif // RF2O_LASER_ODOMETRY__CLASERODOMETRY2DNODE_HPP_
