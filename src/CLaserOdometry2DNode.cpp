/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>

namespace rf2o {

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(const rclcpp::NodeOptions & options):
  Node("CLaserOdometry2DNode", options),
  rf2o_ref(get_logger(), get_clock())
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "/base_pose_ground_truth");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);
  if (!std::isfinite(freq) || freq <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "Invalid freq; using 10.0 Hz");
    freq = 10.0;
  }
  this->declare_parameter<bool>("publish_covariance", true);
  this->get_parameter("publish_covariance", publish_covariance);
  this->declare_parameter<bool>("use_rf2o_twist_covariance", false);
  this->get_parameter("use_rf2o_twist_covariance", use_rf2o_twist_covariance);
  this->declare_parameter<std::vector<double>>(
    "pose_covariance_diagonal",
    {0.0025, 0.0025, 1000000.0, 1000000.0, 1000000.0, 0.0025});
  this->get_parameter("pose_covariance_diagonal", pose_covariance_diagonal);
  this->declare_parameter<std::vector<double>>(
    "twist_covariance_diagonal",
    {0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.01});
  this->get_parameter("twist_covariance_diagonal", twist_covariance_diagonal);
  this->declare_parameter<double>("covariance_scale", 1.0);
  this->get_parameter("covariance_scale", covariance_scale);

  if (!covarianceDiagonalValid(pose_covariance_diagonal, "pose_covariance_diagonal"))
    pose_covariance_diagonal = {0.0025, 0.0025, 1000000.0, 1000000.0, 1000000.0, 0.0025};
  if (!covarianceDiagonalValid(twist_covariance_diagonal, "twist_covariance_diagonal"))
    twist_covariance_diagonal = {0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.01};
  if (!std::isfinite(covariance_scale) || covariance_scale <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "Invalid covariance_scale; using 1.0");
    covariance_scale = 1.0;
  }

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  if (publish_tf)
  {
    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));

  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
  new_scan_available = false;

  // Create processing timer
  const auto period = std::chrono::duration<double>(1.0 / freq);
  timer_ = this->create_wall_timer(period, std::bind(&CLaserOdometry2DNode::process, this));
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    if (new_scan->ranges.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping empty laser scan");
      return;
    }

    if (!rf2o_ref.first_laser_scan && new_scan->ranges.size() != rf2o_ref.width)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping laser scan with width %zu; expected %u",
        new_scan->ranges.size(), rf2o_ref.width);
      return;
    }

    // Keep in memory the last received laser_scan
    last_scan = new_scan;
    rf2o_ref.current_scan_time = last_scan->header.stamp;

    if (rf2o_ref.first_laser_scan == false)
    {
      // inform of new scan available
      new_scan_available = true;
    }
    else
    {
      // Initialize module on first scan (from laser params)
      if (!setLaserPoseFromTf())
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for TF from laser frame [%s] to base frame [%s]",
          last_scan->header.frame_id.c_str(), base_frame_id.c_str());
        return;
      }

      if (rf2o_ref.init(*last_scan, initial_robot_pose.pose.pose))
        rf2o_ref.first_laser_scan = false;
    }
  }
}


/** 
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{  
  bool retrieved = false;  
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan->header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o 
  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  // Do only run when a new scan is ready 
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    // Process odometry estimation
    const bool odometry_updated = rf2o_ref.odometryCalculation(*last_scan);

    if (odometry_updated)
    {
      // Publish odometry over ROS2 (tf/topic)
      publish();
    }

    // Do not run on the same data, even when this scan was rejected.
    new_scan_available = false;
  }
  else
  {
    // This is a warning. We depend on laser scans, so no meaning running faster than scan freq.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for laser_scans....");
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  
  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = rf2o_ref.lin_speed_y;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed

  if (publish_covariance)
  {
    fillConfiguredCovariance(odom.pose.covariance, pose_covariance_diagonal);
    fillTwistCovariance(odom);
  }

  //publish the message
  odom_pub->publish(odom);

  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf && odom_broadcaster)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
  }
}

bool CLaserOdometry2DNode::covarianceDiagonalValid(const std::vector<double>& diagonal,
                                                   const char* name)
{
  if (diagonal.size() != 6)
  {
    RCLCPP_WARN(get_logger(), "%s must contain exactly 6 values", name);
    return false;
  }

  for (const double value : diagonal)
  {
    if (!std::isfinite(value) || value < 0.0)
    {
      RCLCPP_WARN(get_logger(), "%s contains an invalid covariance value", name);
      return false;
    }
  }

  return true;
}

void CLaserOdometry2DNode::fillConfiguredCovariance(std::array<double, 36>& covariance,
                                                    const std::vector<double>& diagonal)
{
  covariance.fill(0.0);
  for (std::size_t i = 0; i < 6; ++i)
    covariance[i * 6 + i] = diagonal[i];
}

void CLaserOdometry2DNode::fillTwistCovariance(nav_msgs::msg::Odometry& odom)
{
  fillConfiguredCovariance(odom.twist.covariance, twist_covariance_diagonal);

  if (!use_rf2o_twist_covariance)
    return;

  const IncrementCov& cov_laser = rf2o_ref.getIncrementCovariance();
  if (!cov_laser.allFinite())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "RF2O twist covariance is non-finite; using configured twist covariance");
    return;
  }

  const Eigen::Matrix3d rotation = rf2o_ref.laser_pose_on_robot_.rotation();
  const Eigen::Vector3d translation = rf2o_ref.laser_pose_on_robot_.translation();
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  jacobian(0, 0) = rotation(0, 0);
  jacobian(0, 1) = rotation(0, 1);
  jacobian(0, 2) = translation.y();
  jacobian(1, 0) = rotation(1, 0);
  jacobian(1, 1) = rotation(1, 1);
  jacobian(1, 2) = -translation.x();
  jacobian(2, 0) = 0.0;
  jacobian(2, 1) = 0.0;
  jacobian(2, 2) = 1.0;

  const Eigen::Matrix3d cov_base =
    covariance_scale * jacobian * cov_laser.cast<double>() * jacobian.transpose();

  if (!cov_base.allFinite())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Base-frame twist covariance is non-finite; using configured twist covariance");
    return;
  }

  const int indexes[3] = {0, 1, 5};
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
      odom.twist.covariance[indexes[r] * 6 + indexes[c]] = cov_base(r, c);
  }

  for (int i = 0; i < 3; ++i)
  {
    const int index = indexes[i];
    const double floor = twist_covariance_diagonal[index];
    const double value = odom.twist.covariance[index * 6 + index];
    odom.twist.covariance[index * 6 + index] = std::max(value, floor);
  }
}

} /* namespace rf2o */


RCLCPP_COMPONENTS_REGISTER_NODE(rf2o::CLaserOdometry2DNode)
