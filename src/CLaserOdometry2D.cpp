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

#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

#include <cmath>
#include <limits>

namespace rf2o {

namespace {
constexpr float kEpsilon = 1e-6f;

bool solveNormalEquations(const Eigen::MatrixXf& AtA,
                          const Eigen::MatrixXf& AtB,
                          Eigen::MatrixXf& solution,
                          Eigen::MatrixXf* inverse_ata)
{
  if (!AtA.allFinite() || !AtB.allFinite())
    return false;

  Eigen::ColPivHouseholderQR<Eigen::MatrixXf> solver(AtA);
  solver.setThreshold(kEpsilon);
  if (solver.rank() < AtA.cols())
    return false;

  solution = solver.solve(AtB);
  if (!solution.allFinite())
    return false;

  if (inverse_ata != nullptr)
  {
    *inverse_ata = solver.solve(Eigen::MatrixXf::Identity(AtA.rows(), AtA.cols()));
    if (!inverse_ata->allFinite())
      return false;
  }

  return true;
}
}


/**
 * Constructor that inherits from Node
*/
CLaserOdometry2D::CLaserOdometry2D(const rclcpp::Logger& logger,
                                   const rclcpp::Clock::SharedPtr& clock) :
  logger_(logger),
  clock_(clock ? clock : std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
  verbose(false),
  module_initialized(false),
  first_laser_scan(true),
  lin_speed(0.0),
  lin_speed_y(0.0),
  ang_speed(0.0),
  last_increment_(Pose3d::Identity()),
  laser_pose_on_robot_(Pose3d::Identity()),
  laser_pose_on_robot_inv_(Pose3d::Identity()),
  laser_pose_(Pose3d::Identity()),
  laser_oldpose_(Pose3d::Identity()),
  robot_pose_(Pose3d::Identity()),
  robot_oldpose_(Pose3d::Identity()),
  zv_enabled(true),
  zv_linear_threshold(0.02),
  zv_angular_threshold(0.05),
  zv_scan_diff_threshold(0.03),
  zv_hold_scans(3),
  zv_release_scans(2),
  zv_external_still(true),
  zv_stationary_(false),
  zv_still_count_(0),
  zv_moving_count_(0),
  zv_scan_diff_(0.0),
  zv_scan_diff_valid_(false)
{

}


/**
 * Sets the laser pose with respect the robot base_link, and its inverse
 * @param laser_pose is the transform between laser_frame_id and base_link
*/
void CLaserOdometry2D::setLaserPose(const Pose3d& laser_pose)
{
  laser_pose_on_robot_     = laser_pose;
  laser_pose_on_robot_inv_ = laser_pose_on_robot_.inverse();
}


bool CLaserOdometry2D::is_initialized()
{
  return module_initialized;
}


/**
 * On the first laser scan, gets its parameters and initialize the node
 * 
*/
bool CLaserOdometry2D::init(const sensor_msgs::msg::LaserScan& scan,
                            const geometry_msgs::msg::Pose& initial_robot_pose)
{
  // Obtain laser parametes
  RCLCPP_INFO(logger_, "Got first Laser Scan .... Configuring node");
  width = scan.ranges.size();         // Num of samples (size) of the scan laser
  if (width < 3)
  {
    RCLCPP_WARN(logger_, "Laser scan must contain at least 3 ranges");
    module_initialized = false;
    return false;
  }

  cols = width;						            // Max resolution. Should be similar to the width parameter
  fovh = std::abs(scan.angle_max - scan.angle_min);  // Horizontal Laser's FOV
  if (!std::isfinite(fovh) || fovh <= 0.f)
  {
    RCLCPP_WARN(logger_, "Laser scan has invalid angular field of view");
    module_initialized = false;
    return false;
  }

  ctf_levels = 5;                     // Coarse-to-Fine levels
  iter_irls  = 5;                     // Num iterations to solve iterative reweighted least squares

  // Set the robot initial pose in the "map" frame_id
  // Odometry estimation will carry out from this initial pose
  Pose3d robot_initial_pose = Pose3d::Identity();
  robot_initial_pose = Eigen::Quaterniond(initial_robot_pose.orientation.w,
                                          initial_robot_pose.orientation.x,
                                          initial_robot_pose.orientation.y,
                                          initial_robot_pose.orientation.z);
  robot_initial_pose.translation()(0) = initial_robot_pose.position.x;
  robot_initial_pose.translation()(1) = initial_robot_pose.position.y;

  //RCLCPP_INFO_STREAM(logger_, "[rf2o] Setting origin at:\n"<< robot_initial_pose.matrix());

  // Get the initial laser pose assuming laser is fixed with respect the base_link
  laser_pose_    = robot_initial_pose * laser_pose_on_robot_;
  laser_oldpose_ = laser_pose_;
  robot_pose_    = robot_initial_pose;
  robot_oldpose_ = robot_pose_;


  // Init rf2o module (internal)
  //-----------------------------
  range_wf = Eigen::MatrixXf::Constant(1, width, 1);
  kai_loc_ = MatrixS31::Zero();
  kai_loc_old_ = MatrixS31::Zero();
  kai_loc_level_ = MatrixS31::Zero();

  // Resize vectors according to coarse2fine levels
  transformations.resize(ctf_levels);
  for (unsigned int i = 0; i < ctf_levels; i++)
    transformations[i].resize(3, 3);

  // Resize pyramid
  unsigned int s, cols_i;
  const unsigned int pyr_levels = std::round(std::log2(round(float(width) / float(cols)))) + ctf_levels;
  range.resize(pyr_levels);
  range_old.resize(pyr_levels);
  range_inter.resize(pyr_levels);
  xx.resize(pyr_levels);
  xx_inter.resize(pyr_levels);
  xx_old.resize(pyr_levels);
  yy.resize(pyr_levels);
  yy_inter.resize(pyr_levels);
  yy_old.resize(pyr_levels);
  range_warped.resize(pyr_levels);
  xx_warped.resize(pyr_levels);
  yy_warped.resize(pyr_levels);

  for (unsigned int i = 0; i < pyr_levels; i++)
  {
    s = std::pow(2.f, int(i));
    cols_i = std::ceil(float(width) / float(s));

    range[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    range_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    range_inter[i].resize(1, cols_i);

    xx[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    xx_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

    yy[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    yy_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

    xx_inter[i].resize(1, cols_i);
    yy_inter[i].resize(1, cols_i);

    if (cols_i <= cols)
    {
      range_warped[i].resize(1, cols_i);
      xx_warped[i].resize(1, cols_i);
      yy_warped[i].resize(1, cols_i);
    }
  }

  dt.resize(1, cols);
  dtita.resize(1, cols);
  normx.resize(1, cols);
  normy.resize(1, cols);
  norm_ang.resize(1, cols);
  weights.resize(1, cols);

  null    = Eigen::MatrixXi::Constant(1, cols, 0);
  cov_odo = IncrementCov::Zero();

  fps = 1.f;		//In Hz
  num_valid_range = 0;

  //Compute gaussian mask
  g_mask[0] = 1.f / 16.f;
  g_mask[1] = 0.25f;
  g_mask[2] = 6.f / 16.f;
  g_mask[3] = g_mask[1];
  g_mask[4] = g_mask[0];

  kai_abs_     = MatrixS31::Zero();
  kai_loc_old_ = MatrixS31::Zero();

  zv_stationary_      = false;
  zv_still_count_     = 0;
  zv_moving_count_    = 0;
  zv_scan_diff_       = 0.0;
  zv_scan_diff_valid_ = false;

  if (!sanitizeScanRanges(scan))
  {
    RCLCPP_WARN(logger_, "Failed to sanitize initial laser scan");
    module_initialized = false;
    return false;
  }

  createImagePyramid();

  module_initialized = true;
  last_odom_time = scan.header.stamp;   // the time of this first scan
  return true;
}


const Pose3d& CLaserOdometry2D::getIncrement() const
{
  return last_increment_;
}


const Eigen::Matrix<float, 3, 3>& CLaserOdometry2D::getIncrementCovariance() const
{
  return cov_odo;
}


Pose3d& CLaserOdometry2D::getPose()
{
  return robot_pose_;
}


const Pose3d& CLaserOdometry2D::getPose() const
{
  return robot_pose_;
}

bool CLaserOdometry2D::sanitizeScanRanges(const sensor_msgs::msg::LaserScan& scan)
{
  if (scan.ranges.size() != width || range_wf.size() != static_cast<int>(width))
    return false;

  const bool has_min = std::isfinite(scan.range_min);
  const bool has_max = std::isfinite(scan.range_max);

  for (unsigned int i = 0; i < width; ++i)
  {
    const float value = scan.ranges[i];
    const bool invalid =
      !std::isfinite(value) ||
      value <= 0.f ||
      (has_min && value < scan.range_min) ||
      (has_max && value > scan.range_max);

    range_wf(i) = invalid ? 0.f : value;
  }

  return true;
}


/**
 * Performs multilevel odometry calculation from a pair of scan lasers (previous-current)
 * Odometry estimation in ROS is cumulative, that is, it adds incrementally to update the robot "odom"
 * @param scan the last laser scan received (will be compared with the previous one stored)
*/
bool CLaserOdometry2D::odometryCalculation(const sensor_msgs::msg::LaserScan& scan)
{
  //=====================================
  //	DIFERENTIAL  ODOMETRY  MULTILEVEL
  //=====================================

  if (scan.ranges.size() != width)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "Rejecting laser scan with width %zu; expected %u",
      scan.ranges.size(), width);
    return false;
  }

  current_scan_time = scan.header.stamp;
  const double time_inc_sec = (current_scan_time - last_odom_time).seconds();
  if (!std::isfinite(time_inc_sec) || time_inc_sec <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "Rejecting laser scan with invalid dt: %.9f", time_inc_sec);
    return false;
  }

  fps = static_cast<float>(1.0 / time_inc_sec);
  if (!std::isfinite(fps) || fps <= 0.f)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Rejecting laser scan with invalid fps");
    return false;
  }

  // range_wf still holds the sanitized ranges of the previous scan at this point;
  // keep them to measure how much the scan changed before they are overwritten.
  const Eigen::MatrixXf previous_range_wf = range_wf;

  if (!sanitizeScanRanges(scan))
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Rejecting laser scan that could not be sanitized");
    return false;
  }

  updateScanDifference(previous_range_wf);

  // Keep record of times
  auto start = clock_->now();

  const auto previous_range = range;
  const auto previous_xx = xx;
  const auto previous_yy = yy;
  const auto restore_previous_scan = [this, previous_range, previous_xx, previous_yy]()
  {
    range = previous_range;
    xx = previous_xx;
    yy = previous_yy;
  };

  // Create pyramid from current scan
  createImagePyramid();

  // Coarse-to-fine scheme (pyramid lvls)
  bool solved_any_level = false;
  for (unsigned int i=0; i<ctf_levels; i++)
  {
    // Clear previous computations
    transformations[i].setIdentity();

    // Keep record of current level (for other methods)
    level = i;
    unsigned int s = std::pow(2.f,int(ctf_levels-(i+1)));
    cols_i = std::ceil(float(cols)/float(s));
    image_level = ctf_levels - i + std::round(std::log2(std::round(float(width)/float(cols)))) - 1;

    // 1. Perform warping
    if (i == 0)
    {
      // No need for the first lvl
      range_warped[image_level] = range[image_level];
      xx_warped[image_level]    = xx[image_level];
      yy_warped[image_level]    = yy[image_level];
    }
    else
      performWarping();

    // 2. Calculate inter coords
    calculateCoord();

    // 3. Find null points
    findNullPoints();

    // 4. Compute derivatives
    calculaterangeDerivativesSurface();

    // 5. Compute normals
    //computeNormals();

    // 6. Compute weights
    computeWeights();

    // 7. Solve odometry
    if (num_valid_range > 3)
    {
      if (!solveSystemNonLinear())
      {
        restore_previous_scan();
        return false;
      }
      //solveSystemOneLevel();    //without robust-function
    }
    else
    {
      /// @todo At initialization something
      /// isn't properly initialized so that
      /// uninitialized values get propagated
      /// from 'filterLevelSolution' first call
      /// Throughout the whole execution. Thus
      /// this 'continue' that surprisingly works.
      continue;
    }

    // 8. Filter solution
    if (!filterLevelSolution())
    {
      restore_previous_scan();
      return false;
    }
    solved_any_level = true;
  } // end pyramid lvls

  if (!solved_any_level)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "Rejecting laser scan with insufficient valid RF2O constraints");
    restore_previous_scan();
    return false;
  }

  // Get computation time 
  auto m_runtime = clock_->now() - start;
  RCLCPP_DEBUG(logger_, "execution time (ms): %f",
                m_runtime.seconds()*double(1000));

  // Update poses with the new odom
  PoseUpdate();

  return true;
}


/**
 * Creates the Pyramid (multi-resolution) from the current laser range data
*/
void CLaserOdometry2D::createImagePyramid()
{
  // Push the scans back
  range_old.swap(range);
  xx_old.swap(xx);
  yy_old.swap(yy);

  // The number of levels of the pyramid does not always match the number of levels used
  // in the odometry computation (because we sometimes want to finish with lower resolutions)
  unsigned int pyr_levels = std::round(std::log2(std::round(float(width)/float(cols)))) + ctf_levels;

  // Generate Pyramid levels
  const float max_range_dif = 0.3f;
  for (unsigned int i = 0; i<pyr_levels; i++)
  {
    unsigned int s = std::pow(2.f,int(i));
    cols_i = std::ceil(float(width)/float(s));

    const unsigned int i_1 = i-1;

    // First level -> Filter (not downsampling);
    if (i == 0)
    {
      for (unsigned int u = 0; u < cols_i; u++)
      {
        const float dcenter = range_wf(u);

        // Inner pixels (avoid first and last points in the scan)
        if ((u>1)&&(u<cols_i-2))
        {
          if (std::isfinite(dcenter) && dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;

            for (int l=-2; l<3; l++)
            {
              const float neighbor = range_wf(u+l);
              const float abs_dif = std::abs(neighbor-dcenter);
              if (std::isfinite(neighbor) && neighbor > 0.f && abs_dif < max_range_dif)
              {
                const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                weight += aux_w;
                sum += aux_w*neighbor;
              }
            }
            range[i](u) = (weight > kEpsilon) ? sum/weight : 0.f;
          }
          else
            range[i](u) = 0.f;
        }

        // Boundary points 
        else
        {
          if (std::isfinite(dcenter) && dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;

            for (int l=-2; l<3; l++)
            {
                const int indu = u+l;
                if ((indu>=0)&&(indu<int(cols_i)))
                {
                  const float neighbor = range_wf(indu);
                  const float abs_dif = std::abs(neighbor-dcenter);
                  if (std::isfinite(neighbor) && neighbor > 0.f && abs_dif < max_range_dif)
                  {
                    const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                    weight += aux_w;
                    sum += aux_w*neighbor;
                  }
                }
              }
            range[i](u) = (weight > kEpsilon) ? sum/weight : 0.f;
          }
          else
            range[i](u) = 0.f;
        }
      }
    }

    // Second level and forth ->  Downsampling    
    else
    {
      for (unsigned int u = 0; u < cols_i; u++)
      {
        const int u2 = 2*u;
        const float dcenter = range[i_1](u2);

        // Inner pixels (avoid first/last point in the scan)
        if ((u>0)&&(u<cols_i-1))
        {
          if (dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;

            for (int l=-2; l<3; l++)
            {
              const int indu = u2+l;
              if ((indu>=0)&&(indu<int(range[i_1].cols())))
              {
                const float neighbor = range[i_1](indu);
                const float abs_dif = std::abs(neighbor-dcenter);
                if (std::isfinite(neighbor) && neighbor > 0.f && abs_dif < max_range_dif)
                {
                  const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                  weight += aux_w;
                  sum += aux_w*neighbor;
                }
              }
            }
            range[i](u) = (weight > kEpsilon) ? sum/weight : 0.f;
          }
          else
            range[i](u) = 0.f;

        }

        //Boundary
        else
        {
          if (dcenter > 0.f)
          {
            float sum = 0.f;
            float weight = 0.f;
            const unsigned int cols_i2 = range[i_1].cols();


            for (int l=-2; l<3; l++)
            {
              const int indu = u2+l;
              if ((indu>=0)&&(indu<int(cols_i2)))
              {
                  const float abs_dif = std::abs(range[i_1](indu)-dcenter);
                  const float neighbor = range[i_1](indu);
                  if (std::isfinite(neighbor) && neighbor > 0.f && abs_dif < max_range_dif)
                  {
                    const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                    weight += aux_w;
                    sum += aux_w*neighbor;
                  }
                }
              }
            range[i](u) = (weight > kEpsilon) ? sum/weight : 0.f;
          }
          else
            range[i](u) = 0.f;

        }
      }
    }

    // Calculate coordinates "xy" of the points
    const float angle_denominator = (cols_i > 1) ? float(cols_i-1) : 1.f;
    for (unsigned int u = 0; u < cols_i; u++)
    {
      if (range[i](u) > 0.f)
      {
        const float tita = -0.5*fovh + float(u)*fovh/angle_denominator;
        xx[i](u) = range[i](u)*std::cos(tita);
        yy[i](u) = range[i](u)*std::sin(tita);
      }
      else
      {
        xx[i](u) = 0.f;
        yy[i](u) = 0.f;
      }
    }
  }
}


void CLaserOdometry2D::calculateCoord()
{
  for (unsigned int u = 0; u < cols_i; u++)
  {
    if (!std::isfinite(range_old[image_level](u)) ||
        !std::isfinite(range_warped[image_level](u)) ||
        range_old[image_level](u) <= 0.f ||
        range_warped[image_level](u) <= 0.f)
    {
      range_inter[image_level](u) = 0.f;
      xx_inter[image_level](u)    = 0.f;
      yy_inter[image_level](u)    = 0.f;
    }
    else
    {
      range_inter[image_level](u) = 0.5f*(range_old[image_level](u) + range_warped[image_level](u));
      xx_inter[image_level](u)    = 0.5f*(xx_old[image_level](u)    + xx_warped[image_level](u));
      yy_inter[image_level](u)    = 0.5f*(yy_old[image_level](u)    + yy_warped[image_level](u));
    }
  }
}


void CLaserOdometry2D::calculaterangeDerivativesSurface()
{
  //The gradient size ir reserved at the maximum size (at the constructor)
  if (cols_i < 3)
  {
    dtita.setZero();
    dt.setZero();
    return;
  }

  //Compute connectivity

  //Defined in a different way now, without inversion
  rtita = Eigen::MatrixXf::Constant(1, cols_i, 1.f);

  for (unsigned int u = 0; u < cols_i-1; u++)
  {
    float dista = xx_inter[image_level](u+1) - xx_inter[image_level](u);
    dista *= dista;
    float distb = yy_inter[image_level](u+1) - yy_inter[image_level](u);
    distb *= distb;
    const float dist = dista + distb;

    if (dist  > 0.f)
      rtita(u) = std::sqrt(dist);
  }

  //Spatial derivatives
  for (unsigned int u = 1; u < cols_i-1; u++)
  {
    const float denominator = rtita(u)+rtita(u-1);
    if (std::isfinite(denominator) && denominator > kEpsilon)
    {
      dtita(u) = (rtita(u-1)*(range_inter[image_level](u+1)-
                  range_inter[image_level](u)) + rtita(u)*(range_inter[image_level](u) -
          range_inter[image_level](u-1)))/denominator;
    }
    else
      dtita(u) = 0.f;
  }

  dtita(0) = dtita(1);
  dtita(cols_i-1) = dtita(cols_i-2);

  //Temporal derivative
  for (unsigned int u = 0; u < cols_i; u++)
    dt(u) = fps*(range_warped[image_level](u) - range_old[image_level](u));


  //Apply median filter to the range derivatives
  //MatrixXf dtitamed = dtita, dtmed = dt;
  //vector<float> svector(3);
  //for (unsigned int u=1; u<cols_i-1; u++)
  //{
  //	svector.at(0) = dtita(u-1); svector.at(1) = dtita(u); svector.at(2) = dtita(u+1);
  //	std::sort(svector.begin(), svector.end());
  //	dtitamed(u) = svector.at(1);

  //	svector.at(0) = dt(u-1); svector.at(1) = dt(u); svector.at(2) = dt(u+1);
  //	std::sort(svector.begin(), svector.end());
  //	dtmed(u) = svector.at(1);
  //}

  //dtitamed(0) = dtitamed(1);
  //dtitamed(cols_i-1) = dtitamed(cols_i-2);
  //dtmed(0) = dtmed(1);
  //dtmed(cols_i-1) = dtmed(cols_i-2);

  //dtitamed.swap(dtita);
  //dtmed.swap(dt);
}

void CLaserOdometry2D::computeNormals()
{
  normx.setConstant(1, cols, 0.f);
  normy.setConstant(1, cols, 0.f);
  norm_ang.setConstant(1, cols, 0.f);

  const float incr_tita = fovh/float(cols_i-1);
  for (unsigned int u=0; u<cols_i; u++)
  {
    if (null(u) == 0.f)
    {
      const float tita = -0.5f*fovh + float(u)*incr_tita;
      const float alfa = -std::atan2(2.f*dtita(u), 2.f*range[image_level](u)*incr_tita);
      norm_ang(u) = tita + alfa;
      if (norm_ang(u) < -M_PI)
        norm_ang(u) += 2.f*M_PI;
      else if (norm_ang(u) < 0.f)
        norm_ang(u) += M_PI;
      else if (norm_ang(u) > M_PI)
        norm_ang(u) -= M_PI;

      normx(u) = std::cos(tita + alfa);
      normy(u) = std::sin(tita + alfa);
    }
  }
}

void CLaserOdometry2D::computeWeights()
{
  //The maximum weight size is reserved at the constructor
  weights.setConstant(1, cols, 0.f);

  if (!std::isfinite(fps) || fps <= kEpsilon)
  {
    num_valid_range = 0;
    return;
  }

  //Parameters for error_linearization
  const float kdtita = 1.f;
  const float kdt = kdtita / (fps*fps);
  const float k2d = 0.2f;

  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      //							Compute derivatives
      //-----------------------------------------------------------------------
      const float ini_dtita = range_old[image_level](u+1) - range_old[image_level](u-1);
      const float final_dtita = range_warped[image_level](u+1) - range_warped[image_level](u-1);

      const float dtitat = ini_dtita - final_dtita;
      const float dtita2 = dtita(u+1) - dtita(u-1);

      const float w_der = kdt*(dt(u)*dt(u)) +
          kdtita*(dtita(u)*dtita(u)) +
          k2d*(std::abs(dtitat) + std::abs(dtita2));

      if (std::isfinite(w_der) && w_der > kEpsilon)
      {
        const float weight = std::sqrt(1.f/w_der);
        weights(u) = std::isfinite(weight) ? weight : 0.f;
      }
    }

  const float max_weight = weights.maxCoeff();
  if (!std::isfinite(max_weight) || max_weight <= kEpsilon)
  {
    weights.setZero();
    num_valid_range = 0;
    return;
  }

  weights *= 1.f / max_weight;

  num_valid_range = 0;
  for (unsigned int u = 1; u < cols_i-1; u++)
  {
    if (null(u) == 0 && std::isfinite(weights(u)) && weights(u) > kEpsilon)
      num_valid_range++;
    else
      null(u) = 1;
  }
}

void CLaserOdometry2D::findNullPoints()
{
  //Size of null matrix is set to its maximum size (constructor)
  num_valid_range = 0;
  null.setConstant(1, cols, 1);

  for (unsigned int u = 1; u < cols_i-1; u++)
  {
    // A point is only valid if its local neighborhood is valid
    // This prevents huge derivative spikes at the edges of invalid regions
    bool valid_neighborhood = true;
    for (int l = -1; l <= 1; ++l)
    {
      int idx = static_cast<int>(u) + l;
      if (idx >= 0 && idx < static_cast<int>(cols_i))
      {
        if (!std::isfinite(range_inter[image_level](idx)) || range_inter[image_level](idx) <= 0.f)
        {
          valid_neighborhood = false;
          break;
        }
      }
    }

    if (!valid_neighborhood)
      null(u) = 1;
    else
    {
      num_valid_range++;
      null(u) = 0;
    }
  }
}

// Solves the system without considering any robust-function
bool CLaserOdometry2D::solveSystemOneLevel()
{
  if (num_valid_range <= 3)
    return false;

  A.resize(num_valid_range, 3);
  B.resize(num_valid_range, 1);

  unsigned int cont = 0;
  const float kdtita = (cols_i-1)/fovh;

  //Fill the matrix A and the vector B
  //The order of the variables will be (vx, vy, wz)

  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      // Precomputed expressions
      const float tw = weights(u);
      const float tita = -0.5*fovh + u/kdtita;

      //Fill the matrix A
      A(cont, 0) = tw*(std::cos(tita) + dtita(u)*kdtita*std::sin(tita)/range_inter[image_level](u));
      A(cont, 1) = tw*(std::sin(tita) - dtita(u)*kdtita*std::cos(tita)/range_inter[image_level](u));
      A(cont, 2) = tw*(-yy[image_level](u)*std::cos(tita) + xx[image_level](u)*std::sin(tita) - dtita(u)*kdtita);
      B(cont, 0) = tw*(-dt(u));

      cont++;
    }

  //Solve the linear system of equations using a minimum least squares method
  Eigen::MatrixXf AtA, AtB;
  AtA = A.transpose()*A;
  AtB = A.transpose()*B;
  Eigen::MatrixXf solution;
  Eigen::MatrixXf inv_ata;
  if (!solveNormalEquations(AtA, AtB, solution, &inv_ata))
    return false;
  Var = solution;

  //Covariance matrix calculation 	Cov Order -> vx,vy,wz
  Eigen::MatrixXf res(num_valid_range,1);
  res = A*Var - B;
  if (!res.allFinite())
    return false;

  const float cov_scale = res.squaredNorm() / float(num_valid_range-3);
  if (!std::isfinite(cov_scale))
    return false;

  cov_odo = cov_scale * inv_ata;
  if (!cov_odo.allFinite())
    return false;

  kai_loc_level_ = Var;
  return true;
}

// Solves the system by considering the Cauchy M-estimator robust-function
bool CLaserOdometry2D::solveSystemNonLinear()
{
  if (num_valid_range <= 3)
    return false;

  A.resize(num_valid_range, 3); Aw.resize(num_valid_range, 3);
  B.resize(num_valid_range, 1); Bw.resize(num_valid_range, 1);
  unsigned int cont = 0;
  const float kdtita = float(cols_i-1)/fovh;

  //Fill the matrix A and the vector B
  //The order of the variables will be (vx, vy, wz)

  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      // Precomputed expressions
      const float tw = weights(u);
      const float tita = -0.5*fovh + u/kdtita;

      //Fill the matrix A
      A(cont, 0) = tw*(std::cos(tita) + dtita(u)*kdtita*std::sin(tita)/range_inter[image_level](u));
      A(cont, 1) = tw*(std::sin(tita) - dtita(u)*kdtita*std::cos(tita)/range_inter[image_level](u));
      A(cont, 2) = tw*(-yy[image_level](u)*std::cos(tita) + xx[image_level](u)*std::sin(tita) - dtita(u)*kdtita);
      B(cont, 0) = tw*(-dt(u));

      cont++;
    }

  //Solve the linear system of equations using a minimum least squares method
  Eigen::MatrixXf AtA, AtB;
  AtA = A.transpose()*A;
  AtB = A.transpose()*B;
  Eigen::MatrixXf solution;
  Eigen::MatrixXf inv_ata;
  if (!solveNormalEquations(AtA, AtB, solution, nullptr))
    return false;
  Var = solution;

  //Covariance matrix calculation 	Cov Order -> vx,vy,wz
  Eigen::MatrixXf res(num_valid_range,1);
  res = A*Var - B;
  if (!res.allFinite())
    return false;
  //cout << endl << "max res: " << res.maxCoeff();
  //cout << endl << "min res: " << res.minCoeff();

  ////Compute the energy
  //Compute the average dt
  float aver_dt = 0.f;
  for (unsigned int u = 1; u < cols_i-1; u++)
    if (null(u) == 0)
    {
      aver_dt  += std::abs(dt(u));
    }
  if (cont == 0)
    return false;

  aver_dt /= cont;
  //    printf("\n Aver dt = %f, aver res = %f", aver_dt, aver_res);


  const float k = (std::isfinite(aver_dt) && aver_dt > kEpsilon) ? 10.f/aver_dt : 1.f; //200
  //float energy = 0.f;
  //for (unsigned int i=0; i<res.rows(); i++)
  //	energy += log(1.f + mrpt::math::square(k*res(i)));
  //printf("\n\nEnergy(0) = %f", energy);

  //Solve iterative reweighted least squares
  //===================================================================
  for (unsigned int i=1; i<=iter_irls; i++)
  {
    cont = 0;

    for (unsigned int u = 1; u < cols_i-1; u++)
      if (null(u) == 0)
      {
        const float robust_arg = k*res(cont);
        const float res_weight = std::isfinite(robust_arg) ?
          std::sqrt(1.f/(1.f + (robust_arg*robust_arg))) : 0.f;

        //Fill the matrix Aw
        Aw(cont,0) = res_weight*A(cont,0);
        Aw(cont,1) = res_weight*A(cont,1);
        Aw(cont,2) = res_weight*A(cont,2);
        Bw(cont)   = res_weight*B(cont);
        cont++;
      }

    //Solve the linear system of equations using a minimum least squares method
    AtA = Aw.transpose()*Aw;
    AtB = Aw.transpose()*Bw;
    if (!solveNormalEquations(AtA, AtB, solution, nullptr))
      return false;
    Var = solution;
    res = A*Var - B;
    if (!res.allFinite())
      return false;

    ////Compute the energy
    //energy = 0.f;
    //for (unsigned int j=0; j<res.rows(); j++)
    //	energy += log(1.f + mrpt::math::square(k*res(j)));
    //printf("\nEnergy(%d) = %f", i, energy);
  }

  if (!solveNormalEquations(AtA, AtB, solution, &inv_ata))
    return false;

  const float cov_scale = res.squaredNorm() / float(num_valid_range-3);
  if (!std::isfinite(cov_scale))
    return false;

  cov_odo = cov_scale * inv_ata;
  if (!cov_odo.allFinite())
    return false;

  kai_loc_level_ = Var;
  return true;

  //RCLCPP_INFO_STREAM(logger_, "[rf2o] COV_ODO:\n" << cov_odo);
}

void CLaserOdometry2D::Reset(const Pose3d& ini_pose/*, CObservation2DRangeScan scan*/)
{
  //Set the initial pose
  laser_pose_    = ini_pose;
  laser_oldpose_ = ini_pose;

  //readLaser(scan);
  createImagePyramid();
}

/**
 * Performs warping on the scan points of the current pyramid lvl(coarse2fine)
*/
void CLaserOdometry2D::performWarping()
{
  Eigen::Matrix3f acu_trans;

  acu_trans.setIdentity();
  for (unsigned int i=1; i<=level; i++)
    acu_trans = transformations[i-1]*acu_trans;

  Eigen::MatrixXf wacu = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

  range_warped[image_level].setConstant(1, cols_i, 0.f);

  const float cols_lim = float(cols_i-1);
  const float kdtita = cols_lim/fovh;

  for (unsigned int j = 0; j<cols_i; j++)
  {
    if (range[image_level](j) > 0.f)
    {
      //Transform point to the warped reference frame
      const float x_w = acu_trans(0,0)*xx[image_level](j) + acu_trans(0,1)*yy[image_level](j) + acu_trans(0,2);
      const float y_w = acu_trans(1,0)*xx[image_level](j) + acu_trans(1,1)*yy[image_level](j) + acu_trans(1,2);
      const float tita_w = std::atan2(y_w, x_w);
      const float range_w = std::sqrt(x_w*x_w + y_w*y_w);

      //Calculate warping
      const float uwarp = kdtita*(tita_w + 0.5*fovh);

      //The warped pixel (which is not integer in general) contributes to all the surrounding ones
      if (( uwarp >= 0.f)&&( uwarp < cols_lim))
      {
        const int uwarp_l = uwarp;
        const int uwarp_r = uwarp_l + 1;
        const float delta_r = float(uwarp_r) - uwarp;
        const float delta_l = uwarp - float(uwarp_l);

        //Very close pixel
        if (std::abs(std::round(uwarp) - uwarp) < 0.05f)
        {
          range_warped[image_level]((int)round(uwarp)) += range_w;
          wacu((int)std::round(uwarp)) += 1.f;
        }
        else
        {
          const float w_r = delta_l*delta_l;
          range_warped[image_level](uwarp_r) += w_r*range_w;
          wacu(uwarp_r) += w_r;

          const float w_l = delta_r*delta_r;
          range_warped[image_level](uwarp_l) += w_l*range_w;
          wacu(uwarp_l) += w_l;
        }
      }
    }
  }

  //Scale the averaged range and compute coordinates
  for (unsigned int u = 0; u<cols_i; u++)
  {
    if (std::isfinite(wacu(u)) && wacu(u) > kEpsilon)
    {
      const float tita = -0.5f*fovh + float(u)/kdtita;
      range_warped[image_level](u) /= wacu(u);
      xx_warped[image_level](u) = range_warped[image_level](u)*std::cos(tita);
      yy_warped[image_level](u) = range_warped[image_level](u)*std::sin(tita);
    }
    else
    {
      range_warped[image_level](u) = 0.f;
      xx_warped[image_level](u) = 0.f;
      yy_warped[image_level](u) = 0.f;
    }
  }
}

bool CLaserOdometry2D::filterLevelSolution()
{
  if (!cov_odo.allFinite() || !kai_loc_level_.allFinite())
  {
    RCLCPP_WARN(logger_, "Non-finite RF2O solution. Pose is not updated");
    return false;
  }

  //		Calculate Eigenvalues and Eigenvectors
  //----------------------------------------------------------
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(cov_odo);
  if (eigensolver.info() != Eigen::Success)
  {
    RCLCPP_WARN(logger_, "WARNING: Eigensolver couldn't find a solution. Pose is not updated");
    return false;
  }

  //First, we have to describe both the new linear and angular speeds in the "eigenvector" basis
  //-------------------------------------------------------------------------------------------------
  Eigen::Matrix<float,3,3> Bii;
  Eigen::Matrix<float,3,1> kai_b;
  Bii = eigensolver.eigenvectors();
  if (!Bii.allFinite() || !eigensolver.eigenvalues().allFinite())
  {
    RCLCPP_WARN(logger_, "Non-finite covariance eigensystem. Pose is not updated");
    return false;
  }

  kai_b = Bii.transpose()*kai_loc_level_;

  //Second, we have to describe both the old linear and angular speeds in the "eigenvector" basis too
  //-------------------------------------------------------------------------------------------------
  MatrixS31 kai_loc_sub;

  //Important: we have to substract the solutions from previous levels
  Eigen::Matrix3f acu_trans;
  acu_trans.setIdentity();
  for (unsigned int i=0; i<level; i++)
    acu_trans = transformations[i]*acu_trans;

  kai_loc_sub(0) = -fps*acu_trans(0,2);
  kai_loc_sub(1) = -fps*acu_trans(1,2);
  if (acu_trans(0,0) > 1.f)
    kai_loc_sub(2) = 0.f;
  else
  {
    kai_loc_sub(2) = -fps*std::acos(acu_trans(0,0))*rf2o::sign(acu_trans(1,0));
  }
  kai_loc_sub += kai_loc_old_;

  if (!kai_loc_sub.allFinite())
    return false;

  Eigen::Matrix<float,3,1> kai_b_old;
  kai_b_old = Bii.transpose()*kai_loc_sub;

  //Filter speed
  const float cf = 15e3f*std::exp(-float(int(level))),
              df = 0.05f*std::exp(-float(int(level)));

  Eigen::Matrix<float,3,1> kai_b_fil;
  for (unsigned int i=0; i<3; i++)
  {
    kai_b_fil(i) = (kai_b(i) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i))/(1.f + cf*eigensolver.eigenvalues()(i,0) + df);
    //kai_b_fil_f(i,0) = (1.f*kai_b(i,0) + 0.f*kai_b_old_f(i,0))/(1.0f + 0.f);
  }

  //Transform filtered speed to local reference frame and compute transformation
  Eigen::Matrix<float, 3, 1> kai_loc_fil = Bii*kai_b_fil;
  if (!kai_loc_fil.allFinite())
    return false;

  //transformation
  const float incrx = kai_loc_fil(0)/fps;
  const float incry = kai_loc_fil(1)/fps;
  const float rot   = kai_loc_fil(2)/fps;

  transformations[level](0,0) = std::cos(rot);
  transformations[level](0,1) = -std::sin(rot);
  transformations[level](1,0) = std::sin(rot);
  transformations[level](1,1) = std::cos(rot);
  transformations[level](0,2) = incrx;
  transformations[level](1,2) = incry;

  return true;
}


/**
 * Measures how much the scan changed with respect to the previous one.
 * Two consecutive scans from a static sensor are near-identical, so the mean
 * absolute range difference over the beams that are valid in both is a motion
 * cue that is independent of the scan-matching solution.
*/
void CLaserOdometry2D::updateScanDifference(const Eigen::MatrixXf& previous_range_wf)
{
  zv_scan_diff_       = 0.0;
  zv_scan_diff_valid_ = false;

  if (previous_range_wf.size() != range_wf.size())
    return;

  double sum = 0.0;
  unsigned int count = 0;

  for (int i = 0; i < range_wf.size(); ++i)
  {
    // sanitizeScanRanges() zeroes out the beams it rejected.
    if (previous_range_wf(i) <= 0.f || range_wf(i) <= 0.f)
      continue;

    sum += std::abs(double(range_wf(i)) - double(previous_range_wf(i)));
    ++count;
  }

  // Too few shared beams to say anything; leave the result invalid so that the
  // gate falls back to refusing a stationary decision.
  if (count < 10)
    return;

  zv_scan_diff_       = sum / double(count);
  zv_scan_diff_valid_ = std::isfinite(zv_scan_diff_);
}

/**
 * Decides whether the scan pair that was just solved indicates no motion.
 *
 * Both edges are held: entering needs zv_hold_scans consecutive still scans so
 * that a single quiet pair cannot latch the gate, and leaving needs
 * zv_release_scans consecutive moving ones so that an isolated noise spike just
 * over a threshold cannot unlatch it. The release count is the smaller of the
 * two, because while it counts down a genuine manoeuvre is being suppressed.
*/
bool CLaserOdometry2D::updateZeroVelocityState()
{
  if (!zv_enabled)
  {
    zv_stationary_   = false;
    zv_still_count_  = 0;
    zv_moving_count_ = 0;
    return false;
  }

  const double solved_linear  = std::hypot(double(kai_loc_(0)), double(kai_loc_(1)));
  const double solved_angular = std::abs(double(kai_loc_(2)));

  bool still =
    std::isfinite(solved_linear) && std::isfinite(solved_angular) &&
    solved_linear  < zv_linear_threshold &&
    solved_angular < zv_angular_threshold;

  // Cross-check the solver against the raw scans.
  if (still && zv_scan_diff_threshold > 0.0)
    still = zv_scan_diff_valid_ && (zv_scan_diff_ < zv_scan_diff_threshold);

  // Optional external signal. It can only veto a scan-derived decision, never
  // produce one on its own.
  if (still)
    still = zv_external_still;

  if (still)
  {
    ++zv_still_count_;
    zv_moving_count_ = 0;
  }
  else
  {
    ++zv_moving_count_;
    zv_still_count_ = 0;
  }

  const int hold    = std::max(1, zv_hold_scans);
  const int release = std::max(1, zv_release_scans);

  if (!zv_stationary_ && zv_still_count_ >= hold)
  {
    zv_stationary_ = true;
    RCLCPP_INFO(logger_, "Zero-velocity detected; holding pose (scan diff %.4f m)", zv_scan_diff_);
  }
  else if (zv_stationary_ && zv_moving_count_ >= release)
  {
    zv_stationary_ = false;
    RCLCPP_INFO(logger_, "Motion detected; resuming pose integration "
                         "(v %.4f m/s, w %.4f rad/s, scan diff %.4f m)",
                solved_linear, solved_angular, zv_scan_diff_);
  }

  return zv_stationary_;
}

/**
 * Updates the laser and robot poses after analyzing the last scan
 * To do so, we need to analyze the coarse2fine pyramid
*/
void CLaserOdometry2D::PoseUpdate()
{
  const double time_inc_sec = 1.0 / static_cast<double>(fps);
  if (!std::isfinite(time_inc_sec) || time_inc_sec <= 0.0)
  {
    RCLCPP_WARN(logger_, "Invalid scan period. Pose is not updated");
    return;
  }

  // First, compute the overall transformation
  Eigen::Matrix3f acu_trans;
  acu_trans.setIdentity();

  for (unsigned int i=1; i<=ctf_levels; i++)
    acu_trans = transformations[i-1]*acu_trans;

  //				Compute kai_loc and kai_abs
  //--------------------------------------------------------
  kai_loc_(0) = fps*acu_trans(0,2);
  kai_loc_(1) = fps*acu_trans(1,2);

  if (acu_trans(0,0) > 1.f)
    kai_loc_(2) = 0.f;
  else
  {
    kai_loc_(2) = fps*std::acos(acu_trans(0,0))*rf2o::sign(acu_trans(1,0));
  }

  //cout << endl << "Arc cos (incr tita): " << kai_loc_(2);

  //			Zero-velocity gate
  //--------------------------------------------------------
  // kai_loc_ is the solved velocity for this scan pair. When it is only noise
  // the pose is held instead of integrated, otherwise that noise accumulates as
  // a random walk. Time still advances so that the node keeps publishing at its
  // normal rate with a zero twist.
  if (updateZeroVelocityState())
  {
    laser_oldpose_  = laser_pose_;
    robot_oldpose_  = robot_pose_;
    last_increment_ = Pose3d::Identity();

    kai_loc_.setZero();
    kai_abs_.setZero();
    kai_loc_old_.setZero();

    lin_speed   = 0.0;
    lin_speed_y = 0.0;
    ang_speed   = 0.0;

    last_odom_time = current_scan_time;

    RCLCPP_DEBUG(logger_, "Stationary; pose held at [x,y,yaw]=[%f %f %f]",
                 robot_pose_.translation()(0),
                 robot_pose_.translation()(1),
                 rf2o::getYaw(robot_pose_.rotation()));
    return;
  }

  float phi = rf2o::getYaw(laser_pose_.rotation());

  kai_abs_(0) = kai_loc_(0)*std::cos(phi) - kai_loc_(1)*std::sin(phi);
  kai_abs_(1) = kai_loc_(0)*std::sin(phi) + kai_loc_(1)*std::cos(phi);
  kai_abs_(2) = kai_loc_(2);


  //						Update poses
  //-------------------------------------------------------
  laser_oldpose_ = laser_pose_;

  //  Eigen::Matrix3f aux_acu = acu_trans;
  Pose3d pose_aux_2D = Pose3d::Identity();

  pose_aux_2D = rf2o::matrixYaw(double(kai_loc_(2)/fps));
  pose_aux_2D.translation()(0) = acu_trans(0,2);
  pose_aux_2D.translation()(1) = acu_trans(1,2);

  laser_pose_ = laser_pose_ * pose_aux_2D;

  last_increment_ = pose_aux_2D;

  //				Compute kai_loc_old
  //-------------------------------------------------------
  phi = rf2o::getYaw(laser_pose_.rotation());
  kai_loc_old_(0) =  kai_abs_(0)*std::cos(phi) + kai_abs_(1)*std::sin(phi);
  kai_loc_old_(1) = -kai_abs_(0)*std::sin(phi) + kai_abs_(1)*std::cos(phi);
  kai_loc_old_(2) =  kai_abs_(2);

  RCLCPP_DEBUG(logger_, "Laser odom [x,y,yaw]=[%f %f %f]",
                laser_pose_.translation()(0),
                laser_pose_.translation()(1),
                rf2o::getYaw(laser_pose_.rotation()));

  // Compose Transformations (robot odom)
  robot_pose_ = laser_pose_ * laser_pose_on_robot_inv_;

  RCLCPP_DEBUG(logger_, "Robot-base odom [x,y,yaw]=[%f %f %f]",
                robot_pose_.translation()(0),
                robot_pose_.translation()(1),
                rf2o::getYaw(robot_pose_.rotation()));

  // Estimate linear/angular speeds (mandatory for base_local_planner)
  // last_scan -> the last scan received
  // last_odom_time -> The time of the previous scan lasser used to estimate the pose
  //-------------------------------------------------------------------------------------
  const Pose3d robot_delta = robot_oldpose_.inverse() * robot_pose_;
  lin_speed = robot_delta.translation().x() / time_inc_sec;
  lin_speed_y = robot_delta.translation().y() / time_inc_sec;
  ang_speed = rf2o::getYaw(robot_delta.rotation()) / time_inc_sec;
  robot_oldpose_ = robot_pose_;
  last_odom_time = current_scan_time;

  //filter speeds
  /*
    last_m_lin_speeds.push_back(lin_speed);
    if (last_m_lin_speeds.size()>4)
        last_m_lin_speeds.erase(last_m_lin_speeds.begin());
    double sum = std::accumulate(last_m_lin_speeds.begin(), last_m_lin_speeds.end(), 0.0);
    lin_speed = sum / last_m_lin_speeds.size();

    last_m_ang_speeds.push_back(ang_speed);
    if (last_m_ang_speeds.size()>4)
        last_m_ang_speeds.erase(last_m_ang_speeds.begin());
    double sum2 = std::accumulate(last_m_ang_speeds.begin(), last_m_ang_speeds.end(), 0.0);
    ang_speed = sum2 / last_m_ang_speeds.size();
    */
}

} /* namespace rf2o */
