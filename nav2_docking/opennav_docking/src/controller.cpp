// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "opennav_docking/controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "tf2/utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_util/line_iterator.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

Controller::Controller(
  const nav2::LifecycleNode::SharedPtr & node, nav2::TransformBuffer::SharedPtr tf,
  std::string base_frame)
: tf2_buffer_(tf), base_frame_(base_frame)
{
  logger_ = node->get_logger();

  std::string costmap_topic, footprint_topic;
  k_phi_ = node->declare_or_get_parameter("controller.k_phi", 3.0);
  k_delta_ = node->declare_or_get_parameter("controller.k_delta", 2.0);
  beta_ = node->declare_or_get_parameter("controller.beta", 0.4);
  lambda_ = node->declare_or_get_parameter("controller.lambda", 2.0);
  v_linear_min_ = node->declare_or_get_parameter("controller.v_linear_min", 0.1);
  v_linear_max_ = node->declare_or_get_parameter("controller.v_linear_max", 0.25);
  v_angular_max_ = node->declare_or_get_parameter("controller.v_angular_max", 0.75);
  slowdown_radius_ = node->declare_or_get_parameter("controller.slowdown_radius", 0.25);
  deceleration_max_ = node->declare_or_get_parameter("controller.deceleration_max", 2.5);
  rotate_to_heading_angular_vel_ = node->declare_or_get_parameter(
    "controller.rotate_to_heading_angular_vel", 1.0);
  rotate_to_heading_max_angular_accel_ = node->declare_or_get_parameter(
    "controller.rotate_to_heading_max_angular_accel", 3.2);
  use_collision_detection_ = node->declare_or_get_parameter(
    "controller.use_collision_detection", true);
  costmap_topic = node->declare_or_get_parameter("controller.costmap_topic",
    std::string("local_costmap/costmap_raw"));
  footprint_topic = node->declare_or_get_parameter("controller.footprint_topic",
    std::string("local_costmap/published_footprint"));
  transform_tolerance_ = node->declare_or_get_parameter(
    "controller.transform_tolerance", 0.1);
  projection_time_ = node->declare_or_get_parameter(
    "controller.projection_time", 5.0);
  simulation_time_step_ = node->declare_or_get_parameter(
    "controller.simulation_time_step", 0.1);
  dock_collision_threshold_ = node->declare_or_get_parameter(
    "controller.dock_collision_threshold", 0.3);
  start_collision_exemption_ = node->declare_or_get_parameter(
    "controller.start_collision_exemption", 0.0);

  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, deceleration_max_,
    v_linear_min_, v_linear_max_, v_angular_max_);

  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &Controller::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &Controller::validateParameterUpdatesCallback,
      this, std::placeholders::_1));

  if (use_collision_detection_) {
    configureCollisionChecker(node, costmap_topic, footprint_topic, transform_tolerance_);
  }

  trajectory_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("docking_trajectory");
}

Controller::~Controller()
{
  control_law_.reset();
  trajectory_pub_.reset();
  collision_checker_.reset();
  costmap_sub_.reset();
  footprint_sub_.reset();
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd,
  const geometry_msgs::msg::TransformStamped & base_to_fixed_transform,
  bool is_docking, bool backward)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  cmd = control_law_->calculateRegularVelocity(pose, backward);
  return isTrajectoryCollisionFree(pose, base_to_fixed_transform, is_docking, backward);
}

geometry_msgs::msg::Twist Controller::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::Twist & current_velocity,
  const double & dt)
{
  geometry_msgs::msg::Twist cmd_vel;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * rotate_to_heading_angular_vel_;
  const double min_feasible_angular_speed =
    current_velocity.angular.z - rotate_to_heading_max_angular_accel_ * dt;
  const double max_feasible_angular_speed =
    current_velocity.angular.z + rotate_to_heading_max_angular_accel_ * dt;
  cmd_vel.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop =
    std::sqrt(2 * rotate_to_heading_max_angular_accel_ * fabs(angular_distance_to_heading));
  if (fabs(cmd_vel.angular.z) > max_vel_to_stop) {
    cmd_vel.angular.z = sign * max_vel_to_stop;
  }

  return cmd_vel;
}

bool Controller::isContactAhead(const geometry_msgs::msg::Pose & start, const double direction)
{
  if (start_collision_exemption_ <= 0.0) {
    return true;  // Nothing to exempt
  }

  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header header;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception &) {
    return true;
  }
  if (!footprint_sub_->getFootprintInRobotFrame(footprint, header) || footprint.empty()) {
    return true;
  }

  // Sweep the footprint outline, as the collision checker sees it, over the exempted
  // distance: any lethal cell it meets ahead of the robot's origin counts as ahead
  const double yaw = tf2::getYaw(start.orientation);
  const double step = costmap->getResolution() / 2.0;
  for (double s = 0.0; s <= start_collision_exemption_ + step; s += step) {
    const double px = start.position.x + direction * s * cos(yaw);
    const double py = start.position.y + direction * s * sin(yaw);
    for (size_t i = 0; i < footprint.size(); ++i) {
      const auto & a = footprint[i];
      const auto & b = footprint[(i + 1) % footprint.size()];
      unsigned int x0, y0, x1, y1;
      if (!costmap->worldToMap(
          px + cos(yaw) * a.x - sin(yaw) * a.y, py + sin(yaw) * a.x + cos(yaw) * a.y, x0, y0) ||
        !costmap->worldToMap(
          px + cos(yaw) * b.x - sin(yaw) * b.y, py + sin(yaw) * b.x + cos(yaw) * b.y, x1, y1))
      {
        return true;
      }
      for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
        if (costmap->getCost(line.getX(), line.getY()) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }
        double wx, wy;
        costmap->mapToWorld(line.getX(), line.getY(), wx, wy);
        const double ahead =
          cos(yaw) * (wx - start.position.x) + sin(yaw) * (wy - start.position.y);
        if (ahead * direction > 0.0) {
          return true;
        }
      }
    }
  }
  return false;
}

bool Controller::isTrajectoryCollisionFree(
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::TransformStamped & base_to_fixed_transform,
  bool is_docking, bool backward)
{
  // Visualization of the trajectory
  auto trajectory = std::make_unique<nav_msgs::msg::Path>();
  trajectory->header.frame_id = base_frame_;
  trajectory->header.stamp = base_to_fixed_transform.header.stamp;

  // First pose
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = base_frame_;
  trajectory->poses.push_back(next_pose);

  // Obstacles already touching the robot don't block it from moving away from them
  double exemption = 0.0;
  if (use_collision_detection_) {
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = base_frame_;
    start.pose.orientation.w = 1.0;
    tf2::doTransform(start, start, base_to_fixed_transform);
    exemption = isContactAhead(start.pose, backward ? -1.0 : 1.0) ?
      0.0 : start_collision_exemption_;
  }

  // Generate path
  double distance = std::numeric_limits<double>::max();
  unsigned int max_iter = static_cast<unsigned int>(ceil(projection_time_ / simulation_time_step_));

  bool fetch_costmap = true;
  do{
    // Apply velocities to calculate next pose
    next_pose.pose = control_law_->calculateNextPose(
      simulation_time_step_, target_pose, next_pose.pose, backward);

    // Add the pose to the trajectory for visualization
    trajectory->poses.push_back(next_pose);

    // Transform pose from base_frame into fixed_frame
    geometry_msgs::msg::PoseStamped local_pose = next_pose;
    local_pose.header.stamp = trajectory->header.stamp;
    tf2::doTransform(local_pose, local_pose, base_to_fixed_transform);

    // Determine the distance at which to check for collisions
    // Skip the final segment of the trajectory for docking
    // and the initial segment for undocking
    // This avoids false positives when the robot is at the dock
    double dock_collision_distance = is_docking ?
      nav2_util::geometry_utils::euclidean_distance(target_pose, next_pose.pose) :
      std::hypot(next_pose.pose.position.x, next_pose.pose.position.y);

    try {
      // If this distance is greater than the dock_collision_threshold, check for collisions,
      // except where the robot already is
      if (use_collision_detection_ &&
        dock_collision_distance > dock_collision_threshold_ &&
        std::hypot(next_pose.pose.position.x, next_pose.pose.position.y) > exemption)
      {
        if(auto score = collision_checker_->scorePose(local_pose.pose, fetch_costmap); score == nav2_costmap_2d::LETHAL_OBSTACLE) {
          RCLCPP_WARN(
            logger_, "Collision (score: %d) detected at distance %.2f and pose: (%.2f, %.2f, %.2f) in frame %s",
            (int)score, dock_collision_distance, local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z,
            local_pose.header.frame_id.c_str());
          trajectory_pub_->publish(std::move(trajectory));
          return false;
        }
        fetch_costmap = false;
      }
    } catch (const nav2_costmap_2d::IllegalPoseException & e) {
      RCLCPP_ERROR(logger_, "%s", e.what());
      return false;
    } catch (const nav2_costmap_2d::CollisionCheckerException & e) {
      RCLCPP_ERROR(logger_, "%s", e.what());
      return false;
    } catch (...) {
      RCLCPP_ERROR(logger_, "Failed to check pose score!");
      return false;
    }

    // Check if we reach the goal
    distance = nav2_util::geometry_utils::euclidean_distance(target_pose, next_pose.pose);
  } while(distance > 1e-2 && trajectory->poses.size() < max_iter);

  trajectory_pub_->publish(std::move(trajectory));

  return true;
}

void Controller::configureCollisionChecker(
  const nav2::LifecycleNode::SharedPtr & node,
  std::string costmap_topic, std::string footprint_topic, double transform_tolerance)
{
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    node, footprint_topic, *tf2_buffer_, base_frame_, transform_tolerance);
  collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, node->get_name());
}

rcl_interfaces::msg::SetParametersResult Controller::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find("controller.") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    }
  }
  return result;
}

void
Controller::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find("controller.") != 0) {
      continue;
    }
    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "controller.k_phi") {
        k_phi_ = parameter.as_double();
      } else if (param_name == "controller.k_delta") {
        k_delta_ = parameter.as_double();
      } else if (param_name == "controller.beta") {
        beta_ = parameter.as_double();
      } else if (param_name == "controller.lambda") {
        lambda_ = parameter.as_double();
      } else if (param_name == "controller.v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (param_name == "controller.v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (param_name == "controller.v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (param_name == "controller.slowdown_radius") {
        slowdown_radius_ = parameter.as_double();
      } else if (param_name == "controller.deceleration_max") {
        deceleration_max_ = parameter.as_double();
      } else if (param_name == "controller.rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (param_name == "controller.rotate_to_heading_max_angular_accel") {
        rotate_to_heading_max_angular_accel_ = parameter.as_double();
      } else if (param_name == "controller.projection_time") {
        projection_time_ = parameter.as_double();
      } else if (param_name == "controller.simulation_time_step") {
        simulation_time_step_ = parameter.as_double();
      } else if (param_name == "controller.dock_collision_threshold") {
        dock_collision_threshold_ = parameter.as_double();
      } else if (param_name == "controller.start_collision_exemption") {
        start_collision_exemption_ = parameter.as_double();
      }

      // Update the smooth control law with the new params
      control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
      control_law_->setSlowdownRadius(slowdown_radius_);
      control_law_->setMaxDeceleration(deceleration_max_);
      control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
    }
  }
}

}  // namespace opennav_docking
