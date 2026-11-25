/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  debug_cloud_topic_ = this->declare_parameter("debug_cloud_topic", "/cloud_for_scan");
  projected_debug_cloud_topic_ = this->declare_parameter(
    "projected_debug_cloud_topic", "/cloud_projected_for_scan");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  use_latest_transform_ = this->declare_parameter("use_latest_transform", true);
  output_now_stamp_ = this->declare_parameter("output_now_stamp", use_latest_transform_);
  publish_debug_cloud_ = this->declare_parameter("publish_debug_cloud", true);
  publish_projected_debug_cloud_ = this->declare_parameter("publish_projected_debug_cloud", true);
  remove_tilt_ = this->declare_parameter("remove_tilt", false);
  height_relative_to_sensor_ = this->declare_parameter("height_relative_to_sensor", true);
  near_sensor_exclusion_radius_ = this->declare_parameter("near_sensor_exclusion_radius", 0.45);
  near_sensor_exclusion_min_z_ = this->declare_parameter("near_sensor_exclusion_min_z", 0.12);
  effective_range_max_ = this->declare_parameter("effective_range_max", 8.0);
  keep_farthest_per_angle_ = this->declare_parameter("keep_farthest_per_angle", false);
  scan_yaw_offset_rad_ = this->declare_parameter("scan_yaw_offset_rad", 0.0);
  max_input_age_sec_ = this->declare_parameter("max_input_age_sec", 1.0);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  // input_queue_size_ = this->declare_parameter(
  //   "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  input_queue_size_ = this->declare_parameter(
    "queue_size", 10);
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 10.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.reliable();
  // qos.best_effort(); 
   qos.durability_volatile();

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
  pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(debug_cloud_topic_, qos);
  pub3_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(projected_debug_cloud_topic_, qos);

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    if (use_latest_transform_) {
      sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    } else {
      message_filter_ = std::make_unique<MessageFilter>(
        sub_, *tf2_, target_frame_, input_queue_size_,
        this->get_node_logging_interface(),
        this->get_node_clock_interface());
      message_filter_->registerCallback(
        std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    }
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    // int subscription_count = pub_->get_subscription_count() +
    //   pub_->get_intra_process_subscription_count();
    // if (subscription_count > 0) {
    //   if (!sub_.getSubscriber()) {
    //     RCLCPP_INFO(
    //       this->get_logger(),
    //       "Got a subscriber to laserscan, starting pointcloud subscriber");
    //     rclcpp::SensorDataQoS qos;
    //     qos.keep_last(input_queue_size_);
    //     qos.reliable();
    //     sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
    //   }
    // } else if (sub_.getSubscriber()) {
    //   RCLCPP_INFO(
    //     this->get_logger(),
    //     "No subscribers to laserscan, shutting down pointcloud subscriber");
    //   sub_.unsubscribe();
    // }

    if (!sub_.getSubscriber()) {
      rclcpp::SensorDataQoS qos;
      qos.keep_last(input_queue_size_);
      qos.reliable();
      sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  const auto output_stamp = output_now_stamp_ ? this->now() : rclcpp::Time(cloud_msg->header.stamp);

  // In use_latest_transform mode, processing very old clouds as if they were current causes
  // visible jumps in RViz/SLAM when the pipeline falls behind. Drop stale frames instead.
  if (use_latest_transform_ && max_input_age_sec_ > 0.0) {
    const double cloud_age = (this->now() - rclcpp::Time(cloud_msg->header.stamp)).seconds();
    if (cloud_age > max_input_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Dropping stale input cloud %.3f s old to keep scan/TF temporally consistent", cloud_age);
      return;
    }
  }

  geometry_msgs::msg::TransformStamped sensor_transform;
  bool has_sensor_transform = false;
  double sensor_origin_z = 0.0;
  tf2::Vector3 sensor_plane_normal(0.0, 0.0, 1.0);
  tf2::Vector3 sensor_origin(0.0, 0.0, 0.0);
  auto height_reference_cloud = cloud_msg;

  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  scan_msg->header.stamp = output_stamp;
  
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  std::vector<std::array<float, 3>> projected_scan_points(
    ranges_size,
    {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()
    });
  std::vector<bool> has_range_value(ranges_size, false);

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

      if (use_latest_transform_) {
        const auto cloud_age = (this->now() - rclcpp::Time(cloud_msg->header.stamp)).seconds();
        if (cloud_age > tolerance_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Incoming cloud stamp is %.3f s old; using latest available transform from '%s' to '%s'",
            cloud_age, cloud_msg->header.frame_id.c_str(), target_frame_.c_str());
        }

        const auto transform = tf2_->lookupTransform(
          target_frame_, cloud_msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tolerance_));
        sensor_transform = transform;
        has_sensor_transform = true;
        sensor_origin_z = transform.transform.translation.z;
        sensor_origin = tf2::Vector3(
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z);
        tf2::Quaternion sensor_rotation(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w);
        sensor_rotation.normalize();
        sensor_plane_normal = tf2::quatRotate(sensor_rotation, tf2::Vector3(0.0, 0.0, 1.0)).normalized();
        tf2::doTransform(*cloud_msg, *cloud, transform);
        cloud->header.frame_id = target_frame_;
        cloud->header.stamp = output_stamp;
      } else {
        sensor_transform = tf2_->lookupTransform(
          target_frame_, cloud_msg->header.frame_id, rclcpp::Time(cloud_msg->header.stamp),
          tf2::durationFromSec(tolerance_));
        has_sensor_transform = true;
        sensor_origin_z = sensor_transform.transform.translation.z;
        sensor_origin = tf2::Vector3(
          sensor_transform.transform.translation.x,
          sensor_transform.transform.translation.y,
          sensor_transform.transform.translation.z);
        tf2::Quaternion sensor_rotation(
          sensor_transform.transform.rotation.x,
          sensor_transform.transform.rotation.y,
          sensor_transform.transform.rotation.z,
          sensor_transform.transform.rotation.w);
        sensor_rotation.normalize();
        sensor_plane_normal = tf2::quatRotate(sensor_rotation, tf2::Vector3(0.0, 0.0, 1.0)).normalized();
        tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      }

      cloud_msg = cloud;
      height_reference_cloud = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  height_reference_cloud = cloud_msg;

  if (remove_tilt_ && has_sensor_transform) {
    auto leveled_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud_msg);
    removeTilt(*leveled_cloud, sensor_transform);
    cloud_msg = leveled_cloud;
  }

  if (publish_debug_cloud_ && pub2_) {
    auto debug_cloud_msg = *cloud_msg;
    debug_cloud_msg.header.stamp = output_stamp;
    pub2_->publish(debug_cloud_msg);
  }

  size_t total_points = 0;
  size_t rejected_nan = 0;
  size_t rejected_height = 0;
  size_t rejected_near_sensor = 0;
  size_t rejected_range_min = 0;
  size_t rejected_range_max = 0;
  size_t rejected_effective_range = 0;
  size_t rejected_angle = 0;
  size_t accepted_new_or_selected = 0;
  size_t dropped_by_angle_competition = 0;

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z"),
    iter_ref_x(*height_reference_cloud, "x"),
    iter_ref_y(*height_reference_cloud, "y"),
    iter_ref_z(*height_reference_cloud, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_ref_x, ++iter_ref_y, ++iter_ref_z)
  {
    ++total_points;
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      ++rejected_nan;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    double point_height = *iter_ref_z;
    if (height_relative_to_sensor_ && has_sensor_transform) {
      const tf2::Vector3 reference_point(*iter_ref_x, *iter_ref_y, *iter_ref_z);
      point_height = sensor_plane_normal.dot(reference_point - sensor_origin);
    } else if (has_sensor_transform) {
      point_height = *iter_ref_z - sensor_origin_z;
    }

    if (point_height > max_height_ || point_height < min_height_) {
      ++rejected_height;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        point_height, min_height_, max_height_);
      continue;
    }

    const double near_sensor_distance = std::hypot(
      *iter_ref_x - sensor_origin.x(), *iter_ref_y - sensor_origin.y());
    if (has_sensor_transform && near_sensor_exclusion_radius_ > 0.0 &&
      near_sensor_distance < near_sensor_exclusion_radius_ &&
      point_height > near_sensor_exclusion_min_z_)
    {
      ++rejected_near_sensor;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for near-sensor exclusion distance %f below %f with relative height %f above %f\n",
        near_sensor_distance, near_sensor_exclusion_radius_, point_height,
        near_sensor_exclusion_min_z_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      ++rejected_range_min;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      ++rejected_range_max;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (effective_range_max_ > 0.0 && range > effective_range_max_) {
      ++rejected_effective_range;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for effective range %f above limit %f. Point: (%f, %f, %f)",
        range, effective_range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x) + scan_yaw_offset_rad_;
    angle = atan2(sin(angle), cos(angle));
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      ++rejected_angle;
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // LaserScan stores one value per angle bin. Choose either nearest or farthest by configuration.
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    bool should_update = false;
    if (!has_range_value[index]) {
      should_update = true;
    } else if (keep_farthest_per_angle_) {
      should_update = range > scan_msg->ranges[index];
    } else {
      should_update = range < scan_msg->ranges[index];
    }

    if (should_update) {
      ++accepted_new_or_selected;
      has_range_value[index] = true;
      scan_msg->ranges[index] = range;
      projected_scan_points[index] = {
        static_cast<float>(*iter_x),
        static_cast<float>(*iter_y),
        0.0f
      };
    } else {
      ++dropped_by_angle_competition;
    }
  }

  size_t valid_scan_bins = 0;
  for (const auto & point : projected_scan_points) {
    if (std::isfinite(point[0]) && std::isfinite(point[1])) {
      ++valid_scan_bins;
    }
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "scan pipeline stats: total=%zu, nan=%zu, height=%zu, near=%zu, range_min=%zu, range_max=%zu, effective_range=%zu, angle=%zu, keep_farthest=%s, kept_selected=%zu, dropped_angle_competition=%zu, valid_bins=%zu",
    total_points, rejected_nan, rejected_height, rejected_near_sensor,
    rejected_range_min, rejected_range_max, rejected_effective_range,
    rejected_angle, keep_farthest_per_angle_ ? "true" : "false",
    accepted_new_or_selected, dropped_by_angle_competition, valid_scan_bins);

  if (publish_projected_debug_cloud_ && pub3_) {
    sensor_msgs::msg::PointCloud2 projected_cloud_msg;
    projected_cloud_msg.header = scan_msg->header;
    projected_cloud_msg.header.stamp = output_stamp;

    sensor_msgs::PointCloud2Modifier modifier(projected_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    modifier.resize(valid_scan_bins);
    sensor_msgs::PointCloud2Iterator<float> iter_proj_x(projected_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_proj_y(projected_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_proj_z(projected_cloud_msg, "z");

    for (const auto & point : projected_scan_points) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
        continue;
      }
      *iter_proj_x = point[0];
      *iter_proj_y = point[1];
      *iter_proj_z = point[2];
      ++iter_proj_x;
      ++iter_proj_y;
      ++iter_proj_z;
    }

    pub3_->publish(projected_cloud_msg);
  }

  pub_->publish(std::move(scan_msg));
}

void PointCloudToLaserScanNode::removeTilt(
  sensor_msgs::msg::PointCloud2 & cloud_msg,
  const geometry_msgs::msg::TransformStamped & transform) const
{
  const auto & rotation = transform.transform.rotation;
  tf2::Quaternion sensor_rotation(rotation.x, rotation.y, rotation.z, rotation.w);
  sensor_rotation.normalize();

  const tf2::Vector3 target_up(0.0, 0.0, 1.0);
  const tf2::Vector3 sensor_up = tf2::quatRotate(sensor_rotation, target_up).normalized();
  const tf2::Vector3 rotation_axis = sensor_up.cross(target_up);
  tf2::Quaternion correction_rotation;

  if (rotation_axis.length2() < 1e-12) {
    if (sensor_up.dot(target_up) < 0.0) {
      correction_rotation.setRotation(tf2::Vector3(0.0, 1.0, 0.0), M_PI);
    } else {
      correction_rotation.setValue(0.0, 0.0, 0.0, 1.0);
    }
  } else {
    const double angle = std::atan2(rotation_axis.length(), sensor_up.dot(target_up));
    correction_rotation.setRotation(rotation_axis.normalized(), angle);
  }

  correction_rotation.normalize();
  const tf2::Matrix3x3 correction(correction_rotation);
  const tf2::Vector3 sensor_origin(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);

  for (sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x"),
    iter_y(cloud_msg, "y"), iter_z(cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      continue;
    }

    const tf2::Vector3 original_point(*iter_x, *iter_y, *iter_z);
    const tf2::Vector3 leveled_point = sensor_origin + correction * (original_point - sensor_origin);
    *iter_x = static_cast<float>(leveled_point.x());
    *iter_y = static_cast<float>(leveled_point.y());
    *iter_z = static_cast<float>(leveled_point.z());
  }
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
