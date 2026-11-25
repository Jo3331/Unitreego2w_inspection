#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "unitree_go/msg/lidar_state.hpp"

class UtlidarTimeSync : public rclcpp::Node
{
public:
  UtlidarTimeSync()
  : Node("utlidar_time_sync")
  {
    const auto subscription_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    const auto publication_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    lag_warn_sec_ = this->declare_parameter<double>("lag_warn_sec", 1.0);

    create_stamped_relay<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud", "/utlidar/cloud_synced", subscription_qos, publication_qos);
    create_stamped_relay<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_base", "/utlidar/cloud_base_synced", subscription_qos, publication_qos);
    create_stamped_relay<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_deskewed", "/utlidar/cloud_deskewed_synced", subscription_qos, publication_qos);
    create_stamped_relay<sensor_msgs::msg::Imu>(
      "/utlidar/imu", "/utlidar/imu_synced", subscription_qos, publication_qos);
    create_stamped_relay<geometry_msgs::msg::PoseStamped>(
      "/utlidar/robot_pose", "/utlidar/robot_pose_synced", subscription_qos, publication_qos);
    create_stamped_relay<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom", "/utlidar/robot_odom_synced", subscription_qos, publication_qos);

    auto lidar_state_pub = this->create_publisher<unitree_go::msg::LidarState>(
      "/utlidar/lidar_state_synced", publication_qos);

    auto lidar_state_sub = this->create_subscription<unitree_go::msg::LidarState>(
      "/utlidar/lidar_state",
      subscription_qos,
      [this, lidar_state_pub](const unitree_go::msg::LidarState::ConstSharedPtr msg) {
        auto synced_msg = *msg;
        const double now_sec = this->get_clock()->now().seconds();
        const double lag_sec = now_sec - msg->stamp;
        synced_msg.stamp = now_sec;
        lidar_state_pub->publish(synced_msg);

        if (lag_sec > lag_warn_sec_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Restamping /utlidar/lidar_state by %.3f s to match ROS time", lag_sec);
        }
      });

    subscriptions_.push_back(lidar_state_sub);

    RCLCPP_INFO(this->get_logger(), "utlidar_time_sync started, publishing *_synced topics with current ROS timestamps");
  }

private:
  template<typename MessageT>
  void create_stamped_relay(
    const std::string & input_topic,
    const std::string & output_topic,
    const rclcpp::QoS & subscription_qos,
    const rclcpp::QoS & publication_qos)
  {
    auto publisher = this->create_publisher<MessageT>(output_topic, publication_qos);
    auto subscription = this->create_subscription<MessageT>(
      input_topic,
      subscription_qos,
      [this, publisher, input_topic](const typename MessageT::ConstSharedPtr msg) {
        auto synced_msg = *msg;
        const rclcpp::Time now = this->get_clock()->now();
        const double lag_sec = (now - rclcpp::Time(msg->header.stamp)).seconds();
        synced_msg.header.stamp = now;
        publisher->publish(synced_msg);

        if (lag_sec > lag_warn_sec_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Restamping %s by %.3f s to match ROS time",
            input_topic.c_str(), lag_sec);
        }
      });

    subscriptions_.push_back(subscription);
  }

  double lag_warn_sec_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UtlidarTimeSync>());
  rclcpp::shutdown();
  return 0;
}