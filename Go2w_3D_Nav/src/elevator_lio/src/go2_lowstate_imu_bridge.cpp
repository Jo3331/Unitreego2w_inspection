#include <array>
#include <cstdint>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_go/msg/low_state.hpp>

class Go2LowstateImuBridge final : public rclcpp::Node {
public:
  Go2LowstateImuBridge() : Node("go2_lowstate_imu_bridge") {
    const auto qos = rclcpp::SensorDataQoS();
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/go2/imu", qos);
    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
        "/lowstate", qos,
        std::bind(&Go2LowstateImuBridge::on_lowstate, this, std::placeholders::_1));
  }

private:
  void on_lowstate(const unitree_go::msg::LowState::SharedPtr state) {
    sensor_msgs::msg::Imu imu;

    // Keep Livox's ROS-clock time domain but prevent small NTP clock steps from
    // producing out-of-order IMU samples, which makes FAST-LIO clear its buffer.
    int64_t stamp_ns = now().nanoseconds();
    if (stamp_ns <= last_stamp_ns_) {
      stamp_ns = last_stamp_ns_ + 1;
    }
    last_stamp_ns_ = stamp_ns;
    imu.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    imu.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    imu.header.frame_id = "go2_imu";

    // Unitree Go2 LowState quaternion order is [w, x, y, z].
    const auto &q = state->imu_state.quaternion;
    imu.orientation.w = q[0];
    imu.orientation.x = q[1];
    imu.orientation.y = q[2];
    imu.orientation.z = q[3];

    const auto &gyro = state->imu_state.gyroscope;
    imu.angular_velocity.x = gyro[0];
    imu.angular_velocity.y = gyro[1];
    imu.angular_velocity.z = gyro[2];

    const auto &accel = state->imu_state.accelerometer;
    imu.linear_acceleration.x = accel[0];
    imu.linear_acceleration.y = accel[1];
    imu.linear_acceleration.z = accel[2];
    imu_pub_->publish(imu);
  }

  int64_t last_stamp_ns_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2LowstateImuBridge>());
  rclcpp::shutdown();
  return 0;
}
