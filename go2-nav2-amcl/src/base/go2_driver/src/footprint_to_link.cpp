#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <numeric>

using namespace std::chrono_literals;

namespace
{

struct Vec3
{
    double x;
    double y;
    double z;
};

Vec3 add(const Vec3 & lhs, const Vec3 & rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 rotate_x(const Vec3 & point, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {point.x, c * point.y - s * point.z, s * point.y + c * point.z};
}

Vec3 rotate_y(const Vec3 & point, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c * point.x + s * point.z, point.y, -s * point.x + c * point.z};
}

double clamp_force(int16_t force)
{
    return std::max(0.0, static_cast<double>(force));
}

}  // namespace

class TFDynamicBroadcaster : public rclcpp::Node
{
public:
    TFDynamicBroadcaster()
    : Node("tf_dynamic_broadcaster"), body_height_(0.333), support_force_threshold_(20.0)
    {
        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lf/lowstate", 10,
            std::bind(&TFDynamicBroadcaster::lowstate_cb, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&TFDynamicBroadcaster::timer_cb, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "启动动态tf变换节点，使用 lowstate 估算 base_link 高度");
    }

private:
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double body_height_;
    const double support_force_threshold_;

    static double estimate_leg_height(double hip, double thigh, double calf, const Vec3 & hip_origin)
    {
        constexpr Vec3 hip_to_thigh{0.0, 0.0955, 0.0};
        constexpr Vec3 thigh_to_calf{0.0, 0.0, -0.213};
        constexpr Vec3 calf_to_foot{0.0, 0.0, -0.213};

        Vec3 foot = hip_origin;
        foot = add(foot, rotate_x(hip_to_thigh, hip));
        foot = add(foot, rotate_x(rotate_y(thigh_to_calf, thigh), hip));
        foot = add(foot, rotate_x(rotate_y(rotate_y(calf_to_foot, calf), thigh), hip));
        return -foot.z;
    }

    void lowstate_cb(const unitree_go::msg::LowState::SharedPtr state_msg)
    {
        static constexpr std::array<Vec3, 4> hip_origins{{
            {0.1934, 0.0465, 0.0},
            {0.1934, -0.0465, 0.0},
            {-0.1934, 0.0465, 0.0},
            {-0.1934, -0.0465, 0.0},
        }};

        std::array<double, 4> leg_heights{};
        std::array<double, 4> support_weights{};

        for (size_t leg_index = 0; leg_index < 4; ++leg_index) {
            const size_t motor_offset = leg_index * 3;
            const auto & hip_motor = state_msg->motor_state[motor_offset];
            const auto & thigh_motor = state_msg->motor_state[motor_offset + 1];
            const auto & calf_motor = state_msg->motor_state[motor_offset + 2];

            leg_heights[leg_index] = estimate_leg_height(
                hip_motor.q, thigh_motor.q, calf_motor.q, hip_origins[leg_index]);

            const double foot_force = clamp_force(state_msg->foot_force[leg_index]);
            support_weights[leg_index] = foot_force >= support_force_threshold_ ? foot_force : 0.0;
        }

        const double support_force_sum = std::accumulate(
            support_weights.begin(), support_weights.end(), 0.0);

        if (support_force_sum > 0.0) {
            double weighted_height = 0.0;
            for (size_t leg_index = 0; leg_index < 4; ++leg_index) {
                weighted_height += leg_heights[leg_index] * support_weights[leg_index];
            }
            body_height_ = weighted_height / support_force_sum;
        } else {
            body_height_ = std::accumulate(leg_heights.begin(), leg_heights.end(), 0.0) /
                static_cast<double>(leg_heights.size());
        }
    }

    void timer_cb()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "base_footprint";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = body_height_;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.0);
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFDynamicBroadcaster>());
    rclcpp::shutdown();
    return 0;
}

