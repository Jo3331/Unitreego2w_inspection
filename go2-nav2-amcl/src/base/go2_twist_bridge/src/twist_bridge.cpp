#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::placeholders;

class TwistBridge : public rclcpp::Node
{
public:
    TwistBridge() : Node("twist_bridge_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "TwistBridge创建，可以将geometry_msgs/msg/twist消息转换成unitree_api/msg/request消息!");
        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        //创建一个request发布对象（BEST_EFFORT 匹配 Unitree 裸 DDS 的 QoS）
        auto sport_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", sport_qos);

        // 同时兼容 Twist 与 TwistStamped；只有与上游类型匹配的订阅会真正收到消息。
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10, std::bind(&TwistBridge::twist_cb, this, _1));
        twist_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            cmd_vel_topic_, 10, std::bind(&TwistBridge::twist_stamped_cb, this, _1));

        RCLCPP_INFO(
            this->get_logger(),
            "Listening on %s for Twist/TwistStamped, publishing Unitree requests to /api/sport/request",
            cmd_vel_topic_.c_str());
    }

private:
    void publish_request(double x, double y, double z)
    {
        unitree_api::msg::Request request;

        if(x != 0.0 || y != 0.0 || z != 0.0)
        {
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
            request.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
        }
        else
        {
            request.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
            request.parameter.clear();
        }

        request_pub_->publish(request);
    }

    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        publish_request(twist->linear.x, twist->linear.y, twist->angular.z);
    }

    void twist_stamped_cb(const geometry_msgs::msg::TwistStamped::SharedPtr twist)
    {
        publish_request(twist->twist.linear.x, twist->twist.linear.y, twist->twist.angular.z);
    }

    std::string cmd_vel_topic_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistBridge>());
    rclcpp::shutdown();
    return 0;
}