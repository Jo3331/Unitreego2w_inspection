#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vector"
#include "cstring"

using namespace std::chrono_literals;

// 自定义节点类
class CloudAccumulator : public rclcpp::Node
{
public:
    CloudAccumulator() : Node("cloud_accumulator")
    {   
        max_clouds_ = static_cast<size_t>(this->declare_parameter<int>("max_clouds", 25));
        min_height_ = this->declare_parameter<double>("min_height", 0.05);
        max_height_ = this->declare_parameter<double>("max_height", 0.8);
        publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 100);
        max_cloud_age_sec_ = this->declare_parameter<double>("max_cloud_age_sec", 2.0);
        RCLCPP_INFO(this->get_logger(), "Cloud accumulator node started.");
        RCLCPP_INFO(
            this->get_logger(),
            "cloud accumulation params: max_clouds=%zu, min_height=%.3f, max_height=%.3f, publish_period_ms=%d, max_cloud_age_sec=%.2f",
            max_clouds_, min_height_, max_height_, publish_period_ms_, max_cloud_age_sec_);
        // 配置qos，确保数据能够传输，需要确认slam-toolbox用的是什么qos
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        qos.reliable();

        // 创建订阅方， 订阅机器狗的 /utlidar/cloud_deskewed 话题
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_deskewed",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudAccumulator::cloud_callback, this, std::placeholders::_1)
        );

        // 创建发布方，发布机器狗的 /utlidar/cloud_accumulated 话题
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_accumulated",
            qos
        );

        // 创建定时器，定时发布数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms_),
            std::bind(&CloudAccumulator::timer_callback, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_; // 创建存储点云的容器(注意点云数据是只读的，不能被修改)

    size_t max_clouds_;  // 最多存储若干帧点云数据
    double min_height_; // 最小高度阈值
    double max_height_; // 最大高度阈值
    int publish_period_ms_; // 积累点云发布周期
    double max_cloud_age_sec_; // 超过此秒数的帧视为过期并丢弃


    // 回调函数：只负责入队，不做 merge/filter，避免每帧重复计算
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        clouds_.push_back(cloud_msg);

        // 如果点云的数量超过max_clouds_，则删除最老的点云数据
        while (clouds_.size() > max_clouds_)
        {
            clouds_.erase(clouds_.begin());
        }
    }

    // 合并多个点云的函数（带 reserve，减少 realloc）
    std::shared_ptr<sensor_msgs::msg::PointCloud2> merged_clouds()
    {
        auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        *merged_cloud = *clouds_[0];

        // 预估总字节数并预留空间，避免 insert 多次 realloc
        size_t total_bytes = merged_cloud->data.size();
        for (size_t i = 1; i < clouds_.size(); i++) {
            total_bytes += clouds_[i]->data.size();
        }
        merged_cloud->data.reserve(total_bytes);

        for (size_t i = 1; i < clouds_.size(); i++)
        {
            merged_cloud->width += clouds_[i]->width;
            merged_cloud->row_step += clouds_[i]->row_step;
            merged_cloud->data.insert(merged_cloud->data.end(),
                    clouds_[i]->data.begin(),
                    clouds_[i]->data.end());
        }
        return merged_cloud;
    }

    // 高度过滤（预留输出缓冲区，减少 realloc）
    sensor_msgs::msg::PointCloud2 filter_cloud(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        sensor_msgs::msg::PointCloud2 filtered_cloud;
        filtered_cloud.header = cloud.header;
        filtered_cloud.height = 1;
        filtered_cloud.width = 0;
        filtered_cloud.fields = cloud.fields;
        filtered_cloud.is_bigendian = cloud.is_bigendian;
        filtered_cloud.point_step = cloud.point_step;
        filtered_cloud.row_step = 0;
        filtered_cloud.is_dense = false;

        const size_t total_points = cloud.width * cloud.height;
        // 最坏情况全部点通过，预留空间
        filtered_cloud.data.reserve(total_points * cloud.point_step);

        for (size_t i = 0; i < total_points; i++)
        {
            float z;
            memcpy(&z, &cloud.data[i * cloud.point_step + cloud.fields[2].offset], sizeof(float));

            if(z >= min_height_ && z <= max_height_)
            {
                filtered_cloud.data.insert(filtered_cloud.data.end(),
                        &cloud.data[i * cloud.point_step],
                        &cloud.data[(i + 1) * cloud.point_step]);
                filtered_cloud.row_step += cloud.point_step;
                filtered_cloud.width++;
            }
        }
        return filtered_cloud;
    }

    void timer_callback()
    {
        if (clouds_.empty()) return;

        // 丢弃过期帧：时间戳早于 (now - max_cloud_age_sec_) 的帧直接删除
        if (max_cloud_age_sec_ > 0.0) {
            const rclcpp::Time now = this->get_clock()->now();
            auto it = clouds_.begin();
            while (it != clouds_.end()) {
                const double age = (now - rclcpp::Time((*it)->header.stamp)).seconds();
                if (age > max_cloud_age_sec_) {
                    it = clouds_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (clouds_.empty()) return;

        // 只在发布时做一次 merge + filter，而不是每帧回调都做
        auto merged_cloud = merged_clouds();
        auto filtered_cloud = filter_cloud(*merged_cloud);

        // 原: filtered_cloud.header.stamp = this->get_clock()->now();  重盖 now() 使 scan 时戳晚于真实采集~100ms
        // 现: 沿用 filter_cloud() 已复制的输入点云 header.stamp(=clouds_[0] 采集时刻), 保持时戳一致消除漂移
        pub_->publish(filtered_cloud);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}
