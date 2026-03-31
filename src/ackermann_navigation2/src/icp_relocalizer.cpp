#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ICPRelocalizer : public rclcpp::Node {
public:
    ICPRelocalizer() : Node("icp_relocalizer") {
        // 配置参数
        this->declare_parameter("max_fitness_score", 0.15); // 阈值：越小越严格，通常 0.05-0.2
        this->declare_parameter("max_correspondence_dist", 3.0); // 搜索半径：越大越能纠正大偏差

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&ICPRelocalizer::map_callback, this, std::placeholders::_1));

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&ICPRelocalizer::initial_pose_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&ICPRelocalizer::scan_callback, this, std::placeholders::_1));

        refined_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose_refined", 10);
        
        RCLCPP_INFO(this->get_logger(), "增强型 ICP 中继器已启动。设置了评分过滤机制。");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        for (unsigned int y = 0; y < msg->info.height; ++y) {
            for (unsigned int x = 0; x < msg->info.width; ++x) {
                if (msg->data[x + y * msg->info.width] > 50) { 
                    map_cloud_->push_back(pcl::PointXYZ(
                        msg->info.origin.position.x + x * msg->info.resolution,
                        msg->info.origin.position.y + y * msg->info.resolution, 0.0));
                }
            }
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (!map_cloud_ || !last_scan_) return;

        double score_threshold = this->get_parameter("max_fitness_score").as_double();
        double max_dist = this->get_parameter("max_correspondence_dist").as_double();

        // 转换点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            float range = last_scan_->ranges[i];
            if (std::isfinite(range) && range > last_scan_->range_min && range < last_scan_->range_max) {
                float angle = last_scan_->angle_min + i * last_scan_->angle_increment;
                scan_cloud->push_back(pcl::PointXYZ(range * std::cos(angle), range * std::sin(angle), 0.0));
            }
        }

        // ICP 参数增强
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(scan_cloud);
        icp.setInputTarget(map_cloud_);
        
        // 核心参数优化
        icp.setMaxCorrespondenceDistance(max_dist); // 允许捕捉更远的特征;
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-5);
        icp.setMaximumIterations(100);
        icp.setRANSACIterations(10); // 增加 RANSAC 鲁棒性

        Eigen::Translation3f init_translation(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);
        Eigen::AngleAxisf init_rotation(tf2::getYaw(msg->pose.pose.orientation), Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, init_guess);

        double fitness_score = icp.getFitnessScore();

        if (icp.hasConverged() && fitness_score < score_threshold) {
            Eigen::Matrix4f trans = icp.getFinalTransformation();
            auto refined_pose = *msg;
            refined_pose.pose.pose.position.x = trans(0, 3);
            refined_pose.pose.pose.position.y = trans(1, 3);
            
            double yaw = std::atan2(trans(1, 0), trans(0, 0));
            tf2::Quaternion q; q.setRPY(0, 0, yaw);
            refined_pose.pose.pose.orientation = tf2::toMsg(q);

            refined_pose_pub_->publish(refined_pose);
            RCLCPP_INFO(this->get_logger(), "配准成功! 分数: %f (阈值: %f)", fitness_score, score_threshold);
        } else {
            RCLCPP_ERROR(this->get_logger(), "配准被拒绝! 分数: %f (得分过高说明匹配不可信或不收敛)", fitness_score);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr refined_pose_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPRelocalizer>());
    rclcpp::shutdown();
    return 0;
}