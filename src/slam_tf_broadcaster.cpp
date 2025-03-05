#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// PCL 관련 헤더
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

class PointCloudAdjustNode : public rclcpp::Node
{
public:
    PointCloudAdjustNode()
        : Node("pointcloud_adjust_node")
    {
        // 파라미터 선언 (필요시 launch 파일에서 설정 가능)
        this->declare_parameter<std::string>("imu_topic", "robot0/imu");
        this->declare_parameter<std::string>("cloud_in_topic", "/robot0/point_cloud2");
        this->declare_parameter<std::string>("cloud_out_topic", "/robot0/point_cloud2_adjusted");

        // Translation 파라미터 추가
        this->declare_parameter<double>("translation_x", 0.0);
        this->declare_parameter<double>("translation_y", 0.0);
        this->declare_parameter<double>("translation_z", 0.0);

        // 파라미터 읽기
        auto imu_topic = this->get_parameter("imu_topic").as_string();
        auto cloud_in_topic = this->get_parameter("cloud_in_topic").as_string();
        auto cloud_out_topic = this->get_parameter("cloud_out_topic").as_string();

        translation_x_ = this->get_parameter("translation_x").as_double();
        translation_y_ = this->get_parameter("translation_y").as_double();
        translation_z_ = this->get_parameter("translation_z").as_double();

        // IMU 구독
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&PointCloudAdjustNode::imuCallback, this, std::placeholders::_1));

        // 원본 PointCloud 구독
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic, 10,
            std::bind(&PointCloudAdjustNode::pointCloudCallback, this, std::placeholders::_1));

        // 보정된 PointCloud 퍼블리셔
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "PointCloudAdjustNode started.\n"
                    " Subscribing IMU: %s\n"
                    " Subscribing Cloud: %s\n"
                    " Publishing adjusted Cloud: %s\n"
                    " Translation Offsets (x, y, z): (%.3f, %.3f, %.3f)",
                    imu_topic.c_str(), cloud_in_topic.c_str(), cloud_out_topic.c_str(),
                    translation_x_, translation_y_, translation_z_);
    }

private:
    // 구독/퍼블리셔
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 최근 IMU roll/pitch (yaw는 0 가정)
    double roll_ = 0.0;
    double pitch_ = 0.0;

    // Translation 오프셋
    double translation_x_ = 0.0;
    double translation_y_ = 0.0;
    double translation_z_ = 0.0;

    // IMU 콜백
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMU에서 Quaternion 추출
        tf2::Quaternion imu_quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        // RPY 변환
        double r, p, y;
        tf2::Matrix3x3(imu_quat).getRPY(r, p, y);

        // yaw는 안 쓰고 roll, pitch만 저장
        roll_ = r;
        pitch_ = p;

        // 로그
        RCLCPP_INFO(this->get_logger(), "IMU RPY: (%.3f, %.3f, %.3f)", roll_, pitch_, y);
    }

    // PointCloud 콜백
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1) sensor_msgs::PointCloud2 -> pcl::PointCloud 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud_in);

        // 2) 변환 행렬 생성 (roll, pitch 적용 + translation 추가)
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(static_cast<float>(roll_), Eigen::Vector3f::UnitX()));
        transform.rotate(Eigen::AngleAxisf(static_cast<float>(pitch_), Eigen::Vector3f::UnitY()));

        // Translation 적용
        transform.translation() << static_cast<float>(translation_x_),
            static_cast<float>(translation_y_),
            static_cast<float>(translation_z_);

        // 3) 포인트 클라우드 변환 적용
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

        // 4) 다시 sensor_msgs::PointCloud2로 변환 후 publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_out, output_msg);
        output_msg.header = msg->header;

        cloud_pub_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Published adjusted LiDAR PointCloud");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudAdjustNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
