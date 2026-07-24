#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <gz/msgs/imu.pb.h>
#include <sensor_msgs/msg/imu.hpp>

class GazeboImageBridge : public rclcpp::Node
{
public:
    GazeboImageBridge()
        : Node("gazebo_image_bridge")
    {
        rgb_pub_ =
            create_publisher<sensor_msgs::msg::Image>(
                "/zed2i/image_raw", 1);

        depth_pub_ =
            create_publisher<sensor_msgs::msg::Image>(
                "/zed2i/depth/image_raw", 1);

        rgb_info_pub_ = 
            create_publisher<sensor_msgs::msg::CameraInfo>(
                "/zed2i/camera_info", 1);

        gz_node_.Subscribe("/zed2i/image",
                           &GazeboImageBridge::RGBCallback,
                           this);

        gz_node_.Subscribe("/zed2i/depth_image",
                           &GazeboImageBridge::DepthCallback,
                           this);
        gz_node_.Subscribe("/zed2i/camera_info",
                           &GazeboImageBridge::CameraInfoCallback,this);

        RCLCPP_INFO(get_logger(), "Gazebo image bridge started.");
                // Best Effort QoS for high-rate IMU data
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/zed2i/imu",
            qos);

        const std::string gz_topic =
            "/zed2i/imu";

        if (!gz_node_.Subscribe(
                gz_topic,
                &GazeboImageBridge::ImuCallback,
                this))
        {
            RCLCPP_ERROR(get_logger(),
                         "Failed to subscribe to Gazebo topic:\n%s",
                         gz_topic.c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                        "Subscribed to Gazebo topic:\n%s",
                        gz_topic.c_str());
        }
    }

private:
    int counter = 0;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME}; // Keeps track of the previous callback time
    void RGBCallback(const gz::msgs::Image &msg)
    {
        sensor_msgs::msg::Image ros;
        builtin_interfaces::msg::Time t;
        ros.header.frame_id = "zed2i_camera";

        t.sec = msg.header().stamp().sec();
        t.nanosec = msg.header().stamp().nsec();
        ros.header.stamp = t;

        ros.height = msg.height();
        ros.width = msg.width();
        std::cout<<"[Publishing frames] "<<"RGB: "<<ros.width<<" x "<<ros.height<<std::endl;

        ros.encoding = "rgb8";

        ros.is_bigendian = false;

        ros.step = msg.step();

        ros.data.resize(msg.data().size());

        memcpy(
            ros.data.data(),
            msg.data().data(),
            msg.data().size());

        rgb_pub_->publish(ros);
    }

    void DepthCallback(const gz::msgs::Image &msg)
    {
        sensor_msgs::msg::Image ros;
        builtin_interfaces::msg::Time t;
        t.sec = msg.header().stamp().sec();
        t.nanosec = msg.header().stamp().nsec();
        ros.header.stamp = t;

        ros.header.frame_id = "zed2i_camera";

        ros.height = msg.height();
        ros.width = msg.width();
        std::cout<<"[Publishing frames] "<<"Depth: "<<ros.width<<" x "<<ros.height<<std::endl;

        switch (msg.pixel_format_type())
        {
        case gz::msgs::PixelFormatType::R_FLOAT32:
            ros.encoding = "32FC1";
            break;

        case gz::msgs::PixelFormatType::L_INT16:
            ros.encoding = "16UC1";
            break;

        default:
            ros.encoding = "32FC1";
            break;
        }

        ros.is_bigendian = false;

        ros.step = msg.step();

        ros.data.resize(msg.data().size());

        memcpy(
            ros.data.data(),
            msg.data().data(),
            msg.data().size());

        depth_pub_->publish(ros);
    }

    void CameraInfoCallback(const gz::msgs::CameraInfo &msg)
    {
        sensor_msgs::msg::CameraInfo ros; 
        builtin_interfaces::msg::Time t;
        t.sec = msg.header().stamp().sec();
        t.nanosec = msg.header().stamp().nsec();
        ros.header.stamp = t;
        ros.header.frame_id = "zed2i_camera";
        ros.width = msg.width(); 
        ros.height = msg.height();
        double width = 1280.0;
        double height = 720.0;
        double hfov = 1.91986;

        double fx = width / (2.0 * std::tan(hfov / 2.0));
        double fy = fx;                  // assuming square pixels
        double cx = width / 2.0;
        double cy = height / 2.0;

        ros.k = {
          fx, 0, cx,
          0, fy, cy,
          0, 0, 1
          };

        ros.p = {
            fx, 0, cx, 0,
            0, fy, cy, 0,
            0, 0, 1, 0
        };

        ros.r = {
            1,0,0,
            0,1,0,
            0,0,1
        };

        rgb_info_pub_->publish(ros);}

    void ImuCallback(const gz::msgs::IMU &msg)
    {
        sensor_msgs::msg::Imu imu;
        
        // 1. Timestamp using current ROS time
        rclcpp::Time now = this->get_clock()->now();
        imu.header.stamp = now;
        imu.header.frame_id = "imu_link";

        // Orientation
        imu.orientation.x = msg.orientation().x();
        imu.orientation.y = msg.orientation().y();
        imu.orientation.z = msg.orientation().z();
        imu.orientation.w = msg.orientation().w();

        // Angular velocity
        imu.angular_velocity.x = msg.angular_velocity().x();
        imu.angular_velocity.y = msg.angular_velocity().y();
        imu.angular_velocity.z = msg.angular_velocity().z();

        // Linear acceleration
        imu.linear_acceleration.x = msg.linear_acceleration().x();
        imu.linear_acceleration.y = msg.linear_acceleration().y();
        imu.linear_acceleration.z = msg.linear_acceleration().z();

        // Unknown covariance
        imu.orientation_covariance[0] = -1.0;
        imu.angular_velocity_covariance[0] = -1.0;
        imu.linear_acceleration_covariance[0] = -1.0;

        imu_pub_->publish(imu);

        // 2. Calculate actual Hz based on the gap between messages
        counter++;
        if (last_time_.nanoseconds() != 0) {
            rclcpp::Duration time_between_messages = now - last_time_;
            double seconds = time_between_messages.seconds();
            
            if (seconds > 0.0) {
                double current_hz = 1.0 / seconds;
                // Only log every 100 messages so your terminal doesn't get completely flooded
                if (counter % 10 == 0) {
                    RCLCPP_INFO(get_logger(), " IMU Rate: %.2f Hz", current_hz);
                }
            }
        }
        
        // Update the timestamp tracking for the next message
        last_time_ = now;
    }

    gz::transport::Node gz_node_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GazeboImageBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
