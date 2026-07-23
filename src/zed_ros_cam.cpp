#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <sensor_msgs/msg/camera_info.hpp>

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
    }

private:

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

    gz::transport::Node gz_node_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GazeboImageBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
