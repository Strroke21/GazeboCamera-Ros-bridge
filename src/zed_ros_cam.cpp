#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

class GazeboImageBridge : public rclcpp::Node
{
public:
    GazeboImageBridge()
        : Node("gazebo_image_bridge")
    {
        rgb_pub_ =
            create_publisher<sensor_msgs::msg::Image>(
                "/zed2i/image_raw", 10);

        depth_pub_ =
            create_publisher<sensor_msgs::msg::Image>(
                "/zed2i/depth/image_raw", 10);

        gz_node_.Subscribe("/zed2i/image",
                           &GazeboImageBridge::RGBCallback,
                           this);

        gz_node_.Subscribe("/zed2i/depth_image",
                           &GazeboImageBridge::DepthCallback,
                           this);

        RCLCPP_INFO(get_logger(), "Gazebo image bridge started.");
    }

private:

    void RGBCallback(const gz::msgs::Image &msg)
    {
        sensor_msgs::msg::Image ros;

        ros.header.stamp = now();
        ros.header.frame_id = "zed2i_camera";

        ros.height = msg.height();
        ros.width = msg.width();
        std::cout<<"[Publishing RGB frames]: "<<ros.width<<"x"<<ros.height<<std::endl;

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

        ros.header.stamp = now();
        ros.header.frame_id = "zed2i_depth";

        ros.height = msg.height();
        ros.width = msg.width();
        std::cout<<"[Publishing Depth frames]: "<<ros.width<<"x"<<ros.height<<std::endl;

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

    gz::transport::Node gz_node_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_; };

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GazeboImageBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
