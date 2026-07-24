#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

class Mid360Bridge : public rclcpp::Node
{
public:
    Mid360Bridge() : Node("mid360_bridge")
    {

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        pub1_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mid360/points1", 1);

        pub2_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mid360/points2", 1);

        gz_node_.Subscribe("/mid360/points",
                           &Mid360Bridge::Callback1,
                           this);

        gz_node_.Subscribe("/mid360/points/points",
                           &Mid360Bridge::Callback2,
                           this);

        RCLCPP_INFO(get_logger(), "MID-360 bridge started.");
    }

private:

    sensor_msgs::msg::PointCloud2 Convert(
        const gz::msgs::PointCloudPacked &msg)
    {
        sensor_msgs::msg::PointCloud2 ros_msg;

        ros_msg.header.stamp = now();
        ros_msg.header.frame_id = "base_link";

        ros_msg.height = msg.height();
        ros_msg.width = msg.width();

        ros_msg.point_step = msg.point_step();
        ros_msg.row_step = msg.row_step();

        ros_msg.is_bigendian = false;
        ros_msg.is_dense = false;

        ros_msg.data.resize(msg.data().size());
        memcpy(ros_msg.data.data(),
               msg.data().data(),
               msg.data().size());

        ros_msg.fields.clear();

        for (int i = 0; i < msg.field_size(); ++i)
        {
            sensor_msgs::msg::PointField field;

            field.name = msg.field(i).name();
            field.offset = msg.field(i).offset();
            field.count = msg.field(i).count();

            switch (msg.field(i).datatype())
            {
                case gz::msgs::PointCloudPacked::Field::INT8:
                    field.datatype = sensor_msgs::msg::PointField::INT8;
                    break;

                case gz::msgs::PointCloudPacked::Field::UINT8:
                    field.datatype = sensor_msgs::msg::PointField::UINT8;
                    break;

                case gz::msgs::PointCloudPacked::Field::INT16:
                    field.datatype = sensor_msgs::msg::PointField::INT16;
                    break;

                case gz::msgs::PointCloudPacked::Field::UINT16:
                    field.datatype = sensor_msgs::msg::PointField::UINT16;
                    break;

                case gz::msgs::PointCloudPacked::Field::INT32:
                    field.datatype = sensor_msgs::msg::PointField::INT32;
                    break;

                case gz::msgs::PointCloudPacked::Field::UINT32:
                    field.datatype = sensor_msgs::msg::PointField::UINT32;
                    break;

                case gz::msgs::PointCloudPacked::Field::FLOAT32:
                    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                    break;

                case gz::msgs::PointCloudPacked::Field::FLOAT64:
                    field.datatype = sensor_msgs::msg::PointField::FLOAT64;
                    break;

                default:
                    continue;
            }

            ros_msg.fields.push_back(field);
        }

        return ros_msg;
    }

    void Callback1(const gz::msgs::PointCloudPacked &msg)
    {
        pub1_->publish(Convert(msg));
    }

    void Callback2(const gz::msgs::PointCloudPacked &msg)
    {
        pub2_->publish(Convert(msg));
    }

    gz::transport::Node gz_node_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Mid360Bridge>());

    rclcpp::shutdown();

    return 0;
}
