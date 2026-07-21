#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>

using namespace std;

class GazeboImuBridge : public rclcpp::Node
{
public:
    GazeboImuBridge() : Node("gazebo_imu_bridge")
    {
        // Best Effort QoS for high-rate IMU data
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data",
            qos);

        const std::string gz_topic =
            "/world/anti_drone/model/ad_drone1/model/iris_with_gimbal/"
            "model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu";

        if (!gz_node_.Subscribe(
                gz_topic,
                &GazeboImuBridge::ImuCallback,
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
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GazeboImuBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
