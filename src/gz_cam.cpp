#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <thread>

// Gazebo camera topic
const std::string CAMERA_TOPIC = "/world/anti_drone/model/ad_drone1/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image";

// Global ROS 2 publisher
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_publisher;

// Convert Gazebo image → ROS 2 Image and publish
void imageCallback(const gz::msgs::Image &msg)
{
    int width = msg.width();
    int height = msg.height();

    // Convert raw Gazebo image data to OpenCV Mat
    cv::Mat img(height, width, CV_8UC3, (void*)msg.data().data());

    // Gazebo images are RGB, OpenCV expects BGR
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

    // Convert OpenCV → ROS 2 Image
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Clock().now();
    header.frame_id = "camera_link";

    sensor_msgs::msg::Image::SharedPtr ros_image =
        cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

    // Publish to ROS 2
    ros_publisher->publish(*ros_image);

    // Optional: Display for debugging
    //cv::imshow("Gazebo Camera", img);
    //cv::waitKey(1);
    std::cout << "Published image: " << width << "x" << height << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node_ros = rclcpp::Node::make_shared("gazebo_camera_bridge");

    // Create ROS 2 publisher
    ros_publisher = node_ros->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

    // OpenCV window
    cv::namedWindow("Gazebo Camera", cv::WINDOW_AUTOSIZE);

    // Gazebo Transport node
    gz::transport::Node node_gz;

    // Subscribe to Gazebo camera
    if (!node_gz.Subscribe(CAMERA_TOPIC, imageCallback))
    {
        std::cerr << "Failed to subscribe to topic [" << CAMERA_TOPIC << "]" << std::endl;
        return -1;
    }

    std::cout << "Subscribed to Gazebo camera topic: " << CAMERA_TOPIC << std::endl;

    // Run ROS 2 spin in a separate thread
    std::thread ros_spin([&]() {
        rclcpp::spin(node_ros);
    });

    // Keep alive
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ros_spin.join();
    rclcpp::shutdown();
    return 0;
}

//./anti_drone_system/build/anti_drone_system/gz_cam_bridge 
