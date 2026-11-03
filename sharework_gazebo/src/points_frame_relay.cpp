// Gazebo's rgbd_camera gives one frame id to everything it publishes, but its
// point cloud is in the sensor frame (x forward), while the images are in the
// optical frame. This relay republishes each camera's cloud with the camera
// link as frame id, and leaves the images on the optical frame.
//
// Parameters: cameras (string list). For each <c>: /<c>/gz/points in,
// /<c>/depth/color/points out, frame <c>_camera_link.

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointsFrameRelay : public rclcpp::Node
{
public:
  PointsFrameRelay()
  : Node("points_frame_relay")
  {
    const auto cameras = declare_parameter<std::vector<std::string>>("cameras", {});
    for (const auto & cam : cameras) {
      auto pub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/" + cam + "/depth/color/points", rclcpp::QoS(5));
      const std::string frame = cam + "_camera_link";
      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        "/" + cam + "/gz/points", rclcpp::QoS(5),
        [pub, frame](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
          msg->header.frame_id = frame;
          pub->publish(std::move(msg));
        }));
      pubs_.push_back(pub);
    }
  }

private:
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointsFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
