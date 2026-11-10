// Turns the Gazebo RGB-D streams of a simulated RealSense into what the
// realsense2_camera driver publishes (align_depth and pointcloud enabled).
//
// In (from ros_gz_bridge):
//   /<camera>/gz/color           rgb8, color sensor
//   /<camera>/gz/depth           32FC1 m, depth sensor
//   /<camera>/gz/aligned_depth   32FC1 m, rendered at the color sensor
//   /<camera>/gz/points          cloud of the depth sensor, x forward
//   /<camera>/color/camera_info
// Out (images through image_transport, as the driver: compressed etc. too):
//   /<camera>/color/image_raw                       rgb8
//   /<camera>/depth/image_rect_raw                  16UC1 mm
//   /<camera>/aligned_depth_to_color/image_raw      16UC1 mm
//   /<camera>/aligned_depth_to_color/camera_info    copy of the color one
//   /<camera>/depth/color/points                    x y z rgb, unordered,
//                                                   <camera>_depth_optical_frame
//
// As the driver: no depth is 0, depth beyond clip_distance (m, <= 0: off) is 0
// and is not in the cloud.
//
// Parameters: camera_name (default "wrist"), clip_distance (default -1).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

class RealsenseSimAdapter : public rclcpp::Node
{
public:
  RealsenseSimAdapter()
  : Node("realsense_sim_adapter")
  {
    const auto cam = "/" + declare_parameter<std::string>("camera_name", "wrist");
    clip_ = declare_parameter<double>("clip_distance", -1.0);
    cloud_frame_ = cam.substr(1) + "_depth_optical_frame";

    const auto qos = rclcpp::QoS(5);
    color_pub_ = image_transport::create_publisher(this, cam + "/color/image_raw");
    depth_pub_ = image_transport::create_publisher(this, cam + "/depth/image_rect_raw");
    aligned_pub_ = image_transport::create_publisher(
      this, cam + "/aligned_depth_to_color/image_raw");
    aligned_info_pub_ = create_publisher<CameraInfo>(
      cam + "/aligned_depth_to_color/camera_info", qos);
    points_pub_ = create_publisher<PointCloud2>(cam + "/depth/color/points", qos);

    color_sub_ = create_subscription<Image>(
      cam + "/gz/color", qos,
      [this](Image::ConstSharedPtr m) {color_pub_.publish(*m);});
    depth_sub_ = create_subscription<Image>(
      cam + "/gz/depth", qos,
      [this](Image::ConstSharedPtr m) {depth_pub_.publish(to_mm(*m));});
    aligned_sub_ = create_subscription<Image>(
      cam + "/gz/aligned_depth", qos,
      [this](Image::ConstSharedPtr m) {aligned_pub_.publish(to_mm(*m));});
    info_sub_ = create_subscription<CameraInfo>(
      cam + "/color/camera_info", qos,
      [this](CameraInfo::ConstSharedPtr m) {aligned_info_pub_->publish(*m);});
    points_sub_ = create_subscription<PointCloud2>(
      cam + "/gz/points", qos,
      [this](PointCloud2::ConstSharedPtr m) {points_pub_->publish(to_optical(*m));});
  }

private:
  bool valid(float d) const
  {
    return std::isfinite(d) && d > 0.0f && (clip_ <= 0.0 || d <= clip_);
  }

  Image to_mm(const Image & in) const
  {
    Image out;
    out.header = in.header;
    out.height = in.height;
    out.width = in.width;
    out.encoding = "16UC1";
    out.is_bigendian = false;
    out.step = in.width * sizeof(uint16_t);
    out.data.resize(out.step * out.height);
    if (in.encoding != "32FC1") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "depth encoding %s, expected 32FC1", in.encoding.c_str());
      return out;
    }
    auto * dst = reinterpret_cast<uint16_t *>(out.data.data());
    for (uint32_t r = 0; r < in.height; ++r) {
      for (uint32_t c = 0; c < in.width; ++c) {
        float d;
        std::memcpy(&d, &in.data[r * in.step + c * sizeof(float)], sizeof(float));
        const float mm = valid(d) ? std::round(d * 1000.0f) : 0.0f;
        dst[r * in.width + c] = static_cast<uint16_t>(std::min(mm, 65535.0f));
      }
    }
    return out;
  }

  // Gazebo cloud: sensor frame, x forward. Driver cloud: optical frame, z forward.
  PointCloud2 to_optical(const PointCloud2 & in) const
  {
    PointCloud2 out;
    out.header.stamp = in.header.stamp;
    out.header.frame_id = cloud_frame_;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(static_cast<size_t>(in.width) * in.height);

    sensor_msgs::PointCloud2ConstIterator<float> ix(in, "x"), iy(in, "y"), iz(in, "z");
    sensor_msgs::PointCloud2ConstIterator<float> irgb(in, "rgb");
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    sensor_msgs::PointCloud2Iterator<float> orgb(out, "rgb");
    size_t n = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz, ++irgb) {
      const float depth = *ix;
      if (!valid(depth) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
        continue;
      }
      *ox = -*iy;
      *oy = -*iz;
      *oz = depth;
      *orgb = *irgb;
      ++ox; ++oy; ++oz; ++orgb; ++n;
    }
    mod.resize(n);
    out.height = 1;
    out.width = static_cast<uint32_t>(n);
    out.is_dense = true;
    return out;
  }

  double clip_;
  std::string cloud_frame_;
  image_transport::Publisher color_pub_, depth_pub_, aligned_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr aligned_info_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_, depth_sub_, aligned_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr points_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealsenseSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
