#ifndef SHAREWORK_GAZEBO__GRAVITY_COMPENSATED_GZ_SYSTEM_HPP_
#define SHAREWORK_GAZEBO__GRAVITY_COMPENSATED_GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace sharework_gazebo
{

// gz_ros2_control/GazeboSimSystem whose effort command is the torque net of
// gravity, as on the real UR (direct_torque): write() adds g(q) before Gazebo
// applies it. g(q) comes from Pinocchio, on the robot_description that Gazebo
// simulates. Position and velocity commands pass through unchanged.
//
// It wraps a GazeboSimSystem loaded through pluginlib rather than deriving from
// it: GazeboSimSystem's implicit destructor needs its private, undeclared data
// type, so no other library can destroy one directly.
//
// Hardware parameters (<param> in <hardware>), all optional:
//   robot_description_node  node holding robot_description (robot_state_publisher)
//   robot_description_param parameter name (robot_description)
//   gravity_compensation    add g(q) to the effort command (true)
//   net_effort_state        report the effort state net of g(q) as well (false)
class GravityCompensatedGazeboSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  GravityCompensatedGazeboSystem();
  ~GravityCompensatedGazeboSystem() override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    gz::sim::EntityComponentManager & ecm,
    unsigned int update_rate) override;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Declared before inner_: the library must outlive the instance.
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader_;
  pluginlib::UniquePtr<gz_ros2_control::GazeboSimSystemInterface> inner_;

  struct Model;
  std::unique_ptr<Model> model_;

  // Per compensated joint.
  struct Joint
  {
    std::string name;
    std::unique_ptr<hardware_interface::StateInterface> position;      // inner, read
    std::unique_ptr<hardware_interface::StateInterface> effort;        // inner, read
    std::unique_ptr<hardware_interface::CommandInterface> effort_cmd;  // inner, written
    double net_effort_cmd = 0.0;     // what the controllers write
    double net_effort_state = 0.0;   // what the controllers read, with net_effort_state
  };
  std::vector<Joint> joints_;

  bool gravity_compensation_ = true;
  bool net_effort_state_ = false;

  void computeGravity();
  CallbackReturn forward(
    CallbackReturn ret, uint8_t success_state_id, const char * success_state_label);
};

}  // namespace sharework_gazebo

#endif  // SHAREWORK_GAZEBO__GRAVITY_COMPENSATED_GZ_SYSTEM_HPP_
