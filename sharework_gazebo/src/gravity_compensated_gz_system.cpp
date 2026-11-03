#include "sharework_gazebo/gravity_compensated_gz_system.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sharework_gazebo
{

using hardware_interface::return_type;
using lifecycle_msgs::msg::State;
namespace lifecycle_names = hardware_interface::lifecycle_state_names;

struct GravityCompensatedGazeboSystem::Model
{
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd q;
  Eigen::VectorXd g;        // generalized gravity, per Pinocchio dof
  std::vector<int> idx_q;   // per Joint, index in q
  std::vector<int> nq;      // per Joint, 1 (revolute/prismatic) or 2 (continuous)
  std::vector<int> idx_v;   // per Joint, index in g
};

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("GravityCompensatedGazeboSystem");

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

bool to_bool(const std::string & s) {return s == "true" || s == "True" || s == "1";}

// robot_description from the node that publishes it. A node of our own: the
// plugin's node is already spun by Gazebo's executor.
std::string fetch_robot_description(
  const std::string & ns, const std::string & node_name, const std::string & param_name)
{
  auto node = rclcpp::Node::make_shared(
    "gravity_compensation_description_client", ns,
    rclcpp::NodeOptions().use_global_arguments(false).start_parameter_services(false));
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, node_name);
  for (int i = 0; i < 20 && !client->wait_for_service(std::chrono::milliseconds(500)); ++i) {
    RCLCPP_INFO(kLogger, "Waiting for %s to read %s", node_name.c_str(), param_name.c_str());
  }
  if (!client->service_is_ready()) {
    throw std::runtime_error("parameter service of " + node_name + " not available");
  }
  auto params = client->get_parameters({param_name}, std::chrono::seconds(5));
  if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw std::runtime_error(node_name + " has no string parameter " + param_name);
  }
  return params[0].as_string();
}
}  // namespace

GravityCompensatedGazeboSystem::GravityCompensatedGazeboSystem()
: loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface")
{
}

GravityCompensatedGazeboSystem::~GravityCompensatedGazeboSystem()
{
  inner_.reset();  // before loader_ unloads its library
}

bool GravityCompensatedGazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, gz::sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  gz::sim::EntityComponentManager & ecm,
  unsigned int update_rate)
{
  nh_ = model_nh;
  try {
    inner_ = loader_.createUniqueInstance("gz_ros2_control/GazeboSimSystem");
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(kLogger, "Cannot load gz_ros2_control/GazeboSimSystem: %s", e.what());
    return false;
  }
  if (!inner_->initSim(model_nh, joints, hardware_info, ecm, update_rate)) {
    return false;
  }

  gravity_compensation_ = to_bool(param(hardware_info, "gravity_compensation", "true"));
  net_effort_state_ = to_bool(param(hardware_info, "net_effort_state", "false"));

  // The joints we compensate: those with an effort command interface.
  joints_.clear();
  for (const auto & joint : hardware_info.joints) {
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_EFFORT) {
        joints_.emplace_back();
        joints_.back().name = joint.name;
      }
    }
  }
  if (!gravity_compensation_ || joints_.empty()) {
    RCLCPP_INFO(kLogger, "%s: gravity compensation off", hardware_info.name.c_str());
    return true;
  }

  std::string urdf;
  try {
    urdf = fetch_robot_description(
      model_nh->get_namespace(),
      param(hardware_info, "robot_description_node", "robot_state_publisher"),
      param(hardware_info, "robot_description_param", "robot_description"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Cannot read the robot description: %s", e.what());
    return false;
  }

  // Full model, then lock every joint we do not drive (the gripper, ...) at its
  // neutral position: the same masses, with only our joints left free.
  pinocchio::Model full;
  try {
    pinocchio::urdf::buildModelFromXML(urdf, full);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Pinocchio cannot parse the robot description: %s", e.what());
    return false;
  }

  std::set<std::string> ours;
  for (const auto & j : joints_) {
    if (!full.existJointName(j.name)) {
      RCLCPP_ERROR(kLogger, "Joint %s is not in the robot description", j.name.c_str());
      return false;
    }
    ours.insert(j.name);
  }
  std::vector<pinocchio::JointIndex> to_lock;
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(full.njoints); ++id) {
    if (!ours.count(full.names[id])) {
      to_lock.push_back(id);
    }
  }

  model_ = std::make_unique<Model>();
  model_->model = pinocchio::buildReducedModel(full, to_lock, pinocchio::neutral(full));
  model_->data = pinocchio::Data(model_->model);
  model_->q = pinocchio::neutral(model_->model);
  model_->g = Eigen::VectorXd::Zero(model_->model.nv);
  for (const auto & j : joints_) {
    const auto id = model_->model.getJointId(j.name);
    model_->idx_q.push_back(model_->model.joints[id].idx_q());
    model_->nq.push_back(model_->model.joints[id].nq());
    model_->idx_v.push_back(model_->model.joints[id].idx_v());
  }

  RCLCPP_INFO(
    kLogger, "%s: gravity compensation on %zu joints, gravity [%g %g %g]",
    hardware_info.name.c_str(), joints_.size(),
    model_->model.gravity.linear().x(), model_->model.gravity.linear().y(),
    model_->model.gravity.linear().z());
  return true;
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (GazeboSimSystemInterface::on_init(params) != CallbackReturn::SUCCESS || !inner_) {
    return CallbackReturn::ERROR;
  }
  // No executor: the inner system must not create a second node of our name.
  hardware_interface::HardwareComponentParams inner_params;
  inner_params.hardware_info = params.hardware_info;
  inner_params.logger = get_logger();
  inner_params.clock = get_clock();
  return inner_->init(inner_params);
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::forward(
  CallbackReturn ret, uint8_t success_state_id, const char * success_state_label)
{
  if (ret == CallbackReturn::SUCCESS) {
    inner_->set_lifecycle_state(rclcpp_lifecycle::State(success_state_id, success_state_label));
  }
  return ret;
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_configure(
  const rclcpp_lifecycle::State & s)
{
  return forward(inner_->on_configure(s), State::PRIMARY_STATE_INACTIVE, lifecycle_names::INACTIVE);
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_cleanup(
  const rclcpp_lifecycle::State & s)
{
  return forward(
    inner_->on_cleanup(s), State::PRIMARY_STATE_UNCONFIGURED, lifecycle_names::UNCONFIGURED);
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_shutdown(
  const rclcpp_lifecycle::State & s)
{
  return forward(inner_->on_shutdown(s), State::PRIMARY_STATE_FINALIZED, lifecycle_names::FINALIZED);
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_activate(
  const rclcpp_lifecycle::State & s)
{
  for (auto & j : joints_) {
    j.net_effort_cmd = 0.0;  // hold the arm when an effort controller starts
  }
  return forward(inner_->on_activate(s), State::PRIMARY_STATE_ACTIVE, lifecycle_names::ACTIVE);
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_deactivate(
  const rclcpp_lifecycle::State & s)
{
  return forward(
    inner_->on_deactivate(s), State::PRIMARY_STATE_INACTIVE, lifecycle_names::INACTIVE);
}

hardware_interface::CallbackReturn GravityCompensatedGazeboSystem::on_error(
  const rclcpp_lifecycle::State & s)
{
  return inner_->on_error(s);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// GazeboSimSystem exports through the legacy pointer-based API; so do we.

std::vector<hardware_interface::StateInterface>
GravityCompensatedGazeboSystem::export_state_interfaces()
{
  auto inner = inner_->export_state_interfaces();
  if (!model_) {
    return inner;
  }
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(inner.size());
  for (auto & si : inner) {
    Joint * joint = nullptr;
    for (auto & j : joints_) {
      if (si.get_prefix_name() == j.name) {
        joint = &j;
      }
    }
    if (joint && si.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint->position = std::make_unique<hardware_interface::StateInterface>(si);
    } else if (joint && si.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      joint->effort = std::make_unique<hardware_interface::StateInterface>(si);
      if (net_effort_state_) {
        out.emplace_back(joint->name, hardware_interface::HW_IF_EFFORT, &joint->net_effort_state);
        continue;
      }
    }
    out.push_back(std::move(si));
  }
  for (const auto & j : joints_) {
    if (!j.position) {
      throw std::runtime_error("joint " + j.name + " has no position state interface");
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
GravityCompensatedGazeboSystem::export_command_interfaces()
{
  auto inner = inner_->export_command_interfaces();
  if (!model_) {
    return inner;
  }
  // The controllers get our net command; the inner system keeps its own,
  // which write() sets to net + g(q).
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(inner.size());
  for (auto & ci : inner) {
    Joint * joint = nullptr;
    for (auto & j : joints_) {
      if (ci.get_prefix_name() == j.name &&
        ci.get_interface_name() == hardware_interface::HW_IF_EFFORT)
      {
        joint = &j;
      }
    }
    if (joint) {
      joint->effort_cmd = std::make_unique<hardware_interface::CommandInterface>(std::move(ci));
      out.emplace_back(joint->name, hardware_interface::HW_IF_EFFORT, &joint->net_effort_cmd);
    } else {
      out.push_back(std::move(ci));
    }
  }
  return out;
}

#pragma GCC diagnostic pop

return_type GravityCompensatedGazeboSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return inner_->prepare_command_mode_switch(start_interfaces, stop_interfaces);
}

return_type GravityCompensatedGazeboSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (const auto & name : start_interfaces) {
    for (auto & j : joints_) {
      if (name == j.name + "/" + hardware_interface::HW_IF_EFFORT) {
        j.net_effort_cmd = 0.0;  // no torque jump from a stale command
      }
    }
  }
  return inner_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

void GravityCompensatedGazeboSystem::computeGravity()
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double pos = joints_[i].position->get_optional<double>().value_or(0.0);
    if (model_->nq[i] == 2) {  // continuous joint: cos, sin
      model_->q[model_->idx_q[i]] = std::cos(pos);
      model_->q[model_->idx_q[i] + 1] = std::sin(pos);
    } else {
      model_->q[model_->idx_q[i]] = pos;
    }
  }
  model_->g = pinocchio::computeGeneralizedGravity(model_->model, model_->data, model_->q);
}

return_type GravityCompensatedGazeboSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto ret = inner_->read(time, period);
  if (ret != return_type::OK || !model_) {
    return ret;
  }
  computeGravity();
  if (net_effort_state_) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      const double total = joints_[i].effort ?
        joints_[i].effort->get_optional<double>().value_or(0.0) : 0.0;
      joints_[i].net_effort_state = total - model_->g[model_->idx_v[i]];
    }
  }
  return ret;
}

return_type GravityCompensatedGazeboSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (model_) {
    // g(q) at the state read() just got: q changes little in one cycle.
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i].effort_cmd) {
        const double net = std::isfinite(joints_[i].net_effort_cmd) ?
          joints_[i].net_effort_cmd : 0.0;
        (void)joints_[i].effort_cmd->set_value(net + model_->g[model_->idx_v[i]]);
      }
    }
  }
  return inner_->write(time, period);
}

}  // namespace sharework_gazebo

PLUGINLIB_EXPORT_CLASS(
  sharework_gazebo::GravityCompensatedGazeboSystem, gz_ros2_control::GazeboSimSystemInterface)
