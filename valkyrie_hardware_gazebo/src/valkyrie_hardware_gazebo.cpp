#include <cassert>
#include <boost/foreach.hpp>

#include <gazebo/sensors/SensorManager.hh>

#include <urdf_parser/urdf_parser.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <valkyrie_hardware_gazebo/valkyrie_hardware_gazebo.h>

//#define ROS_YELLOW_STEAM(x) ROS_INFO_STREAM("\033[1;33" << x << "\033[0m\n")
//#define ROS_YELLOW_STEAM(x) ROS_INFO_STREAM(x)

using std::string;
using std::vector;

namespace valkyrie_hardware_gazebo
{
  using namespace hardware_interface;

  ValkyrieHardwareGazebo::ValkyrieHardwareGazebo()
    : gazebo_ros_control::RobotHWSim()
  {}


  bool ValkyrieHardwareGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle nh,
      gazebo::physics::ModelPtr model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    using gazebo::physics::JointPtr;

    // Cleanup
    eff_sim_joints_.clear();
    sim_joints_.clear();
    jnt_pos_.clear();
    jnt_vel_.clear();
    jnt_eff_.clear();
    jnt_eff_cmd_.clear();

    // Simulation joints
    std::vector<gazebo::physics::JointPtr> sim_joints_tmp = model->GetJoints();

    std::vector<std::string> jnt_names;
    for (size_t i = 0; i < sim_joints_tmp.size(); ++i)
    {
      sim_joints_.push_back(sim_joints_tmp[i]);
      eff_sim_joints_.push_back(sim_joints_tmp[i]);
      jnt_names.push_back(sim_joints_tmp[i]->GetName());
    }

    eff_n_dof_ = eff_sim_joints_.size();
    n_dof_ = sim_joints_.size();

    // Raw data
    jnt_pos_.resize(n_dof_);
    jnt_vel_.resize(n_dof_);
    jnt_eff_.resize(n_dof_);
    jnt_eff_cmd_.resize(eff_n_dof_);

    // Hardware interfaces
    for (size_t i = 0; i < n_dof_; ++i)
    {

      jnt_state_interface_.registerHandle(JointStateHandle(jnt_names[i],
                                                           &jnt_pos_[i],
                                                           &jnt_vel_[i],
                                                           &jnt_eff_[i]));
    }
    for (size_t i = 0; i < eff_n_dof_; ++i)
    {

      jnt_eff_cmd_interface_.registerHandle(JointHandle(jnt_state_interface_.getHandle(jnt_names[i]),
                                                        &jnt_eff_cmd_[i]));
      ROS_INFO_STREAM("\033[1;33 Registered joint '" << jnt_names[i] << "' in the EffortJointInterface. \033[0m\n");
    }
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_eff_cmd_interface_);

    vector<string> eff_joints_with_limits, eff_joints_without_limits;
    for (unsigned int i = 0; i < eff_n_dof_; ++i)
    {
      JointHandle cmd_handle = jnt_eff_cmd_interface_.getHandle(eff_sim_joints_[i]->GetName());
      const string name = cmd_handle.getName();

      using namespace joint_limits_interface;
      boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(name);
      JointLimits limits;
      if (!getJointLimits(urdf_joint, limits) || !getJointLimits(name, nh, limits))
      {
        eff_joints_without_limits.push_back(name);
        continue;
      }
      eff_jnt_limits_interface_.registerHandle(EffortJointSaturationHandle(cmd_handle, limits));
      eff_joints_with_limits.push_back(name);
    }
    if (!eff_joints_with_limits.empty())
    {
      ROS_DEBUG_STREAM("Joint limits will be enforced for effort-controlled joints:" <<
                        containerToString(eff_joints_with_limits, "\n - "));
    }
    if (!eff_joints_without_limits.empty())
    {
      ROS_WARN_STREAM("Joint limits will not be enforced for effort-controlled joints:" <<
                      containerToString(eff_joints_without_limits, "\n - "));
    }


    // Hardware interfaces: Base IMU sensors
    const string imu_name = "pelvis_imu";
    imu_sensor_  = (gazebo::sensors::ImuSensor*)gazebo::sensors::SensorManager::Instance()->GetSensor(imu_name+"_sensor").get(); // TODO: Fetch from URDF?
    if (!this->imu_sensor_)
    {
      ROS_ERROR_STREAM("Could not find base IMU sensor.");
      return false;
    }

    ImuSensorHandle::Data data;
    data.name     = imu_name;           // TODO: Fetch from elsewhere?
    data.frame_id = "pelvis"; // TODO: Fetch from URDF?
    data.orientation = &base_orientation_[0];
    imu_sensor_interface_.registerHandle(ImuSensorHandle(data));
    registerInterface(&imu_sensor_interface_);
    ROS_DEBUG_STREAM("Registered IMU sensor.");

    return true;
  }

  void ValkyrieHardwareGazebo::readSim(ros::Time time, ros::Duration period)
  {
    for(unsigned int j = 0; j < n_dof_; ++j)
    {
      // Gazebo has an interesting API...
      jnt_pos_[j] += angles::shortest_angular_distance
          (jnt_pos_[j], sim_joints_[j]->GetAngle(0u).Radian());
      jnt_vel_[j] = sim_joints_[j]->GetVelocity(0u);
      jnt_eff_[j] = sim_joints_[j]->GetForce(0u);
    }

    // Read IMU sensor
    gazebo::math::Quaternion imu_quat = imu_sensor_->GetOrientation();
    base_orientation_[0] = imu_quat.x;
    base_orientation_[1] = imu_quat.y;
    base_orientation_[2] = imu_quat.z;
    base_orientation_[3] = imu_quat.w;

    gazebo::math::Vector3 imu_ang_vel = imu_sensor_->GetAngularVelocity();
    base_ang_vel_[0] = imu_ang_vel.x;
    base_ang_vel_[1] = imu_ang_vel.y;
    base_ang_vel_[2] = imu_ang_vel.z;

    gazebo::math::Vector3 imu_lin_acc = imu_sensor_->GetLinearAcceleration();
    base_lin_acc_[0] =  imu_lin_acc.x;
    base_lin_acc_[1] =  imu_lin_acc.y;
    base_lin_acc_[2] =  imu_lin_acc.z;
  }

  void ValkyrieHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits
    eff_jnt_limits_interface_.enforceLimits(period);

    // Send commands
    for(unsigned int i = 0; i < eff_n_dof_; ++i)
    {
      eff_sim_joints_[i]->SetForce(0u, jnt_eff_cmd_[i]);
    }
  }

} // valkyrie_hardware_gazebo

PLUGINLIB_EXPORT_CLASS( valkyrie_hardware_gazebo::ValkyrieHardwareGazebo, gazebo_ros_control::RobotHWSim)
