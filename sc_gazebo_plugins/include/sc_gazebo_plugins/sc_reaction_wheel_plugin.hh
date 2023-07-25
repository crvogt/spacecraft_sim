#ifndef SC_REACTION_WHEEL_PLUGIN_HH
#define SC_REACTION_WHEEL_PLUGIN_HH

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  // Forward declration of ScWheel class
  class ScWheel;

  // Wheel class
  class Wheel
  {
    // Constructor
    // _parent pointer to an SDF element to parse
    public: explicit Wheel(ScWheel *_parent);

    // Callback for new torque commands
    // _msg the torque message to process
    public: void OnTorqueCmd(const std_msgs::Float32::ConstPtr &_msg);

    // Max abs val of incoming command
    public: double maxCmd;

    // Max torque in Nm
    public: double maxTorque;
    
    // Link where torque is applied
    public: physics::LinkPtr link;

    public: physics::Link_V link_vect;

    // Torque mapping
    public: int mappingType;

    // Topic name for incoming ROS torque commands
    public: std::string cmdTopic;

    // Subscription to torque commands
    public: ros::Subscriber cmdSub;

    // Current or most recent command
    public: double currCmd;
    
    // Last time received a command via ROS topic
    public: common::Time lastCmdTime;

    // Joint controlling the wheel
    public: physics::JointPtr wheelJoint;

    // Plugin parent pointer - for accessing world, etc
    protected: ScWheel *plugin;
  };

  class ScWheel : public ModelPlugin
  {
    // Constructor
    public: ScWheel() = default;

    // Destructor
    public: virtual ~ScWheel() = default;

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf);

    // Callback executed at every physics update
    protected: virtual void Update();

    // Get SDF parameters
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const::std::string &_paramName,
                                   const double _defaultVal) const;

    // Takes ROS drive commands and scales them by max torque
    private: double ScaleTorqueCmd(const double _cmd, 
                                   const double _max_cmd,
                                   const double _max_pos) const;

    // A mutex to protect member variables accessed during
    // OnTorqueCmd and Update
    public: std::mutex mutex;

    // ROS node handler
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    // Pointer to the Gazebo world
    public: physics::WorldPtr world;

    // Pointer to the Gazebo parent model
    private: physics::ModelPtr model;

    // Timeout for receiving drive commands (sec)
    private: double cmdTimeout;

    // Vector of reaction wheel instances
    private: std::vector<Wheel> wheels;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // For publishing to /joint_state
    private: ros::Publisher jointStatePub;

    private: sensor_msgs::JointState jointStateMsg;

    // Update rate of the joint ROS publisher
    private: double publisherRate = 100.0;

    // The last time we publish joints
    private: common::Time prevUpdateTime;
  };
}

#endif
