#ifndef SC_GAZEBO_PLUGINS_THRUST_HH
#define SC_GAZEBO_PLUGINS_THRUST_HH

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
// #include <memory>
#include <string>
#include <vector>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
	// Forward declaration of ScThrust class
	class ScThrust;

	// Thruster class
	class Thruster
	{
		// Constructor
		// _parent pointer to an SDF element to parse
		public: explicit Thruster(ScThrust *_parent);

		// Callback for new thrust commands
		// _msg the thrust message to process
		public: void OnThrustCmd(const std_msgs::Float32::ConstPtr &_msg);

    // Callback for new thrust angle
    // _msg the thrust angle to process
    public: void OnThrustAngle(const std_msgs::Float32::ConstPtr &_msg);

		// Maximum abs val of incoming command
		public: double maxCmd;

		// Max forward force in Newtons
		public: double maxForceFwd;

		// Max reverse force in Newtons
		public: double maxForceRev;

		// Max abs val of angle
		// TODO check you might not want this...
		public: double maxAngle;

		// Link where thrust force is applied
		public: physics::LinkPtr link;

		// Thruster mapping (0=linear; 1=GLF, nonlinear)
		public: int mappingType;

		// Topic name for incoming ROS thruster commands
		public: std::string cmdTopic;

		// Subscription to thruster commands
		public: ros::Subscriber cmdSub;

		// If true, thruster will have adjustable angle
		public: bool enableAngle;

		// Topic name for incoming ROS thruster angle commands
		public: std::string angleTopic;

		// Subscription to thruster angle commands
		public: ros::Subscriber angleSub;

		// Current or most recent command
		public: double currCmd;

		// Most recent desired angle
		public: double desiredAngle;

		// Last time received a command via ROS topic
		public: common::Time lastCmdTime;

		// Last time of angle update
		public: common::Time lastAngleUpdateTime;

		// Joint controlling the thruster 
		public: physics::JointPtr thrusterJoint;

		// PID for thruster joint angle
		public: common::PID thrusterJointPID;

		// Plugin parent point - for accessing world, etc.
		protected: ScThrust *plugin;
	};

	class ScThrust : public ModelPlugin
	{
		// Constructor
		public: ScThrust() = default;

		// Destructor
		public: virtual ~ScThrust() = default;

		// Documentation inherited
		public: virtual void Load(physics::ModelPtr _parent,
															sdf::ElementPtr _sdf);

		// Callback executed at every physics update
		protected: virtual void Update();

		// Convenience function for getting SDF parameters
		// _sdfPtr pointer to an SDF element to parse
		// _paramName the name of the element to parse
		// _defaultVal the default value returned if the element doesnt exist
		// return the value parsed
		private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
																	 const::std::string &_paramName,
																	 const double _defaultVal) const;

		// Takes ROS Drive commands and scales them by max thrust
		// _cmd ROS drive command
		// return Value scaled and saturated
		private: double ScaleThrustCmd(const double _cmd,
																	 const double _max_cmd,
																	 const double _max_pos,
																	 const double _max_neg) const;

		// Generalized ligistic function (GLF) used for non-linear
		// thrust model
		// _x Independent variable (input) of GLF
		// _A Lower asymptote
		// _K Upper asymptote
		// _B Growth rate
		// _v Affects near which asymptote max growth occurs
		// _C Typically near 1.0
		// _M Offset to input
		private: double Glf(const double _x,
											  const float _A,
											  const float _K,
											  const float _B,
											  const float _v,
											  const float _C,
											  const float _M) const;

		// Uses GLF function to map thrust command to thruster force 
		// in newtons
		private: double GlfThrustCmd(const double _cmd,
																 const double _maxPos,
																 const double _maxNeg) const;

		// Rotate thruster using thruster joint PID
		private: void RotateThruster(size_t _i,
															 common::Time _stepTime);

		// A mutex to protect member variables accessed during 
		// OnThrustCmd and Update
		public: std::mutex mutex;

		// Ros node handler
		private: std::unique_ptr<ros::NodeHandle> rosnode;

		// Pointer to the Gazebo world, retrieved when the model is loaded
		public: physics::WorldPtr world;

		// Pointer to the Gazebo parent model, retrieved when the model
		// is loaded
		private: physics::ModelPtr model;

		// Timeout for receiving Drive commands (sec)
		private: double cmdTimeout;

		// Vector of thruster instances
		private: std:: vector<Thruster> thrusters;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		// For publishing to /joint_state with prop state
		private: ros::Publisher jointStatePub;

		// The prop message state
		private: sensor_msgs::JointState jointStateMsg;

		// Update rate of the joint ROS publisher
		private: double publisherRate = 100.0;

		// The last time we publish joints
		private: common::Time prevUpdateTime;
	};
}

#endif
