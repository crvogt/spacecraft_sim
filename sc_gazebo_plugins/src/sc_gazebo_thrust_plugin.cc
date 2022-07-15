#include <boost/algorithm/clamp.hpp>
#include <ros/time.h>

#include <cmath>
#include <functional>

#include "sc_gazebo_plugins/sc_gazebo_thrust_plugin.hh"

#include <iostream>

using namespace gazebo;

Thruster::Thruster(ScThrust *_parent)
{
	// Initialize fields
	this->plugin = _parent;
	this->thrusterJointPID.Init(300, 0.0, 20);
	this->currCmd = 0.0;
	this->desiredAngle = 0.0;

	#if GAZEBO_MAJOR_VERSION >= 8
		this->lastCmdTime = this->plugin->world->SimTime();
	#else
		this->lastCmdTime = this->plugin->world->GetSimTime();
	#endif
}

void Thruster::OnThrustCmd(const std_msgs::Float32::ConstPtr &_msg)
{
	// When we get a new thrust command...
	ROS_DEBUG_STREAM("New thrust command! " << _msg->data);
	std::lock_guard<std::mutex> lock(this->plugin->mutex);
	#if GAZEBO_MAJOR_VERSION >= 8
		this->lastCmdTime = this->plugin->world->SimTime();
	#else
		this->lastCmdTime = this->plugin->world->GetSimTime();
	#endif
		this->currCmd = _msg->data;
}

void Thruster::OnThrustAngle(const std_msgs::Float32::ConstPtr &_msg)
{
	// When we get a new thrust angle
	ROS_DEBUG_STREAM("New thrust angle! " << _msg->data);
	std::lock_guard<std::mutex> lock(this->plugin->mutex);
	this->desiredAngle = boost::algorithm::clamp(_msg->data, -this->maxAngle,
																							 this->maxAngle);
}

double ScThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr,
const std::string &_paramName, const double _defaultVal) const
{
	if (!_sdfPtr->HasElement(_paramName))
	{
		ROS_INFO_STREAM("Parameter <" << _paramName << ">not found: "
										"Using default value of <" << _defaultVal << ">.");
		return _defaultVal;
	}

	double val = _sdfPtr->Get<double>(_paramName);
	ROS_DEBUG_STREAM("Parameter found - setting <" << _paramName <<
									 "> to <" << val << ">.");
	return val;
}

void ScThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	ROS_DEBUG("Loading sc_gazebo_thrust_plugin");
	this->model = _parent;
	this->world = this->model->GetWorld();

	// Get parameters from SDF
	std::string nodeNamespace = "";
	if(_sdf->HasElement("robotNamespace"))
	{
		nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
		ROS_INFO_STREAM("Thruster namespace <" << nodeNamespace << ">");
	}

	this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

	// Parse joint publisher update rate
  	this->publisherRate = this->SdfParamDouble(_sdf, "publisherRate", 100.0);

	ROS_DEBUG_STREAM("Loading thrusters from SDF");

	// for each thruster
	int thrusterCounter = 0;
	if(_sdf->HasElement("thruster"))
	{
		sdf::ElementPtr thrusterSDF = _sdf->GetElement("thruster");
		while(thrusterSDF)
		{
			// Instantiate
			Thruster thruster(this);

			ROS_DEBUG_STREAM("thruster #" << thrusterCounter);

			// Required Parameters
			// find link by name in SDF
			if(thrusterSDF->HasElement("linkName"))
			{
				std::string linkName = thrusterSDF->Get<std::string>("linkName");
				thruster.link = this->model->GetLink(linkName);
				if(!thruster.link)
				{
          			thruster.link_vect = this->model->GetLinks();
          			ROS_INFO_STREAM("Links found: ");
         			for(int i = 0; i < thruster.link_vect.size(); i++){
            			ROS_INFO_STREAM(thruster.link_vect[i]->GetName());
          			}
					ROS_ERROR_STREAM("Could not find a link by the name <" << linkName
						<< "> in the model");
				}
				else
				{
					ROS_DEBUG_STREAM("Thruster added to link <" << linkName << ">");
				}					
			}
			else
			{
				ROS_ERROR_STREAM("Please specify a link name for each thruster");
			}

			// Parse out thruster joint name
			if(thrusterSDF->HasElement("jointName"))
			{
				std::string thrusterName = 
					thrusterSDF->GetElement("jointName")->Get<std::string>();
				thruster.thrusterJoint = this->model->GetJoint(thrusterName);
				if(!thruster.thrusterJoint)
				{
					ROS_ERROR_STREAM("Could not find a thruster joint by the name of <" <<
						thrusterName << "> in the model!");
				}
				else
				{
					ROS_DEBUG_STREAM("thruster joint <" << thrusterName <<
						"> added to thruster");
				}
			}
			else
			{
				ROS_ERROR_STREAM("No thrusterJointName SDF parameter for thruster #"
					<< thrusterCounter);
			}

			// Parse for cmd sub topic
			if(thrusterSDF->HasElement("cmdTopic"))
			{
				thruster.cmdTopic = thrusterSDF->Get<std::string>("cmdTopic");
			}
			else
			{
				ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
					"for each thruster");
			}

			// Parse for angle subscription
			if(thrusterSDF->HasElement("angleTopic"))
			{
				thruster.angleTopic = thrusterSDF->Get<std::string>("angleTopic");
			}
			else{
				ROS_ERROR_STREAM("Please specify an angleTopic (for ROS sub) "
					"for each thruster");
			}

			// Parse for enableAngle bool
			if(thrusterSDF->HasElement("enableAngle"))
			{
				thruster.enableAngle = thrusterSDF->Get<bool>("enableAngle");
			}
			else
			{
				ROS_ERROR_STREAM("Please specify for each thruster if it should enable "
					"angle adjustment (for ROS sub)");
			}

			// Optional params
			// parse individual thruster SDF params
			if(thrusterSDF->HasElement("mappingType"))
			{
				thruster.mappingType = thrusterSDF->Get<int>("mappingType");
				ROS_DEBUG_STREAM("Param found - setting <mappingType> to <" <<
					thruster.mappingType << ">.");
			}
			else
			{
				thruster.mappingType = 0;
				ROS_INFO_STREAM("Param <mappingType> not found: "
					"Using default value of <" << thruster.mappingType << ">.");
			}

			thruster.maxCmd = this->SdfParamDouble(thrusterSDF, "maxCmd", 1.0);
			thruster.maxForceFwd = 
				this->SdfParamDouble(thrusterSDF, "maxForceFwd", 500.0);
			thruster.maxForceRev = 
				this->SdfParamDouble(thrusterSDF, "maxForceRev", -100.0);
			thruster.maxAngle = this->SdfParamDouble(thrusterSDF, "maxAngle", 
																							 M_PI / 2);

			// Push to vector and increment
			this->thrusters.push_back(thruster);
			thrusterSDF = thrusterSDF->GetNextElement("thruster");
			thrusterCounter++;
		}
	}
	else
	{
		ROS_WARN_STREAM("No 'thruster' tags in description - how will you move?");
	}
	ROS_DEBUG_STREAM("Found " << thrusterCounter << " thrusters");

	// Initialize the ROS node and subscribe to cmd_drive
	this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

	#if GAZEBO_MAJOR_VERSION >= 8
		this->prevUpdateTime = this->world->SimTime();
	#else
		this->prevUpdateTime = this->world->GetSimTime();
	#endif

	// Advertise joint state publisher to view thruster and propellers in rviz
	// Consider throttling joint_state_pub for performance
	// every onupdate() may be too frequent
	this->jointStatePub = 
		this->rosnode->advertise<sensor_msgs::JointState>("joint_states", 1);
	this->jointStateMsg.name.resize(2 * thrusters.size());
	this->jointStateMsg.position.resize(2 * thrusters.size());
	this->jointStateMsg.velocity.resize(2 * thrusters.size());
	this->jointStateMsg.effort.resize(2 * thrusters.size());

	for(size_t i = 0; i < this->thrusters.size(); ++i)
	{
		// Prefill the joint state message with the thruster joint
		this->jointStateMsg.name[2 * i] = this->thrusters[i].thrusterJoint->GetName();

		// Sub to commands for each thrusters
		this->thrusters[i].cmdSub = this->rosnode->subscribe(
			this->thrusters[i].cmdTopic, 1, &Thruster::OnThrustCmd,
			&this->thrusters[i]);

		// Sub to angles for each thruster
		if(this->thrusters[i].enableAngle)
		{
			this->thrusters[i].angleSub = this->rosnode->subscribe(
				this->thrusters[i].angleTopic, 1, &Thruster::OnThrustAngle,
				&this->thrusters[i]);
		}
	}
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		std::bind(&ScThrust::Update, this));
}

double ScThrust::ScaleThrustCmd(const double _cmd, const double _maxCmd,
	const double _maxPos) const
{
	double val = 0.0;
	if(_cmd >= 0.0)
	{
		val = _cmd / _maxCmd * _maxPos;
		val = std::min(val, _maxPos);
	}
  else
  {
    val = 0;
  }
	return val;
}

double ScThrust::Glf(const double _x, const float _A, const float _K,
	const float _B, const float _v, const float _C, const float _M) const
{
	return _A + (_K - _A) / (pow(_C + exp(-_B * (_x - _M)), 1.0 / _v));
}

double ScThrust::GlfThrustCmd(const double _cmd,
															const double _maxPos) const
{
	double val = 0.0;
	if(_cmd > 0.01)
	{
		val = this->Glf(_cmd, 0.01f, 59.82f, 5.0f, 0.38f, 0.56f, 0.28f);
		val = std::min(val, _maxPos);
	}
  else if(_cmd < 0.01)
  {
    val = 0;
  }
	return val;
}				


void ScThrust::Update()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    common::Time now = this->world->SimTime();
  #else
    common::Time now = this->world->GetSimTime();
  #endif

  for(size_t i = 0; i < this->thrusters.size(); ++i)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      // Enforce command timeout
      double dtc = (now - this->thrusters[i].lastCmdTime).Double();
      if(dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
      {
        this->thrusters[i].currCmd = 0.0;
        ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << i << "] Cmd Timeout");
      }

      // Adjust thruster joint angle with PID
      this->RotateThruster(i, now - this->thrusters[i].lastAngleUpdateTime);

      // Apply the thrust mapping
      ignition::math::Vector3d tforcev(0, 0, 0);
      switch(this->thrusters[i].mappingType)
      {
        case 0:
          tforcev.X() = this->ScaleThrustCmd(this->thrusters[i].currCmd/
                                             this->thrusters[i].maxCmd,
                                             this->thrusters[i].maxCmd,
                                             this->thrusters[i].maxForceFwd);
          break;
        case 1:
          tforcev.X() = this->GlfThrustCmd(this->thrusters[i].currCmd/
                                           this->thrusters[i].maxCmd,
                                           this->thrusters[i].maxForceFwd);
          break;
        default:
          ROS_FATAL_STREAM("Cannot use mappingType=" <<
              this->thrusters[i].mappingType);
          break;
      }

      // Apply force for each thruster
      this->thrusters[i].link->AddLinkForce(tforcev);
    }
  }

  // Publish the joint state
  if(now - this->prevUpdateTime >= (1 / this->publisherRate))
  {
    this->jointStateMsg.header.stamp = ros::Time::now();
    //this->jointStatePub.publish(this->jointStateMsg);

    this->prevUpdateTime = now;
  }
}

void ScThrust::RotateThruster(size_t _i, common::Time _stepTime)
{
  // Calculate angleError for PID calculation
  double desiredAngle = this->thrusters[_i].desiredAngle;
  #if GAZEBO_MAJOR_VERSION >= 8
    double currAngle = this->thrusters[_i].thrusterJoint->Position(0);
  #else
    double currAngle = this->thrusters[_i].thrusterJoint->GetAngle(0).Radian();
  #endif
  double angleError = currAngle - desiredAngle;

  double effort = this->thrusters[_i].thrusterJointPID.Update(angleError, 
                                                            _stepTime);
  this->thrusters[_i].thrusterJoint->SetForce(0, effort);

  // Set position, velocity, and effort of joint from gazebo
  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Angle position = 
      this->thrusters[_i].thrusterJoint->Position(0);
  #else
    gazebo::math::Angle position = this->thrusters[_i].thrusterJoint->GetAngle(0);
  #endif
  position.Normalize();
  this->jointStateMsg.position[2 * _i] = position.Radian();
  this->jointStateMsg.velocity[2 * _i] = 
    this->thrusters[_i].thrusterJoint->GetVelocity(0);
  this->jointStateMsg.effort[2 * _i] = effort;

  //Store last update time
  this->thrusters[_i].lastAngleUpdateTime += _stepTime;
}

GZ_REGISTER_MODEL_PLUGIN(ScThrust);
