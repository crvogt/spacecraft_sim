#include <ros/time.h>

#include "sc_gazebo_plugins/sc_reaction_wheel_plugin.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
Wheel::Wheel(ScWheel *_parent)
{
  // Initialize Fields
  this->plugin = _parent;
  this->currCmd = 0.0;
  this->lastCmdTime = this->plugin->world->SimTime();
}

///////////////////////////////////////////////////////////////////////////////
void Wheel::OnTorqueCmd(const std_msgs::Float32::ConstPtr &_msg)
{
  // When we get a new torque command
  ROS_DEBUG_STREAM("New torque command! " << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->lastCmdTime = this->plugin->world->SimTime();
  this->currCmd = _msg->data;
}

///////////////////////////////////////////////////////////////////////////////
double ScWheel::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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

///////////////////////////////////////////////////////////////////////////////
void ScWheel::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_DEBUG("Loading sc_reaction_wheel_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if(_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("Reaction wheel namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  // parse joint publisher update rate
  this->publisherRate = this->SdfParamDouble(_sdf, "publisherRate", 100.0);

  ROS_DEBUG_STREAM("Loading wheels from SDF");

  // For each wheel
  int wheelCounter = 0;
  if(_sdf->HasElement("reaction_wheel"))
  {
    sdf::ElementPtr wheelSDF = _sdf->GetElement("reaction_wheel");
    while(wheelSDF)
    {
      // Instantiate
      Wheel wheel(this);

      ROS_DEBUG_STREAM("reaction wheel #" << wheelCounter);
      
      // Required Parameters
      // find link by name in SDF
      if(wheelSDF->HasElement("linkName"))
      {
        std::string linkName = wheelSDF->Get<std::string>("linkName");
        wheel.link = this->model->GetLink(linkName);
        if(!wheel.link)
        {
          wheel.link_vect = this->model->GetLinks();
          ROS_INFO_STREAM("Links found: ");
          for(int i = 0; i < wheel.link_vect.size(); i++){
            ROS_INFO_STREAM(wheel.link_vect[i]->GetName());
          }
          ROS_ERROR_STREAM("Could not find a link by the name <" << linkName
              << "> in the model");
        }
        else
        {
          ROS_DEBUG_STREAM("Reaction wheel added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each reaction whee;");
      }

      // Parse out wheel joint name
      if(wheelSDF->HasElement("jointName"))
      {
        std::string wheelName = 
          wheelSDF->GetElement("jointName")->Get<std::string>();
        wheel.wheelJoint = this->model->GetJoint(wheelName);
        if(!wheel.wheelJoint)
        {
          ROS_ERROR_STREAM("Could not find a reaction wheel joint by the name" 
             " of <" << wheelName << "> in the model!");
        }
        else
        {
          ROS_DEBUG_STREAM("reaction wheel joint <" << wheelName << 
              "> added to reaction wheel");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No reaction_wheel_jointName SDF parameter for" 
            "reaction wheel #" << wheelCounter);
      }

      // Parse for cmd sub topic
      if(wheelSDF->HasElement("cmdTopic"))
      {
        wheel.cmdTopic = wheelSDF->Get<std::string>("cmdTopic");
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subsscription)"
            "for each reaction wheel");
      }
      
      wheel.maxCmd = this->SdfParamDouble(wheelSDF, "maxCmd", 1.0);
      wheel.maxTorque = this->SdfParamDouble(wheelSDF, "maxTorque", 300.0);

      // Push to vector and increment
      this->wheels.push_back(wheel);
      wheelSDF = wheelSDF->GetNextElement("reaction_wheel");
      wheelCounter++;
    }
  }
  else
  {
    ROS_WARN_STREAM("No 'reaction_wheel' tags in description");
  }
  ROS_DEBUG_STREAM("Found " << wheelCounter << " reaction wheels");

  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  this->prevUpdateTime = this->world->SimTime();

  // Advertise joint state publisher to view reaction wheels
  // in rviz
  this->jointStatePub = 
    this->rosnode->advertise<sensor_msgs::JointState>("joint_states", 1);
  this->jointStateMsg.name.resize(2 * wheels.size());
  this->jointStateMsg.position.resize(2 * wheels.size());
  this->jointStateMsg.velocity.resize(2 * wheels.size());
  this->jointStateMsg.effort.resize(2 * wheels.size());

  for(size_t i = 0; i < this->wheels.size(); ++i)
  {
    // Prefill the joint state message with the wheel joint
    this->jointStateMsg.name[2 * i] = this->wheels[i].wheelJoint->GetName();

    // Sub to commands for each wheel
    this->wheels[i].cmdSub = this->rosnode->subscribe(
        this->wheels[i].cmdTopic, 1, &Wheel::OnTorqueCmd,
        &this->wheels[i]);
  }
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ScWheel::Update, this));
}

double ScWheel::ScaleTorqueCmd(const double _cmd, const double _maxCmd,
                               const double _maxPos) const
{
  double val = 0.0;
  
  if(_cmd > _maxCmd)
    val = _maxCmd;
  else
    val = _cmd; // _maxCmd * _maxPos;

  return val;
}

void ScWheel::Update()
{
  common::Time now = this->world->SimTime();

  for(size_t i = 0;i < this->wheels.size(); ++i)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    // Enforce command timeout
    double dtc = (now - this->wheels[i].lastCmdTime).Double();
    if(dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
    {
      this->wheels[i].currCmd = 0.0;
      ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << i << "] Cmd Timeout");
    }

    // Apply torque mapping
    ignition::math::Vector3d wtorquev(0, 0, 0);
    wtorquev.Z() = this->ScaleTorqueCmd(this->wheels[i].currCmd,
                                        this->wheels[i].maxCmd,
                                        this->wheels[i].maxTorque);

    //ROS_INFO_STREAM("Torque: " << wtorquev.Z());
    // Apply torque for each wheel
    this->wheels[i].link->AddTorque(wtorquev);
  }

  // Publish the joint state
  if(now - this->prevUpdateTime >= (1 / this->publisherRate))
  {
    this->jointStateMsg.header.stamp = ros::Time::now();
    // this->jointStatePub.publish(this->jointStateMsg);

    this->prevUpdateTime = now;
  }
}

GZ_REGISTER_MODEL_PLUGIN(ScWheel);

