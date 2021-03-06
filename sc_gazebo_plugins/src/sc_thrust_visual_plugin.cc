#include <mutex>

#include <ignition/math/Color.hh>
#include <ros/time.h>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

#include "sc_gazebo_plugins/sc_thrust_visual_plugin.hh"

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ScThrustVisualPlugin)

ScThrustVisualPlugin::ScThrustVisualPlugin() : 
  dataPtr(new ScThrustVisualPluginPrivate)
{
}

ScThrustVisualPlugin::~ScThrustVisualPlugin()
{
  this->dataPtr->infoSub.reset();
  if(this->dataPtr->node)
    this->dataPtr->node->Fini();
}

void ScThrustVisualPlugin::Load(rendering::VisualPtr _visual, 
    sdf::ElementPtr _sdf)
{
  if(!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load" << 
      std::endl;
    return;
  }

  this->dataPtr->visual = _visual;
  
  //Get the color
  if(_sdf->HasElement("thrust_color"))
  {
    this->dataPtr->thrustColor = 
      _sdf->Get<ignition::math::Color>("thrust_color");
    this->dataPtr->emptyColor = ignition::math::Color(1, 0, 0, 0);
  }
  else
    gzerr << "No thrust_color specified" << std::endl;

  // Get the thruster number
  if(_sdf->HasElement("thruster_number"))
  {
    this->dataPtr->thrusterNum = 
      _sdf->Get<std::string>("thruster_number");
  }
  else
    gzerr << "No thruster_number specified" << std::endl;

  this->dataPtr->period.Set(1);
  if(_sdf->HasElement("period"))
    this->dataPtr->period = _sdf->Get<double>("period");

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectPreRender(
      std::bind(&ScThrustVisualPlugin::Update, this));

  // Subscribe to world statistics to get sim time
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->infoSub = this->dataPtr->node->Subscribe(
      "~/pose/local/info", &ScThrustVisualPlugin::OnInfo, this);

  // Ros node 
  std::string thrustSubTopic = "/polysat/thrusters/thruster_" + 
    this->dataPtr->thrusterNum;
  this->rosnode.reset(new ros::NodeHandle("thrust_visual_node"));
  this->dataPtr->thrustSub = this->rosnode->subscribe(
      thrustSubTopic, 1, 
      &ScThrustVisualPlugin::OnThrustInfo, 
      this);
}

void ScThrustVisualPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if(!this->dataPtr->visual)
  {
    gzerr << "The visual is null" << std::endl;
    return;
  }

  common::Time currentTime;
  currentTime = this->dataPtr->currentSimTime;
  
  double alpha = this->dataPtr->thrustVal / 5.0;
  if(alpha > 1.0)
    alpha = 1.0;
  // TODO change thruster output to only ever be positive...
  else if(alpha < 0.0)
    alpha = 0.0;
  double red = this->dataPtr->thrustColor.R();
  double blue = this->dataPtr->thrustColor.B();
  double green = this->dataPtr->thrustColor.G();

  ignition::math::Color color(red, green, blue, alpha);

  this->dataPtr->visual->SetDiffuse(color);
  this->dataPtr->visual->SetAmbient(color);
  this->dataPtr->visual->SetTransparency(1-color.A());
}

void ScThrustVisualPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
}

void ScThrustVisualPlugin::OnThrustInfo(const std_msgs::Float32::ConstPtr 
    &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->thrustVal = _msg->data;
}
