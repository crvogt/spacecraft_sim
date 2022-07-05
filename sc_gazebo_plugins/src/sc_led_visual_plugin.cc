#include <mutex>

#include <ignition/math/Color.hh>
#include <ros/time.h>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include "sc_gazebo_plugins/sc_led_visual_plugin.hh"

namespace gazebo
{
  class ScLedPluginPrivate
  {
    // Visual whose color will be changed
    public: rendering::VisualPtr visual;

    // Connects to rendering update event
    public: event::ConnectionPtr updateConnection;

    public: ignition::math::Color colorA;

    public: ignition::math::Color colorB;

    public: common::Time period;

    public: common::Time cycleStartTime;

    public: common::Time currentSimTime;

    public: transport::NodePtr node;

    public: std::mutex mutex;

    public: bool useWallTime;

    public: transport::SubscriberPtr infoSub;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ScLedPlugin)

ScLedPlugin::ScLedPlugin() : dataPtr(new ScLedPluginPrivate)
{
}

ScLedPlugin::~ScLedPlugin()
{
  this->dataPtr->infoSub.reset();
  if(this->dataPtr->node)
    this->dataPtr->node->Fini();
}

void ScLedPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if(!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
    return;
  }
  this->dataPtr->visual = _visual;
  // Get color A
  this->dataPtr->colorA.Set(1, 0, 0, 1);
  if(_sdf->HasElement("color_a"))
    this->dataPtr->colorA = _sdf->Get<ignition::math::Color>("color_a");
  // Get color B
  this->dataPtr->colorB.Set(0, 0, 0, 1);
  if(_sdf->HasElement("color_b"))
    this->dataPtr->colorB = _sdf->Get<ignition::math::Color>("color_b");

  // Get the period
  this->dataPtr->period.Set(1);
  if(_sdf->HasElement("period"))
    this->dataPtr->period = _sdf->Get<double>("period");

  if(this->dataPtr->period <= 0)
  {
    gzerr << "Period can't be lower than zero" << std::endl;
    return;
  }

  // Get whether to use wall time or sim time
  this->dataPtr->useWallTime = false;
  if(_sdf->HasElement("use_wall_time"))
    this->dataPtr->useWallTime = _sdf->Get<bool>("use_wall_time");

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectPreRender(
      std::bind(&ScLedPlugin::Update, this));

  //Subscribe to world statistics to get sim time
  if(!this->dataPtr->useWallTime)
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();

    this->dataPtr->infoSub = this->dataPtr->node->Subscribe(
        "~/pose/local/info", &ScLedPlugin::OnInfo, this);
  }
}

void ScLedPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if(!this->dataPtr->visual)
  {
    gzerr << "The visual is null" << std::endl;
    return;
  }

  common::Time currentTime;
  if(this->dataPtr->useWallTime)
    currentTime = common::Time::GetWallTime();
  else
    currentTime = this->dataPtr->currentSimTime;

  if(this->dataPtr->cycleStartTime == common::Time::Zero ||
      this->dataPtr->cycleStartTime > currentTime)
  {
    this->dataPtr->cycleStartTime = currentTime;
  }

  auto elapsed = currentTime - this->dataPtr->cycleStartTime;

  // Restart cycle
  if(elapsed >= this->dataPtr->period)
    this->dataPtr->cycleStartTime = currentTime;

  ignition::math::Color from;
  ignition::math::Color to;
  // Color A -> B
  if(elapsed < this->dataPtr->period*0.5)
  {
    from = this->dataPtr->colorA;
    to = this->dataPtr->colorB;
  }
  // Color B -> A
  else
  {
    from = this->dataPtr->colorB;
    to = this->dataPtr->colorA;
    elapsed -= this->dataPtr->period*0.5;
  }

  // interpolate each color component
  double pos = (elapsed/(this->dataPtr->period*0.5)).Double();

  double red = from.R() + (to.R() - from.R()) * pos;
  double green = from.G() +(to.G() - from.G()) * pos;
  double blue = from.B() + (to.B() - from.B()) * pos;
  double alpha = from.A() + (to.A() - from.A()) * pos;
  
  ignition::math::Color color(red, green, blue, alpha);

  this->dataPtr->visual->SetDiffuse(color);
  this->dataPtr->visual->SetAmbient(color);
  this->dataPtr->visual->SetTransparency(1-color.A());
}

void ScLedPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
}
