#ifndef SC_THRUST_VISUAL_PLUGIN_HH
#define SC_THRUST_VISUAL_PLUGIN_HH

//#include <memory>
#include <gazebo/common/Plugin.hh>
#include <std_msgs/Float32.h>
/*Will need to add the visual model for thrusters into thruster link
 *get link name for each thruster
 *subscribe to thruster value for each thruster
 *change color/alpha value for each thruster
 *
 *
 * thought: could create a little thruster nozzle, then include the thrust
 * cloud coming from each nozzle... food for thought
 */
namespace gazebo
{
  class ScThrustVisualPluginPrivate;

  class GZ_PLUGIN_VISIBLE ScThrustVisualPlugin : public VisualPlugin
  {
    public: ScThrustVisualPlugin();

    public: ~ScThrustVisualPlugin();

    public: virtual void Load(rendering::VisualPtr _visual, 
                sdf::ElementPtr _sdf);

    public: std::string thrustTopic;
     
    private: void Update();
    
    // Callback to receive info
    private: void OnInfo(ConstPosesStampedPtr &_msg);

    private: std::unique_ptr<ros::NodeHandle> rosnode;

    private: void OnThrustInfo(const std_msgs::Float32::ConstPtr &_msg);

    private: std::unique_ptr<ScThrustVisualPluginPrivate> dataPtr;
  };

  class ScThrustVisualPluginPrivate
  {
    //Visual whose color will be changed
    public:rendering::VisualPtr visual;

    // Connects to rendering update event
    public: event::ConnectionPtr updateConnection;

    public: ignition::math::Color thrustColor;

    public: ignition::math::Color emptyColor;

    public: common::Time period;

    public: common::Time cycleStartTime;
            
    public: common::Time currentSimTime;

    public: transport::NodePtr node;

    public: std::mutex mutex;

    public: std::string thrusterNum;

    public: transport::SubscriberPtr infoSub;
    
    public: ros::Subscriber thrustSub;

    public: double thrustVal;
  };
}
#endif
