#ifndef SC_THRUST_VISUAL_PLUGIN_HH
#define SC_THRUST_VISUAL_PLUGIN_HH

//#include <memory>
#include <gazebo/common/Plugin.hh>
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

    private: void Update();
    
    // Callback to receive info
    private: void OnInfo(ConstPosesStampedPtr &_msg);

    private: std::unique_ptr<ScThrustVisualPluginPrivate> dataPtr;
  };
}
#endif
