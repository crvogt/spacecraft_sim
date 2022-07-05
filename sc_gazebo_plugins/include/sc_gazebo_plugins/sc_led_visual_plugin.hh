#ifndef SC_GAZEBO_PLUGINS_LED_HH
#define SC_GAZEBO_PLUGINS_LED_HH

#include <memory>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class ScLedPluginPrivate;

  class GZ_PLUGIN_VISIBLE ScLedPlugin : public VisualPlugin
  {
    public: ScLedPlugin();

    public: ~ScLedPlugin();

    public: virtual void Load(rendering::VisualPtr _visual, 
         sdf::ElementPtr _sdf);

    private: void Update();

    // Callback to receive info
    private: void OnInfo(ConstPosesStampedPtr &_msg);

    private: std::unique_ptr<ScLedPluginPrivate> dataPtr;
  };
}
#endif
