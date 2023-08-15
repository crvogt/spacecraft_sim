#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sc_gazebo_plugins/ROSBaseModelPlugin.hh>

using namespace gazebo;
class ScTorsionalSpring : public ROSBaseModelPlugin
{
  public: ScTorsionalSpring() = default;
  public: virtual ~ScTorsionalSpring() = default;

  public: std::mutex mutex;
  private: std::unique_ptr<ros::NodeHandle> rosnode;
  public: physics::WorldPtr world;
  private: physics::ModelPtr model;
  private: event::ConnectionPtr updateConnection;
  public: physics::JointPtr tJoint;

  public: double kx;
  public: double setPoint;

  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_DEBUG("Loading sc_torsional_spring_plugin");
    this->model = _model;
    this->world = this->model->GetWorld();
    this->rosSensorOutputPub = this->rosNode->advertise<std_msgs::Float32>(this->sensorOutputTopic, 1);

    //Get parameters from SDF
    if(_model->GetJointCount() == 0)
    {
      ROS_ERROR_STREAM("Zero joints found!");
    }
    else
    {
      if(_sdf->HasElement("torsion_joint"))
        this->tJoint = _model->GetJoint(_sdf->Get<std::string>("torsion_joint"));
      else
        ROS_ERROR_STREAM("Must specify a joint for torsional spring...");

      if(_sdf->HasElement("kx"))
        this->kx = _sdf->Get<double>("kx");
      else
        ROS_ERROR_STREAM("Must specify a spring constant...");

      if(_sdf->HasElement("set_point"))
        this->setPoint = _sdf->Get<double>("set_point");
      else
        ROS_ERROR_STREAM("Must specify a set point...");
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ScTorsionalSpring::Update, this));
  }

  protected: virtual void Update()
  {
    double current_angle = this->tJoint->Position(0);
    this->tJoint->SetForce(0, this->kx*(this->setPoint - current_angle));
    ROS_INFO_STREAM("angledif " << (this->setPoint - current_angle) << " force " << this->kx*(this->setPoint-current_angle));
  }
};
GZ_REGISTER_MODEL_PLUGIN(ScTorsionalSpring)
