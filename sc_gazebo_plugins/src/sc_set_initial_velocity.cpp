/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/math.hh>
// Might just need this for the headers
#include "sc_gazebo_plugins/sc_set_initial_velocity.hh"
#include <ros/ros.h>
#include <vector>
//#include "ode_perfect_linear.hh"
//#include "ode_perfect_angular.hh"
//#include "ode_perfect_velocity.hh"
//#include "pid_link.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
//#include <gazebo/math/Vector3.hh>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>


namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief Sets velocity on a link or joint
  class SetLinkVelocityPlugin : public ModelPlugin
  {
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        this->world = this->model->GetWorld();
        //this->angularVel = math::Vector3(0, 0, 0);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
        rosNode.reset(new ros::NodeHandle("wheel_torque"));
        torque_publisher = rosNode->advertise<std_msgs::Float32>("wheel_torque", 1);

        // Init torque_vec
        torque_applied.X() = 0.0;
        torque_applied.Y() = 0.0;
        torque_applied.Z() = 0.0;
      }

    public: void Update(const common::UpdateInfo &_info)
      {
        if (update_num >= 10000 && update_num < 10200)
        {
          if(update_num == 10000){
            ROS_INFO_STREAM("SET VELOCITY");
            ReadTorqueTime();
          }
          
          // Model angle, will not change
          double theta = 0;
          // Model angular vel
          //double theta_dot = 0.0349066;
          double theta_dot = 0.0;
          // Mass joint angle
          //double theta_2 = 0.0872665;
          double theta_2 = 0.0;
          // Mass joint angular vel
          //double theta_2_dot = 0.0;
          double theta_2_dot = 0.0349066;

          ignition::math::Vector3 theta_dot_vec(0.0, 0.0, theta_dot);
          ignition::math::Vector3 theta_2_dot_vec(0.0, 0.0, theta_2_dot);
          // Link velocity instantaneously without applying forces
          //model->GetLink("prime/prime_link")->SetAngularVel({0, 0, theta_dot});
          // Sets model angular vel, theta_dot
          model->SetAngularVel(theta_dot_vec);
          //Sets mass joint angle, theta_2
          model->GetJoint("prime/torsion_spring_joint")->SetPosition(0, theta_2);
          // Sets mass joint angular vel
          model->GetJoint("prime/torsion_spring_joint")->SetVelocity(0, theta_2_dot);
          //model->GetLink("prime/mass_bar")->SetAngularVel(theta_2_dot_vec);

          
        }
        else if(update_num >= 10200){
          if(update_num == 10200){
            startTime = this->world->SimTime();
          }
          else{
            // Add torque profile here
            curTime = this->world->SimTime() - startTime;

            if(curTime > time_vec[counter]){
              counter++;
              //torque_applied.Z() = torque_vec[counter];
            }
            else if(curTime > time_vec.back()){
              torque_applied.Z() = 0.0;
              ROS_INFO_STREAM("update num: " << update_num);
            }           
            else{
              //torque_applied.Z() = torque_vec[counter];
            }
            wheel_torque_val.data = torque_applied.Z();
            torque_publisher.publish(wheel_torque_val);
            // Apply torque to link
            ROS_INFO_STREAM("Updater: " << update_num << " torque: " << torque_applied.Z());
            model->GetLink("prime/base_link")->AddTorque(torque_applied);

          }
        }

        update_num++;
      }

    public: void ReadTorqueTime(void)
      {
        //std::ifstream read_torque("/home/carson/sc_ws/src/spacecraft_sim/sc_utils/data/paper_files/Paper1_Ex2/myFile_nonRest_to_Rest_time_torque_cvmod.txt");
        std::ifstream read_torque("/home/carson/sc_ws/src/spacecraft_sim/sc_utils/data/paper_files/Paper1_Ex2/myFile_nonRest_to_Rest_time_torque.txt");
        //std::ifstream read_torque("/home/carson/sc_ws/src/spacecraft_sim/sc_utils/data/paper_files/Paper1_Ex1/myFile_ResttoRest_time_torque.txt");
        //std::ifstream read_torque("/home/carson/sc_ws/src/spacecraft_sim/sc_utils/data/paper_files/Paper2_Ex1/ZVDD_ex1_RtR2.txt");
        std::string out_line;
        char split_char = '\t';

        while(getline(read_torque, out_line))
        {
          std::istringstream split(out_line);
          std::vector<std::string> tokens;
          for(std::string each; std::getline(split, each, split_char); tokens.push_back(each));
          time_vec.push_back(std::stof(tokens[0]));
          torque_vec.push_back(std::stof(tokens[1]));
        }
      }

    /// \brief a pointer to the model this plugin was loaded by
    public: physics::ModelPtr model;
    /// \brief object for callback connection
    public: event::ConnectionPtr updateConnection;
    /// \brief number of updates received
    public: int update_num = 0;
    public: int counter = 0;
    /// \brief vector for torque
    public: std::vector<float> torque_vec;
    /// \brief vector for time
    public: std::vector<float> time_vec;
    protected: physics::WorldPtr world;
    public: common::Time curTime; 
    public: common::Time startTime; 
    public: ignition::math::Vector3d torque_applied;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    protected: ros::Publisher torque_publisher;
    private: std_msgs::Float32 wheel_torque_val;
  };

  GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
}
