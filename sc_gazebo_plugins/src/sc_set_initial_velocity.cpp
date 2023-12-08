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
// Might just need this for the headers
#include "sc_gazebo_plugins/sc_set_initial_velocity.hh"
#include <ros/ros.h>
//#include "ode_perfect_linear.hh"
//#include "ode_perfect_angular.hh"
//#include "ode_perfect_velocity.hh"
//#include "pid_link.hh"


namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief Sets velocity on a link or joint
  class SetLinkVelocityPlugin : public ModelPlugin
  {
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
      }

    public: void Update(const common::UpdateInfo &_info)
      {
        if (update_num >= 20000 && update_num < 20200)
        {
          if(update_num == 20000){
            ROS_INFO_STREAM("SET VELOCITY");
          }
          // Link velocity instantaneously without applying forces
          model->GetLink("prime/base_link")->SetAngularVel({0, 0, 1});
          model->GetLink("prime/mass_bar")->SetAngularVel({0, 0, 1});
        }

        update_num++;
      }

    /// \brief a pointer to the model this plugin was loaded by
    public: physics::ModelPtr model;
    /// \brief object for callback connection
    public: event::ConnectionPtr updateConnection;
    /// \brief number of updates received
    public: int update_num = 0;
  };

  GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
}
