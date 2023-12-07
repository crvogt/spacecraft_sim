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
#include "sc_set_initial_velocity.hh"
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
        if (update_num <= 200)
        {
          // Link velocity instantaneously without applying forces
          model->GetLink("white_link_0")->SetLinearVel({0, 1, 0});
          model->GetLink("white_link_1")->SetLinearVel({0, 1, 0});
          model->GetLink("white_link_1")->SetAngularVel({1, 0, 0});
          model->GetLink("white_link_2")->SetAngularVel({1, 0, 0});
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
