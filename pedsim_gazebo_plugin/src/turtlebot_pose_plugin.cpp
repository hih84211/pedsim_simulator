//
// Created by peter on 4/10/22.
//
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <string>


namespace gazebo
{
    class TurtlebotPosePlugin:public ModelPlugin
    {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->model = _parent;

            this->updateConnection=event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
        }

    public: void OnUpdate()
        {
            ignition::math::Pose3d pose;
            pose = this->model->WorldPose();
            double x,y,z;
            x = pose.Pos().X(); // x coordinate
            y = pose.Pos().Y(); // y coordinate
            z = pose.Pos().Z(); // z coordinate
            std::cout<<("Position update: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")") << std::endl;
        }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(TurtlebotPosePlugin);
}
