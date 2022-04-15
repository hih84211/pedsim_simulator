/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include<pedsim_msgs/TrackedPersons.h>
#include<pedsim_msgs/AgentStates.h>


namespace gazebo
{
    class ActorPosesPlugin : public WorldPlugin{
        public:
            ActorPosesPlugin() : WorldPlugin(){}

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
            this->world_ = _world;
            if (!ros::isInitialized()){
                ROS_ERROR("ROS not initialized");
                return;
            }
            rosNode.reset(new ros::NodeHandle("gazebo_client"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
                    "/pedsim_simulator/simulated_agents", 1,
                    boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1),
                    ros::VoidPtr(),&rosQueue);
            rosSub = rosNode->subscribe(so);
            tbPose = rosNode->advertise<pedsim_msgs::AgentState>("/pedsim_gazebo/turtlebot3_pose", 1);
            rosQueueThread =std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));
            physics::ModelPtr tmp_model;
            if(this->world_->ModelByName("turtlebot3_burger") != NULL)
                tmp_model = this->world_->ModelByName("turtlebot3_burger");
            else if(this->world_->ModelByName("turtlebot3_waffle") != NULL)
                tmp_model = this->world_->ModelByName("turtlebot3_waffle");
            if(tmp_model != NULL)
            {
                pedsim_msgs::AgentState first_msg = pedsim_msgs::AgentState();
                first_msg.pose.position.x = tmp_model->WorldPose().Pos().X();
                first_msg.pose.position.y = tmp_model->WorldPose().Pos().Y();
                first_msg.pose.position.z = tmp_model->WorldPose().Pos().Z();

                first_msg.pose.orientation.w = tmp_model->WorldPose().Rot().W();
                first_msg.pose.orientation.x = tmp_model->WorldPose().Rot().X();
                first_msg.pose.orientation.y = tmp_model->WorldPose().Rot().Y();
                first_msg.pose.orientation.z = tmp_model->WorldPose().Rot().Z();

                first_msg.twist.linear.x = tmp_model->WorldLinearVel().X();
                first_msg.twist.linear.y = tmp_model->WorldLinearVel().Y();
                first_msg.twist.linear.z = tmp_model->WorldLinearVel().Z();

                first_msg.twist.angular.x = tmp_model->WorldAngularVel().X();
                first_msg.twist.angular.y = tmp_model->WorldAngularVel().Y();
                first_msg.twist.angular.z = tmp_model->WorldAngularVel().Z();
                first_msg.type = 2;
                tbPose.publish(first_msg);
            }

            // in case you need to change/modify model on update
            // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate, this));
        }


        public:
            // call back function when receive rosmsg
            void OnRosMsg( const pedsim_msgs::AgentStatesConstPtr msg) {
//              ROS_INFO ("OnRosMsg ... ");
                std::string model_name;
#if GAZEBO_MAJOR_VERSION < 9
                for(unsigned int mdl = 0; mdl < world_->GetModelCount(); mdl++) {
#else
                for(unsigned int mdl = 0; mdl < world_->ModelCount(); mdl++) {
#endif
                    physics::ModelPtr tmp_model;
#if GAZEBO_MAJOR_VERSION < 9
                    tmp_model = world_->GetModel(mdl);
#else
                    tmp_model = world_->ModelByIndex(mdl);
#endif
                    std::string frame_id;
                    frame_id = tmp_model->GetName();
                    for (uint actor =0; actor< msg->agent_states.size() ; actor++) {
                        if(frame_id == std::to_string( msg->agent_states[actor].id)  ){
//                            ROS_INFO_STREAM("actor_id: "<< std::to_string( msg->tracks[actor].track_id) );
                            ignition::math::Pose3d gzb_pose;
                            gzb_pose.Pos().Set( msg->agent_states[actor].pose.position.x,
                                                msg->agent_states[actor].pose.position.y,
                                                msg->agent_states[actor].pose.position.z + MODEL_OFFSET);
                            gzb_pose.Rot().Set(msg->agent_states[actor].pose.orientation.w,
                                               msg->agent_states[actor].pose.orientation.x,
                                               msg->agent_states[actor].pose.orientation.y,
                                               msg->agent_states[actor].pose.orientation.z);

                            try{
                                tmp_model->SetWorldPose(gzb_pose);
                            }
                            catch(gazebo::common::Exception gz_ex){
                                ROS_ERROR("Error setting pose %s - %s", frame_id.c_str(), gz_ex.GetErrorStr().c_str());
                            }

                        }

                        if(frame_id == "turtlebot3_burger" || frame_id == "turtlebot3_waffle")
                        {
                            // ToDo: Move the publisher to an independent .cpp file (or make it run concurrently)
                            pedsim_msgs::AgentState state_msg = pedsim_msgs::AgentState();
                            state_msg.pose.position.x = tmp_model->WorldPose().Pos().X();
                            state_msg.pose.position.y = tmp_model->WorldPose().Pos().Y();
                            state_msg.pose.position.z = tmp_model->WorldPose().Pos().Z();

                            state_msg.pose.orientation.w = tmp_model->WorldPose().Rot().W();
                            state_msg.pose.orientation.x = tmp_model->WorldPose().Rot().X();
                            state_msg.pose.orientation.y = tmp_model->WorldPose().Rot().Y();
                            state_msg.pose.orientation.z = tmp_model->WorldPose().Rot().Z();

                            state_msg.twist.linear.x = tmp_model->WorldLinearVel().X();
                            state_msg.twist.linear.y = tmp_model->WorldLinearVel().Y();
                            state_msg.twist.linear.z = tmp_model->WorldLinearVel().Z();

                            state_msg.twist.angular.x = tmp_model->WorldAngularVel().X();
                            state_msg.twist.angular.y = tmp_model->WorldAngularVel().Y();
                            state_msg.twist.angular.z = tmp_model->WorldAngularVel().Z();
                            state_msg.type = 2;

                            tbPose.publish(state_msg);
                        }
                    }
               }
          }

        // ROS helper function that processes messages
        private: void QueueThread() {
            static const double timeout = 0.1;
            while (rosNode->ok()) {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::Publisher tbPose;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        physics::WorldPtr world_;
        event::ConnectionPtr updateConnection_;
        const float MODEL_OFFSET = 0.75;

    };
    GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin);
}


