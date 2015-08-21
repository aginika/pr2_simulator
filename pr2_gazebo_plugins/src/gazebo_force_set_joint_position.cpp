#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  class ForceSetJointPosition : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->robot_namespace_     = "";

      if (!ros::isInitialized())
	{
	  int argc = 0;
	  char** argv = NULL;
	  ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	}


      // Store the pointer to the model
      this->model = _parent;
      rosnode_        = new ros::NodeHandle(this->robot_namespace_);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      sub_  = rosnode_->subscribe("force_set_state", 10, &ForceSetJointPosition::SetJointState, this);
    }

    ~ForceSetJointPosition()
    {
      this->rosnode_->shutdown();

      delete rosnode_;
    }

  void SetJointState(const sensor_msgs::JointState& joint_state_msg){
    for (unsigned int i = 0; i < joint_state_msg.position.size(); ++i)
      {
	this->model->GetJoint(joint_state_msg.name[i])->SetAngle(0, joint_state_msg.position[i]);
	this->model->GetJoint(joint_state_msg.name[i])->SetVelocity(0, 0);
      }
  }

    // Pointer to the model
  private: physics::ModelPtr model;

    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
    ros::Subscriber sub_;
    ros::NodeHandle* rosnode_;
    std::string robot_namespace_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ForceSetJointPosition)
}
