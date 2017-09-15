#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"

#include <stdio.h>

namespace gazebo {
  class rosConnection : public ModelPlugin {
  public:
  rosConnection() {
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_talker", ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_talker"));

    this->pubFor = this->rosNode.get()->advertise<geometry_msgs::Vector3>("forca", 1000);
    this->pubAcc = this->rosNode.get()->advertise<geometry_msgs::Vector3>("aceleracao", 1000);
    this->pubVel = this->rosNode.get()->advertise<geometry_msgs::Vector3>("velocidade", 1000);
    this->pubPos = this->rosNode.get()->advertise<geometry_msgs::Pose>("posicao", 1000);
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->sdf = _sdf;
    this->link = _parent->GetLinks()[0];

    this->dforce = this->sdf->Get<double>("dforce");
    this->intialX = this->link->GetWorldPose().pos.x;

  	// Listen to the update event. This event is broadcast every
  	// simulation iteration.
  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
  	  boost::bind(&rosConnection::OnUpdate, this));
  }

  // Called by the world update start event
  void OnUpdate() {
    if (this->isClose(this->sdf->Get<double>("dlimit"))) {
      this->force += this->dforce;
    	this->link->AddForce(math::Vector3(this->force, 0, 0));
    } else {
      this->link->SetForce(math::Vector3(0, 0, 0));
    }

    this->publishForce();
    this->publishAcceleration();
    this->publishVelocity();
    this->publishPosition();
  }

  bool isClose(double limitDist) {
    this->displacementX = this->link->GetWorldPose().pos.x - this->intialX;
    return this->displacementX < limitDist;
}

  void publishForce() {
    math::Vector3 force = this->link->GetWorldForce();
    geometry_msgs::Vector3 msgFor;
    msgFor.x = force.x; msgFor.y = force.y; msgFor.z = force.z;
    this->pubFor.publish(msgFor);
  }

  void publishAcceleration() {
    math::Vector3 acc = this->link->GetWorldLinearAccel();
    geometry_msgs::Vector3 msgAcc;
    msgAcc.x = acc.x; msgAcc.y = acc.y; msgAcc.z = acc.z;
    this->pubAcc.publish(msgAcc);
  }

  void publishVelocity() {
    math::Vector3 vel = this->link->GetWorldLinearVel();
    geometry_msgs::Vector3 msgVel;
    msgVel.x = vel.x; msgVel.y = vel.y; msgVel.z = vel.z;
    this->pubVel.publish(msgVel);
  }

  void publishPosition() {
    math::Pose pos = this->link->GetWorldPose();
    geometry_msgs::Pose msgPos;
    msgPos.position.x = pos.pos.x; msgPos.position.y = pos.pos.y; msgPos.position.z = pos.pos.z;
    msgPos.orientation.x = pos.rot.x; msgPos.orientation.y = pos.rot.y; msgPos.orientation.z = pos.rot.z; msgPos.orientation.w = pos.rot.w;
    this->pubPos.publish(msgPos);
  }

  private:
  // SDF elements
  sdf::ElementPtr sdf;
  physics::LinkPtr link;

  // events
  event::ConnectionPtr updateConnection;

  // ROS
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Publisher pubFor, pubAcc, pubVel, pubPos;

  // force
  double dforce, force;

  // displacement
  double intialX, displacementX;
  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(rosConnection)
}
