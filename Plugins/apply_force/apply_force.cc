#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <fstream>

namespace gazebo {
  class ApplyForce : public ModelPlugin {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
      this->link = _parent->GetLinks()[0];
      this->sdf = _sdf;

      std::cout << "nome do modelo: " << _parent->GetName() << std::endl;
      std::cout << "quantidade de links: " << _parent->GetLinks().size() << std::endl;
      std::cout << "nome do link: " << this->link->GetName();

      // listen to events
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ApplyForce::OnUpdate, this));

      this->pause = event::Events::ConnectPause(
          boost::bind(&ApplyForce::OnPause, this));
    }

    // Called by the world update start event
    void OnUpdate() {
      // ajusta força
      double force = this->sdf->Get<double>("force");
      this->link->SetForce(math::Vector3(force, 0, 0));

      // imprime velocidade e aceleraçao
      math::Pose pos = this->link->GetWorldPose();
      math::Vector3 vel = this->link->GetWorldLinearVel();
      math::Vector3 acc = this->link->GetWorldLinearAccel();
      std::cout << "Aceleraçao: " << acc << " / Velocidade: " << vel << "/ Posiçao:" << pos << std::endl;

    }

    void OnPause() {
      std::cout << std::endl << "--- Simulador pausado ---" << std::endl;
    }

    private:
    // elements
    sdf::ElementPtr sdf;
    physics::LinkPtr link;

    // events
    event::ConnectionPtr updateConnection;
    event::ConnectionPtr pause;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ApplyForce)
}
