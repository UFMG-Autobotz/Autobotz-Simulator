#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {
  class ModelPush : public ModelPlugin {
    public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      // Store the pointer to the model
      this->model = _parent;
      std::cerr << "nome do modelo: " << this->model->GetName() << std::endl;

      this->link = this->model->GetLinks()[0];
      std::cerr << "quantidade de links: " << this->model->GetLinks().size() << std::endl;
      std::cerr << "nome do link: " << this->link->GetName();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/) {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(.03, 0, 0));

      //this->link->SetForce(math::Vector3(.0955, 0, 0));
      math::Vector3 vel = this->link->GetRelativeLinearVel();
      std::cerr << "Velocidade: " << vel.x << std::endl;
    }

    private:
    physics::ModelPtr model; // Pointer to the model
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
