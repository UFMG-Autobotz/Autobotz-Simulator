#include "getJoints.hh"

const unsigned int REVOLUTE = 576;
const unsigned int PRISMATIC = 1088;

Data::Data(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->model = _model;
  this->sdf = sdf;
}

void Data::ReadVariables() {

  // read global velocity pid gains
	this->vel_pid_gains = math::Vector3(1, 0, 0); // default value
	if (this->sdf->HasElement("vel_pid")) {
		this->vel_pid_gains = this->sdf->Get<math::Vector3>("vel_pid");
	} else if (this->sdf->HasElement("pid")) {
		this->vel_pid_gains = this->sdf->Get<math::Vector3>("pid");
	}

	// read global position pid gains
	this->pos_pid_gains = math::Vector3(1, 0, 0); // default value
	if (this->sdf->HasElement("pos_pid")) {
		this->pos_pid_gains = this->sdf->Get<math::Vector3>("pos_pid");
	} else if (this->sdf->HasElement("pid")) {
		this->pos_pid_gains = this->sdf->Get<math::Vector3>("pid");
	}

	// read joints
	sdf::ElementPtr parameter;
	joint_param joint;
	int idx = 1;
	std::ostringstream tag;
	tag << "joint" << idx;
	while (this->sdf->HasElement(tag.str())) {
		parameter = this->sdf->GetElementImpl(tag.str());
		joint.valid = true;

		if (parameter->HasAttribute("name")) {
			std::string jointName = parameter->GetAttribute("name")->GetAsString();
			joint.joint = this->model->GetJoint(jointName);

			std::ostringstream validateJoint;
			validateJoint << joint.joint;

			if (validateJoint.str() == "0") {
				gzerr << jointName << " isn't a valid joint name, " << tag.str() << " will be ignored!" << std::endl;
				joint.valid = false;
			} else {
				int jointType = joint.joint->GetType();
				if (jointType != REVOLUTE && jointType != PRISMATIC) {
					gzerr << jointName << " has an invalid type, " << tag.str() << " will be ignored!" << std::endl;
					joint.valid = false;
				} else {
					joint.name = joint.joint->GetScopedName();
				}
			}

		} else {
			gzerr << tag.str() << " doesn't have a name and will be ignored!" << std::endl;
			joint.valid = false;
		}

		if (parameter->HasElement("vel_pid")) {
			joint.vel_pid_gains = parameter->Get<math::Vector3>("vel_pid");
		} else if (this->sdf->HasElement("pid")) {
			joint.vel_pid_gains = parameter->Get<math::Vector3>("pid");
		} else {
			joint.vel_pid_gains = this->vel_pid_gains;
		}

		if (parameter->HasElement("pos_pid")) {
			joint.pos_pid_gains = parameter->Get<math::Vector3>("pos_pid");
		} else if (this->sdf->HasElement("pid")) {
			joint.pos_pid_gains = parameter->Get<math::Vector3>("pid");
		} else {
			joint.pos_pid_gains = this->pos_pid_gains;
		}

		joint.velocity = false;
		if (parameter->HasElement("velocity")) {
			joint.velocity = parameter->Get<bool>("velocity");
		}

		joint.position = false;
		if (parameter->HasElement("position")) {
			joint.position = parameter->Get<bool>("position");
		}

		if (joint.velocity && parameter->HasAttribute("vel_topic")) {
			joint.veltopic = "/" + parameter->GetAttribute("vel_topic")->GetAsString();
		} else {
			joint.veltopic = "/" + this->model->GetName() + "/joint_vel_" + joint.joint->GetName();
		}

		if (joint.position && parameter->HasAttribute("pos_topic")) {
			joint.postopic = "/" + parameter->GetAttribute("pos_topic")->GetAsString();
		} else {
			joint.postopic = "/" + this->model->GetName() + "/joint_pos_" + joint.joint->GetName();
		}

		this->joints.push_back(joint);

		idx++;
		tag.str("");
		tag << "joint" << idx;

  }

	if (idx == 1) {
		this->all = true;
		this->n_joints = this->model->GetJointCount();
		std::cout << "------------" << std::endl;
		gzmsg << this->n_joints << " Joints found" << std::endl;
		std::cout << "------------" << std::endl;

		if (this->n_joints == 0) {
			gzerr << "Invalid joint count, VT_sim plugin not loaded" << std::endl;
			return;
		}


		this->joints_vector = this->model->GetJoints();

	} else {
		this->n_joints = this->joints.size();
	}
}
