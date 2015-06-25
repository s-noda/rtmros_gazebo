#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "ForceSensorPlugin.h"

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(ForceSensorPlugin);

  ForceSensorPlugin::ForceSensorPlugin() : force_sensor_average_window_size(5)
  {
  }

  ForceSensorPlugin::~ForceSensorPlugin() {
  }

  void ForceSensorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->controller_name = "hrpsys_gazebo_configuration";
    if (_sdf->HasElement("controller")) {
      this->controller_name = _sdf->Get<std::string>("controller");
    }

    this->robot_name = _parent->GetScopedName();
    if (_sdf->HasElement("robotname")) {
      this->robot_name = _sdf->Get<std::string>("robotname");
      ROS_WARN("USE ROBOT NAME from URDF: %s, scoped name(%s)",
	       this->robot_name.c_str(),
	       _parent->GetScopedName().c_str());
    }

    // ros node
    this->rosNode = new ros::NodeHandle("");

    this->model = _parent;
    this->world = this->model->GetWorld();

    // save sdf
    this->sdf = _sdf;

    // initialize update time
    this->lastControllerUpdateTime = this->world->GetSimTime();

    XmlRpc::XmlRpcValue param_val;
    this->rosNode->getParam(this->controller_name, param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      XmlRpc::XmlRpcValue fsensors = param_val["force_torque_sensors"];
      XmlRpc::XmlRpcValue fsensors_config = param_val["force_torque_sensors_config"];

      // Force sensor setting
      if (fsensors.getType() == XmlRpc::XmlRpcValue::TypeArray &&
	  fsensors_config.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
	for(int s = 0; s < fsensors.size(); s++) {
	  this->forceSensorNames.push_back(fsensors[s]);
	}
	for(XmlRpc::XmlRpcValue::iterator f = fsensors_config.begin();
	    f != fsensors_config.end(); f++) {
	  std::string sensor_name = f->first;
	  if (f->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
	    std::string jn = f->second["joint_name"];
	    std::string fi = f->second["frame_id"];
	    ROS_INFO("force: %s, %s %s", sensor_name.c_str(), jn.c_str(), fi.c_str());

	    struct force_sensor_info fsi;
	    fsi.joint = this->model->GetJoint(jn);
	    if(!fsi.joint) {
	      gzerr << "force torque joint (" << jn << ") not found\n";
	    } else {
	      fsi.frame_id = fi;
	      this->forceSensors[sensor_name] = fsi;
	      XmlRpc::XmlRpcValue trs = f->second["translation"];
	      XmlRpc::XmlRpcValue rot = f->second["rotation"];
	      fsi.pose.reset();
	      if ((trs.getType() == XmlRpc::XmlRpcValue::TypeArray) ||
		  (rot.getType() == XmlRpc::XmlRpcValue::TypeArray)) {
		math::Vector3 vtr;
		math::Quaternion qt;
		if (trs.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		  vtr.x = xmlrpc_value_as_double(trs[0]);
		  vtr.y = xmlrpc_value_as_double(trs[1]);
		  vtr.z = xmlrpc_value_as_double(trs[2]);
		}
		if (rot.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		  qt.w = xmlrpc_value_as_double(rot[0]);
		  qt.x = xmlrpc_value_as_double(rot[1]);
		  qt.y = xmlrpc_value_as_double(rot[2]);
		  qt.z = xmlrpc_value_as_double(rot[3]);
		}
		fsi.pose = PosePtr(new math::Pose (vtr, qt));
	      }
	    }
	  } else {
	    ROS_ERROR("Force-Torque sensor: %s has invalid configuration", sensor_name.c_str());
	  }
	}
      } else {
	ROS_WARN("Force-Torque sensor: no setting exists");
      }
    }

    // ros callback queue for processing subscription
    this->deferredLoadThread = boost::thread(boost::bind(&ForceSensorPlugin::DeferredLoad, this));
  }

  void ForceSensorPlugin::DeferredLoad() {
    // publish multi queue
    this->pmq.startServiceThread();

    this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&ForceSensorPlugin::UpdateStates, this));
  }

  void ForceSensorPlugin::UpdateStates() {
    //ROS_DEBUG("update");
    common::Time curTime = this->world->GetSimTime();
    if (curTime > this->lastControllerUpdateTime) {
    }
    this->lastControllerUpdateTime = curTime;
  }

  void ForceSensorPlugin::RosQueueThread() {
    static const double timeout = 0.01;

    while (this->rosNode->ok()) {
      ros::spinOnce();
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }
}
