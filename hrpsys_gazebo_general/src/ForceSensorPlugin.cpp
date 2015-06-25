#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "ForceSensorPlugin.h"

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(ForceSensorPlugin);

  ForceSensorPlugin::ForceSensorPlugin() : force_sensor_average_window_size(5),
					   force_sensor_average_cnt(0)
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
    this->controller_name = this->robot_name + "/" + this->controller_name;
    ROS_INFO("[ForceSensorPlugin] intialize with param: %s", this->controller_name.c_str());

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
	  // setup force sensor publishers
	  boost::shared_ptr<std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> > > forceValQueue(new std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> >);
	  // forceValQueue->resize(this->force_sensor_average_window_size);
	  for ( int i=0; i<this->force_sensor_average_window_size; i++ ){
	    boost::shared_ptr<geometry_msgs::WrenchStamped> fbuf(new geometry_msgs::WrenchStamped);
	    forceValQueue->push_back(fbuf);
	  }
	  this->forceValQueueMap[sensor_name] = forceValQueue;
	  //
	  this->pubForceValQueueMap[sensor_name] = this->pmq.addPub<geometry_msgs::WrenchStamped>();
	  this->pubForceValMap[sensor_name] = this->rosNode->advertise<geometry_msgs::WrenchStamped>(this->robot_name + "/" + sensor_name, 100, true);
	  ROS_INFO("[ForceSensorPlugin] advertise %s", this->pubForceValMap.find(sensor_name)->second.getTopic().c_str());
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
    common::Time curTime = this->world->GetSimTime();
    if (curTime > this->lastControllerUpdateTime) {
      // enqueue force sensor values
      for (unsigned int i = 0; i < this->forceSensorNames.size(); i++) {
	forceSensorMap::iterator it = this->forceSensors.find(this->forceSensorNames[i]);
	boost::shared_ptr<std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> > > forceValQueue = this->forceValQueueMap.find(this->forceSensorNames[i])->second;
	boost::shared_ptr<geometry_msgs::WrenchStamped> forceVal = forceValQueue->at(this->force_sensor_average_cnt);
	geometry_msgs::WrenchStamped forceValMsg;
	if(it != this->forceSensors.end()) {
	  physics::JointPtr jt = it->second.joint;
	  if (!!jt) {
	    physics::JointWrench wrench = jt->GetForceTorque(0u);
	    forceValMsg.header.frame_id = it->second.frame_id;
	    if (!!it->second.pose) {
	      // convert force
	      math::Vector3 force_trans = it->second.pose->rot * wrench.body2Force;
	      math::Vector3 torque_trans = it->second.pose->rot * wrench.body2Torque;
	      // rotate force
	      forceVal->wrench.force.x = force_trans.x;
	      forceVal->wrench.force.y = force_trans.y;
	      forceVal->wrench.force.z = force_trans.z;
	      // rotate torque + additional torque
	      torque_trans += it->second.pose->pos.Cross(force_trans);
	      forceVal->wrench.torque.x = torque_trans.x;
	      forceVal->wrench.torque.y = torque_trans.y;
	      forceVal->wrench.torque.z = torque_trans.z;
	    } else {
	      forceVal->wrench.force.x = wrench.body2Force.x;
	      forceVal->wrench.force.y = wrench.body2Force.y;
	      forceVal->wrench.force.z = wrench.body2Force.z;
	      forceVal->wrench.torque.x = wrench.body2Torque.x;
	      forceVal->wrench.torque.y = wrench.body2Torque.y;
	      forceVal->wrench.torque.z = wrench.body2Torque.z;
	    }
	  } else {
	    ROS_WARN("[ForceSensorPlugin] joint not found for %s", this->forceSensorNames[i].c_str());
	  }
	}
	// std::cout << "[ForceSensorPlugin] set " << this->force_sensor_average_cnt << "-th value (";
	// std::cout << forceVal->wrench.force.x << ","
	// 	  << forceVal->wrench.force.y << ","
	// 	  << forceVal->wrench.force.z << ","
	// 	  << forceVal->wrench.torque.x << ","
	// 	  << forceVal->wrench.torque.y << ","
	// 	  << forceVal->wrench.torque.z << ")" << std::endl;
	// calc average force sensors
	forceValMsg.wrench.force.x = 0;
	forceValMsg.wrench.force.y = 0;
	forceValMsg.wrench.force.z = 0;
	forceValMsg.wrench.torque.x = 0;
	forceValMsg.wrench.torque.y = 0;
	forceValMsg.wrench.torque.z = 0;
	for ( int j=0; j<forceValQueue->size() ; j++ ){
	  boost::shared_ptr<geometry_msgs::WrenchStamped> forceValBuf = forceValQueue->at(j);
	  forceValMsg.wrench.force.x += forceValBuf->wrench.force.x;
	  forceValMsg.wrench.force.y += forceValBuf->wrench.force.y;
	  forceValMsg.wrench.force.z += forceValBuf->wrench.force.z;
	  forceValMsg.wrench.torque.x += forceValBuf->wrench.torque.x;
	  forceValMsg.wrench.torque.y += forceValBuf->wrench.torque.y;
	  forceValMsg.wrench.torque.z += forceValBuf->wrench.torque.z;
	  //
	  // std::cout << "[ForceSensorPlugin] get " << j << "-th value (";
	  // std::cout << forceValBuf->wrench.force.x << ","
	  // 	    << forceValBuf->wrench.force.y << ","
	  // 	    << forceValBuf->wrench.force.z << ","
	  // 	    << forceValBuf->wrench.torque.x << ","
	  // 	    << forceValBuf->wrench.torque.y << ","
	  // 	    << forceValBuf->wrench.torque.z << ")" << std::endl;
	}
	if ( forceValQueue->size() > 0 ){
	  forceValMsg.wrench.force.x *= 1.0/forceValQueue->size();
	  forceValMsg.wrench.force.y *= 1.0/forceValQueue->size();
	  forceValMsg.wrench.force.z *= 1.0/forceValQueue->size();
	  forceValMsg.wrench.torque.x *= 1.0/forceValQueue->size();
	  forceValMsg.wrench.torque.y *= 1.0/forceValQueue->size();
	  forceValMsg.wrench.torque.z *= 1.0/forceValQueue->size();
	} else {
	  ROS_WARN("[ForceSensorPlugin] invalid force val queue size 0");
	}
	// publish force sensor values
	forceValMsg.header.stamp = ros::Time(curTime.sec, curTime.nsec);
	PubQueue<geometry_msgs::WrenchStamped>::Ptr pubForceValQueue = this->pubForceValQueueMap.find(this->forceSensorNames[i])->second;
	ros::Publisher pubForceVal = this->pubForceValMap.find(this->forceSensorNames[i])->second;
	pubForceValQueue->push(forceValMsg, pubForceVal);
      }
      this->force_sensor_average_cnt = (this->force_sensor_average_cnt+1) % this->force_sensor_average_window_size;
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
