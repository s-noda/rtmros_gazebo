#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "hrpsys_gazebo_msgs/JointCommand.h"
#include "hrpsys_gazebo_msgs/RobotState.h"

#include "hrpsys_gazebo_msgs/SyncCommand.h"

#include "PubQueue.h"

namespace gazebo
{
  typedef boost::shared_ptr< math::Pose > PosePtr;

  class ForceSensorPlugin : public ModelPlugin
  {
  public:
    ForceSensorPlugin();
    virtual ~ForceSensorPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  private:
    void UpdateStates();
    void RosQueueThread();
    void DeferredLoad();

    void GetForceTorqueSensorState(const common::Time &_curTime);

    struct force_sensor_info {
      physics::JointPtr joint;
      std::string frame_id;
      PosePtr pose;
    };

    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;

    physics::WorldPtr world;
    physics::ModelPtr model;
    sdf::ElementPtr sdf;

    event::ConnectionPtr updateConnection;
    boost::thread deferredLoadThread;

    common::Time lastControllerUpdateTime;

    typedef std::map< std::string, struct force_sensor_info > forceSensorMap;
    std::vector<std::string> forceSensorNames;
    forceSensorMap forceSensors;

    //
    PubMultiQueue pmq;
    //
    std::string robot_name;
    std::string controller_name;

    static inline int xmlrpc_value_as_int(XmlRpc::XmlRpcValue &v) {
      if((v.getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
         (v.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double d = v;
          return (int)d;
        } else {
          int i = v;
          return i;
        }
      }
      // not number
      return 0;
    }
    static inline double xmlrpc_value_as_double(XmlRpc::XmlRpcValue &v) {
      if((v.getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
         (v.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double d = v;
          return d;
        } else {
          int i = v;
          return (double)i;
        }
      }
      // not number
      return 0.0;
    }

    int force_sensor_average_window_size;
  };
}
