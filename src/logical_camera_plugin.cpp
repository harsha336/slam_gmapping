#include "logical_camera_plugin/logical_camera_plugin.hh"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <sstream>
#include <string>
#include <algorithm>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LogicalCameraPlugin);

LogicalCameraPlugin::LogicalCameraPlugin()
{
  //this->wheelRadius = 0.110;
  //this->wheelSeperation = 0.250; // Distance between opposite wheels (Meters)

}

LogicalCameraPlugin::~LogicalCameraPlugin()
{
  this->rosnode->shutdown();
}

void LogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->robotNamespace = "logical_camera";
  if(_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  this->world = _parent->GetWorld();
  this->name = _parent->GetName();

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
       << "unable to load plugin. Load the Gazebo system plugin "
       << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }


  this->modelFramePrefix = this->name + "_";
  if (_sdf->HasElement("model_frame_prefix"))
  {
    this->modelFramePrefix = _sdf->GetElement("model_frame_prefix")->Get<std::string>();
  }
  gzdbg << "Using model frame prefix of: " << this->modelFramePrefix << std::endl;

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->findLogicalCamera();
  if ( !this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }


  for (int i=0;i<4;i++){
    this->joint[i] = this->model->GetJoints()[i];
    gzdbg << "Joints found are: " <<this->joint[i]->GetScopedName() << "\n";
  }



  std::string imageTopic_ros = this->name;
  if (_sdf->HasElement("image_topic_ros")) 
  {
    imageTopic_ros = _sdf->Get<std::string>("image_topic_ros");
  }

  this->wheelSeperation = this->joint[RIGHT_FRONT]->GetAnchor(0).Distance(this->joint[LEFT_FRONT]->GetAnchor(0)); 

  physics::EntityPtr wheelLink = boost::dynamic_pointer_cast<physics::Entity>( this->joint[RIGHT_FRONT]->GetChild() );

  if (wheelLink)
  {
    math::Box bb = wheelLink->GetBoundingBox();
    this->wheelRadius = bb.GetSize().GetMax() * 0.5;
  }

  if (this->wheelSeperation <= 0)
    {
      gzerr << "Unable to find the wheel separation distance." << std::endl
      << "  This could mean that the right_front link and the left_front "
      << "link are overlapping." << std::endl;
    }
  if (this->wheelRadius <= 0)
  {
    gzerr << "Unable to find the wheel radius." << std::endl
    << "  This could mean that the sdf is missing a wheel link on "
    << "the right_front joint." << std::endl;
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
  	&LogicalCameraPlugin::onImage, this);
  gzdbg << "Subscribing to gazebo topic: "<< this->sensor->Topic() << "\n";

  this->rosQueueThread = std::thread(std::bind(&LogicalCameraPlugin::queueThread, this));

  //imagePub = this->rosnode->advertise<gazebo::msgs::LogicalCameraImage>(imageTopic_ros, 1, true);
  gzdbg << "Publishing to ROS topic: " << imagePub.getTopic() << "\n";

  transformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

}


void LogicalCameraPlugin::queueThread()
{
  static const double timeout = 0.01;
  while (this->rosnode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

void LogicalCameraPlugin::findLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i =0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));

      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
	break;
      }
    }

    if (this->sensor)
    {
      this->cameraLink = link;
      break;

    }
  }
}

void LogicalCameraPlugin::onImage(ConstLogicalCameraImagePtr &_msg)
{

  gazebo::msgs::LogicalCameraImage imageMsg;
  math::Vector3 modelPosition;
  math::Quaternion modelOrientation;
  math::Pose modelPose;

  for (int i = 0;i < _msg->model_size();++i)
  {
    modelPosition = math::Vector3(msgs::ConvertIgn(_msg->model(i).pose().position()));
    modelOrientation = math::Quaternion(msgs::ConvertIgn(_msg->model(i).pose().orientation()));
    modelPose = math::Pose(modelPosition, modelOrientation);
    
    std::string modelName = _msg->model(i).name();
    std::string modelFrameId = this->modelFramePrefix + modelName + "harsha";
    if ( modelName != "ground_plane" && modelName != "se1f1" ) { 
      //gzdbg <<"Model detected: "<< modelName << "with pose" << modelPose << "\n";
      //std::cout <<"Model detected: "<< modelName << "with pose" << modelPose << std::endl;
      this->publishTF(modelPose, this->name,modelFrameId);
    }
  }
  //this->imagePub.publish(imageMsg);
}

void LogicalCameraPlugin::publishTF(
  const math::Pose &pose, const std::string &parentFrame, const std::string &frame)
{
    ros::Time currentTime = ros::Time::now();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform transform(qt, vt);
    transformBroadcaster->sendTransform(tf::StampedTransform(transform, currentTime, parentFrame, frame));
}
