#include <gazebo_mimic_plugin/mimic_plugin.h>

using namespace gazebo;

MimicPlugin::MimicPlugin():  ModelPlugin()
{
  kill_sim = false;

  joint_.reset();
  mimic_joint_.reset();
}

MimicPlugin::~MimicPlugin()
{
  // https://github.com/crigroup/robotiq/issues/4#issuecomment-505989048
#if GAZEBO_MAJOR_VERSION >= 9
  this->updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif

  kill_sim = true;
}

void MimicPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();

  joint_name_ = "joint";
  if (_sdf->HasElement("joint"))
    joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

  mimic_joint_name_ = "mimicJoint";
  if (_sdf->HasElement("mimicJoint"))
    mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

  multiplier_ = 1.0;
  if (_sdf->HasElement("multiplier"))
    multiplier_ = _sdf->GetElement("multiplier")->Get<double>();

  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicPlugin::UpdateChild, this));
  
  ROS_INFO_STREAM("MimicPlugin model name: "<< modelName);

  joint_ = model_->GetJoint(joint_name_);
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);
}

void MimicPlugin::UpdateChild()
{
  // https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins/blob/509a32ea6accc58d03cebd9d670ae44635adc924/src/mimic_joint_plugin.cpp#L160-L167
#if GAZEBO_MAJOR_VERSION >= 8
  double angle = joint_->Position(0)*multiplier_;
#else
  double angle = joint_->GetAngle(0).Radian()*multiplier_;
#endif

  // https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins/blob/509a32ea6accc58d03cebd9d670ae44635adc924/src/mimic_joint_plugin.cpp#L178-L188
#if GAZEBO_MAJOR_VERSION >= 9
   mimic_joint_->SetPosition(0, angle, true);
#elif GAZEBO_MAJOR_VERSION > 2
   mimic_joint_->SetPosition(0, angle);
#else
   mimic_joint_->SetAngle(0, math::Angle(angle));
#endif
}

GZ_REGISTER_MODEL_PLUGIN(MimicPlugin);
