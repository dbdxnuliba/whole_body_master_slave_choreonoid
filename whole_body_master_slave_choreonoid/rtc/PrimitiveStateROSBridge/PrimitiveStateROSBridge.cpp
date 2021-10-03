#include "PrimitiveStateROSBridge.h"
#include <tf2/utils.h>

#include <cnoid/BodyLoader>

PrimitiveStateROSBridge::PrimitiveStateROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_primitiveCommandROSOut_("primitiveStateOut", m_primitiveCommandROS_),
  m_primitiveCommandRTMIn_("primitiveStateIn", m_primitiveCommandRTM_)
{
}

RTC::ReturnCode_t PrimitiveStateROSBridge::onInitialize(){
  addOutPort("primitiveStateOut", m_primitiveCommandROSOut_);
  addInPort("primitiveStateIn", m_primitiveCommandRTMIn_);

  cnoid::BodyLoader bodyLoader;

  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  this->robot_vrml_ = bodyLoader.load(fileName);
  if(!this->robot_vrml_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->robot_urdf_ = std::make_shared<urdf::Model>();
  this->robot_urdf_->initParam("robot_description");

  ros::NodeHandle pnh("~");
  sub_ = pnh.subscribe("input", 1, &PrimitiveStateROSBridge::topicCallback, this);
  pub_ = pnh.advertise<whole_body_master_slave_choreonoid::PrimitiveStateArray>("output", 1);

  return RTC::RTC_OK;
}

std::string URDFToVRMLLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& URDFLinkName){
  std::shared_ptr<const urdf::Link> link = robot_urdf->getLink(URDFLinkName);
  if(link){
    if(link->parent_joint){
      return link->parent_joint->name;
    }else if (link == robot_urdf->getRoot()){
      return robot_vrml->rootLink()->name();
    }
  }
  std::cerr << "\x1b[31m" << "failed to find link [" << URDFLinkName << "]" << "\x1b[39m" << std::endl;
  return URDFLinkName;
};

std::string VRMLToURDFLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& VRMLLinkName){
  std::shared_ptr<const urdf::Joint> joint = robot_urdf->getJoint(VRMLLinkName);
  if(joint){
    return joint->child_link_name;
  }else if (robot_vrml->rootLink()->name() == VRMLLinkName){
    return robot_urdf->getRoot()->name;
  }
  std::cerr << "\x1b[31m" << "failed to find link [" << VRMLLinkName << "]" << "\x1b[39m" << std::endl;
  return VRMLLinkName;
};

RTC::ReturnCode_t PrimitiveStateROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_primitiveCommandRTMIn_.isNew()){
    this->m_primitiveCommandRTMIn_.read();

    whole_body_master_slave_choreonoid::PrimitiveStateArray msg;
    for(int i=0;i<m_primitiveCommandRTM_.data.length();i++){
      whole_body_master_slave_choreonoid::PrimitiveState state;
      state.name = std::string(m_primitiveCommandRTM_.data[i].name);
      state.parent_link_name = VRMLToURDFLinkName(this->robot_vrml_, this->robot_urdf_, std::string(m_primitiveCommandRTM_.data[i].parentLinkName));
      state.local_pose.position.x = m_primitiveCommandRTM_.data[i].localPose.position.x;
      state.local_pose.position.y = m_primitiveCommandRTM_.data[i].localPose.position.y;
      state.local_pose.position.z = m_primitiveCommandRTM_.data[i].localPose.position.z;
      tf2::Quaternion quat;
      quat.setRPY(m_primitiveCommandRTM_.data[i].localPose.orientation.r,
                  m_primitiveCommandRTM_.data[i].localPose.orientation.p,
                  m_primitiveCommandRTM_.data[i].localPose.orientation.y);
      state.local_pose.orientation.x = quat.x();
      state.local_pose.orientation.y = quat.y();
      state.local_pose.orientation.z = quat.z();
      state.local_pose.orientation.w = quat.w();
      state.time = m_primitiveCommandRTM_.data[i].time;
      state.pose.position.x = m_primitiveCommandRTM_.data[i].pose.position.x;
      state.pose.position.y = m_primitiveCommandRTM_.data[i].pose.position.y;
      state.pose.position.z = m_primitiveCommandRTM_.data[i].pose.position.z;
      quat.setRPY(m_primitiveCommandRTM_.data[i].pose.orientation.r,
                  m_primitiveCommandRTM_.data[i].pose.orientation.p,
                  m_primitiveCommandRTM_.data[i].pose.orientation.y);
      state.pose.orientation.x = quat.x();
      state.pose.orientation.y = quat.y();
      state.pose.orientation.z = quat.z();
      state.pose.orientation.w = quat.w();
      state.wrench.resize(6);
      for(int j=0;j<6;j++) state.wrench[j] = m_primitiveCommandRTM_.data[i].wrench[j];
      state.M.resize(6);
      for(int j=0;j<6;j++) state.M[j] = m_primitiveCommandRTM_.data[i].M[j];
      state.D.resize(6);
      for(int j=0;j<6;j++) state.D[j] = m_primitiveCommandRTM_.data[i].D[j];
      state.K.resize(6);
      for(int j=0;j<6;j++) state.K[j] = m_primitiveCommandRTM_.data[i].K[j];
      state.act_wrench.resize(6);
      for(int j=0;j<6;j++) state.act_wrench[j] = m_primitiveCommandRTM_.data[i].actWrench[j];
      state.wrench_gain.resize(6);
      for(int j=0;j<6;j++) state.wrench_gain[j] = m_primitiveCommandRTM_.data[i].wrenchGain[j];
      state.support_com = m_primitiveCommandRTM_.data[i].supportCOM;

      msg.primitive_state.push_back(state);
    }
    this->pub_.publish(msg);
  }

  return RTC::RTC_OK;
}

void PrimitiveStateROSBridge::topicCallback(const whole_body_master_slave_choreonoid::PrimitiveStateArray::ConstPtr& msg) {
  m_primitiveCommandROS_.tm.sec = msg->header.stamp.sec;
  m_primitiveCommandROS_.tm.nsec = msg->header.stamp.nsec;
  m_primitiveCommandROS_.data.length(msg->primitive_state.size());
  for(int i=0;i<msg->primitive_state.size();i++){
    m_primitiveCommandROS_.data[i].name = msg->primitive_state[i].name.c_str();
    m_primitiveCommandROS_.data[i].parentLinkName = URDFToVRMLLinkName(this->robot_vrml_, this->robot_urdf_, msg->primitive_state[i].parent_link_name).c_str();
    m_primitiveCommandROS_.data[i].localPose.position.x = msg->primitive_state[i].local_pose.position.x;
    m_primitiveCommandROS_.data[i].localPose.position.y = msg->primitive_state[i].local_pose.position.y;
    m_primitiveCommandROS_.data[i].localPose.position.z = msg->primitive_state[i].local_pose.position.z;
    tf2::Quaternion quat(msg->primitive_state[i].local_pose.orientation.x,msg->primitive_state[i].local_pose.orientation.y,msg->primitive_state[i].local_pose.orientation.z,msg->primitive_state[i].local_pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(m_primitiveCommandROS_.data[i].localPose.orientation.r,m_primitiveCommandROS_.data[i].localPose.orientation.p,m_primitiveCommandROS_.data[i].localPose.orientation.y);
    m_primitiveCommandROS_.data[i].time = msg->primitive_state[i].time;
    m_primitiveCommandROS_.data[i].pose.position.x = msg->primitive_state[i].pose.position.x;
    m_primitiveCommandROS_.data[i].pose.position.y = msg->primitive_state[i].pose.position.y;
    m_primitiveCommandROS_.data[i].pose.position.z = msg->primitive_state[i].pose.position.z;
    quat = tf2::Quaternion(msg->primitive_state[i].pose.orientation.x,msg->primitive_state[i].pose.orientation.y,msg->primitive_state[i].pose.orientation.z,msg->primitive_state[i].pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(m_primitiveCommandROS_.data[i].pose.orientation.r,m_primitiveCommandROS_.data[i].pose.orientation.p,m_primitiveCommandROS_.data[i].pose.orientation.y);
    if(msg->primitive_state[i].wrench.size() == 6){
      for(int j=0;j<msg->primitive_state[i].wrench.size();j++) m_primitiveCommandROS_.data[i].wrench[j] = msg->primitive_state[i].wrench[j];
    }
    if(msg->primitive_state[i].M.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].M.size();j++) m_primitiveCommandROS_.data[i].M[j] = msg->primitive_state[i].M[j];
    }
    if(msg->primitive_state[i].D.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].D.size();j++) m_primitiveCommandROS_.data[i].D[j] = msg->primitive_state[i].D[j];
    }
    if(msg->primitive_state[i].K.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].K.size();j++) m_primitiveCommandROS_.data[i].K[j] = msg->primitive_state[i].K[j];
    }
    if(msg->primitive_state[i].act_wrench.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].act_wrench.size();j++) m_primitiveCommandROS_.data[i].actWrench[j] = msg->primitive_state[i].act_wrench[j];
    }
    if(msg->primitive_state[i].wrench_gain.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].wrench_gain.size();j++) m_primitiveCommandROS_.data[i].wrenchGain[j] = msg->primitive_state[i].wrench_gain[j];
    }
    m_primitiveCommandROS_.data[i].supportCOM = msg->primitive_state[i].support_com;
  }

  m_primitiveCommandROSOut_.write();
}

static const char* PrimitiveStateROSBridge_spec[] = {
  "implementation_id", "PrimitiveStateROSBridge",
  "type_name",         "PrimitiveStateROSBridge",
  "description",       "PrimitiveStateROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void PrimitiveStateROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveStateROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveStateROSBridge>, RTC::Delete<PrimitiveStateROSBridge>);
    }
};
