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
  pub_ = pnh.advertise<primitive_motion_level_msgs::PrimitiveStateArray>("output", 1);

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
  std::cerr << "\x1b[31m" << "[URDFToVRMLLinkName] failed to find link [" << URDFLinkName << "]" << "\x1b[39m" << std::endl;
  return URDFLinkName;
};

std::string VRMLToURDFLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& VRMLLinkName){
  std::shared_ptr<const urdf::Joint> joint = robot_urdf->getJoint(VRMLLinkName);
  if(joint){
    return joint->child_link_name;
  }else if (robot_vrml->rootLink()->name() == VRMLLinkName){
    return robot_urdf->getRoot()->name;
  }
  std::cerr << "\x1b[31m" << "[VRMLToURDFLinkName] failed to find link [" << VRMLLinkName << "]" << "\x1b[39m" << std::endl;
  return VRMLLinkName;
};

RTC::ReturnCode_t PrimitiveStateROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_primitiveCommandRTMIn_.isNew()){
    this->m_primitiveCommandRTMIn_.read();

    primitive_motion_level_msgs::PrimitiveStateArray msg;
    msg.header.stamp = ros::Time::now();
    for(int i=0;i<m_primitiveCommandRTM_.data.length();i++){
      primitive_motion_level_msgs::PrimitiveState state;
      state.name = std::string(m_primitiveCommandRTM_.data[i].name);
      if(std::string(m_primitiveCommandRTM_.data[i].parentLinkName) == "com") state.parent_link_name = "com";
      else state.parent_link_name = VRMLToURDFLinkName(this->robot_vrml_, this->robot_urdf_, std::string(m_primitiveCommandRTM_.data[i].parentLinkName));
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
      state.pose_follow_gain.resize(6);
      for(int j=0;j<6;j++) state.pose_follow_gain[j] = m_primitiveCommandRTM_.data[i].poseFollowGain[j];
      state.wrench_follow_gain.resize(6);
      for(int j=0;j<6;j++) state.wrench_follow_gain[j] = m_primitiveCommandRTM_.data[i].wrenchFollowGain[j];
      state.is_poseC_global = m_primitiveCommandRTM_.data[i].isPoseCGlobal;
      state.poseC.resize(m_primitiveCommandRTM_.data[i].poseC.length()*6);
      for(int j=0;j<m_primitiveCommandRTM_.data[i].poseC.length();j++){
        for(int k=0;k<6;k++) state.poseC[j*6+k] = m_primitiveCommandRTM_.data[i].poseC[j][k];
      }
      state.poseld.resize(m_primitiveCommandRTM_.data[i].poseld.length());
      for(int j=0;j<m_primitiveCommandRTM_.data[i].poseld.length();j++){
        state.poseld[j] = m_primitiveCommandRTM_.data[i].poseld[j];
      }
      state.poseud.resize(m_primitiveCommandRTM_.data[i].poseud.length());
      for(int j=0;j<m_primitiveCommandRTM_.data[i].poseud.length();j++){
        state.poseud[j] = m_primitiveCommandRTM_.data[i].poseud[j];
      }
      state.is_wrenchC_global = m_primitiveCommandRTM_.data[i].isWrenchCGlobal;
      state.wrenchC.resize(m_primitiveCommandRTM_.data[i].wrenchC.length()*6);
      for(int j=0;j<m_primitiveCommandRTM_.data[i].wrenchC.length();j++){
        for(int k=0;k<6;k++) state.wrenchC[j*6+k] = m_primitiveCommandRTM_.data[i].wrenchC[j][k];
      }
      state.wrenchld.resize(m_primitiveCommandRTM_.data[i].wrenchld.length());
      for(int j=0;j<m_primitiveCommandRTM_.data[i].wrenchld.length();j++){
        state.wrenchld[j] = m_primitiveCommandRTM_.data[i].wrenchld[j];
      }
      state.wrenchud.resize(m_primitiveCommandRTM_.data[i].wrenchud.length());
      for(int j=0;j<m_primitiveCommandRTM_.data[i].wrenchud.length();j++){
        state.wrenchud[j] = m_primitiveCommandRTM_.data[i].wrenchud[j];
      }
      state.M.resize(6);
      for(int j=0;j<6;j++) state.M[j] = m_primitiveCommandRTM_.data[i].M[j];
      state.D.resize(6);
      for(int j=0;j<6;j++) state.D[j] = m_primitiveCommandRTM_.data[i].D[j];
      state.K.resize(6);
      for(int j=0;j<6;j++) state.K[j] = m_primitiveCommandRTM_.data[i].K[j];
      state.act_wrench.resize(6);
      for(int j=0;j<6;j++) state.act_wrench[j] = m_primitiveCommandRTM_.data[i].actWrench[j];
      state.support_com = m_primitiveCommandRTM_.data[i].supportCOM;

      msg.primitive_state.push_back(state);
    }
    this->pub_.publish(msg);
  }

  return RTC::RTC_OK;
}

void PrimitiveStateROSBridge::topicCallback(const primitive_motion_level_msgs::PrimitiveStateArray::ConstPtr& msg) {
  coil::TimeValue coiltm(coil::gettimeofday());
  m_primitiveCommandROS_.tm.sec  = coiltm.sec();
  m_primitiveCommandROS_.tm.nsec = coiltm.usec() * 1000;
  m_primitiveCommandROS_.data.length(msg->primitive_state.size());
  for(int i=0;i<msg->primitive_state.size();i++){
    m_primitiveCommandROS_.data[i].name = msg->primitive_state[i].name.c_str();
    if(msg->primitive_state[i].parent_link_name == "com") m_primitiveCommandROS_.data[i].parentLinkName = "com";
    else m_primitiveCommandROS_.data[i].parentLinkName = URDFToVRMLLinkName(this->robot_vrml_, this->robot_urdf_, msg->primitive_state[i].parent_link_name).c_str();
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
    }else{
      for(int j=0;j<msg->primitive_state[i].wrench.size();j++) m_primitiveCommandROS_.data[i].wrench[j] = 0.0;
    }
    if(msg->primitive_state[i].pose_follow_gain.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].pose_follow_gain.size();j++) m_primitiveCommandROS_.data[i].poseFollowGain[j] = msg->primitive_state[i].pose_follow_gain[j];
    }else{
      for(int j=0;j<msg->primitive_state[i].pose_follow_gain.size();j++) m_primitiveCommandROS_.data[i].poseFollowGain[j] = 0.0;
    }
    if(msg->primitive_state[i].wrench_follow_gain.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].wrench_follow_gain.size();j++) m_primitiveCommandROS_.data[i].wrenchFollowGain[j] = msg->primitive_state[i].wrench_follow_gain[j];
    }else{
      for(int j=0;j<msg->primitive_state[i].wrench_follow_gain.size();j++) m_primitiveCommandROS_.data[i].wrenchFollowGain[j] = 0.0;
    }
    m_primitiveCommandROS_.data[i].isPoseCGlobal = msg->primitive_state[i].is_poseC_global;
    if(msg->primitive_state[i].poseC.size() % 6 == 0){
      m_primitiveCommandROS_.data[i].poseC.length(msg->primitive_state[i].poseC.size() / 6);
      for(int j=0;j<m_primitiveCommandROS_.data[i].poseC.length(); j++) {
        for(int k=0;k<6;k++){
          m_primitiveCommandROS_.data[i].poseC[j][k] = msg->primitive_state[i].poseC[j*6+k];
        }
      }
    }
    m_primitiveCommandROS_.data[i].poseld.length(msg->primitive_state[i].poseld.size());
    for(int j=0;j<m_primitiveCommandROS_.data[i].poseld.length(); j++) {
        m_primitiveCommandROS_.data[i].poseld[j] = msg->primitive_state[i].poseld[j];
    }
    m_primitiveCommandROS_.data[i].poseud.length(msg->primitive_state[i].poseud.size());
    for(int j=0;j<m_primitiveCommandROS_.data[i].poseud.length(); j++) {
        m_primitiveCommandROS_.data[i].poseud[j] = msg->primitive_state[i].poseud[j];
    }
    m_primitiveCommandROS_.data[i].isWrenchCGlobal = msg->primitive_state[i].is_wrenchC_global;
    if(msg->primitive_state[i].wrenchC.size() % 6 == 0){
      m_primitiveCommandROS_.data[i].wrenchC.length(msg->primitive_state[i].wrenchC.size() / 6);
      for(int j=0;j<m_primitiveCommandROS_.data[i].wrenchC.length(); j++) {
        for(int k=0;k<6;k++){
          m_primitiveCommandROS_.data[i].wrenchC[j][k] = msg->primitive_state[i].wrenchC[j*6+k];
        }
      }
    }
    m_primitiveCommandROS_.data[i].wrenchld.length(msg->primitive_state[i].wrenchld.size());
    for(int j=0;j<m_primitiveCommandROS_.data[i].wrenchld.length(); j++) {
        m_primitiveCommandROS_.data[i].wrenchld[j] = msg->primitive_state[i].wrenchld[j];
    }
    m_primitiveCommandROS_.data[i].wrenchud.length(msg->primitive_state[i].wrenchud.size());
    for(int j=0;j<m_primitiveCommandROS_.data[i].wrenchud.length(); j++) {
        m_primitiveCommandROS_.data[i].wrenchud[j] = msg->primitive_state[i].wrenchud[j];
    }
    if(msg->primitive_state[i].M.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].M.size();j++) m_primitiveCommandROS_.data[i].M[j] = msg->primitive_state[i].M[j];
    }else{
      for(int j=0;j<msg->primitive_state[i].M.size();j++) m_primitiveCommandROS_.data[i].M[j] = 0.0;
    }
    if(msg->primitive_state[i].D.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].D.size();j++) m_primitiveCommandROS_.data[i].D[j] = msg->primitive_state[i].D[j];
    }else{
      for(int j=0;j<msg->primitive_state[i].D.size();j++) m_primitiveCommandROS_.data[i].D[j] = 0.0;
    }
    if(msg->primitive_state[i].K.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].K.size();j++) m_primitiveCommandROS_.data[i].K[j] = msg->primitive_state[i].K[j];
    }else{
      for(int j=0;j<msg->primitive_state[i].K.size();j++) m_primitiveCommandROS_.data[i].K[j] = 0.0;
    }
    if(msg->primitive_state[i].act_wrench.size() == 6) {
      for(int j=0;j<msg->primitive_state[i].act_wrench.size();j++) m_primitiveCommandROS_.data[i].actWrench[j] = msg->primitive_state[i].act_wrench[j];
    }else{
      for(int j=0;j<msg->primitive_state[i].act_wrench.size();j++) m_primitiveCommandROS_.data[i].actWrench[j] = 0.0;
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
