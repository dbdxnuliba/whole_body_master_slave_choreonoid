#include "PrimitiveMotionLevelController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* PrimitiveMotionLevelController_spec[] = {
  "implementation_id", "PrimitiveMotionLevelController",
  "type_name",         "PrimitiveMotionLevelController",
  "description",       "wholebodymasterslave component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  "conf.default.debugLevel", "0",
  ""
};

PrimitiveMotionLevelController::PrimitiveMotionLevelController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  m_debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t PrimitiveMotionLevelController::onInitialize(){
  bindParameter("debugLevel", this->m_debugLevel_, "0");

  addInPort("qRefin", this->ports_.m_qRefIn_);// from sh
  addInPort("basePosRefin", this->ports_.m_basePosRefIn_);
  addInPort("baseRpyRefin", this->ports_.m_baseRpyRefIn_);
  addInPort("primitiveCommandRefin", this->ports_.m_primitiveCommandRefIn_);
  addOutPort("qComOut", this->ports_.m_qComOut_);
  addOutPort("basePosComOut", this->ports_.m_basePosComOut_);
  addOutPort("baseRpyComOut", this->ports_.m_baseRpyComOut_);
  addOutPort("baseTformComOut", this->ports_.m_baseTformComOut_);
  addOutPort("primitiveCommandComOut", this->ports_.m_primitiveCommandComOut_);
  this->ports_.m_PrimitiveMotionLevelControllerServicePort_.registerProvider("service0", "PrimitiveMotionLevelControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_PrimitiveMotionLevelControllerServicePort_);

  this->loop_ = 0;

  cnoid::BodyLoader bodyLoader;
  RTC::Properties& prop = this->getProperties();
  cnoid::BodyPtr robot = bodyLoader.load(prop["model"]);
  if(!robot){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << prop["model"] << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  this->m_robot_ref_ = robot;
  this->m_robot_com_ = robot->clone();

  this->outputRatioInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  return RTC::RTC_OK;
}

namespace PrimitiveMotionLevelControllerImpl {

  void readPorts(const std::string& instance_name, PrimitiveMotionLevelController::Ports& port) {
    if(port.m_qRefIn_.isNew()) port.m_qRefIn_.read();
    if(port.m_basePosRefIn_.isNew()) port.m_basePosRefIn_.read();
    if(port.m_baseRpyRefIn_.isNew()) port.m_baseRpyRefIn_.read();
    if(port.m_primitiveCommandRefIn_.isNew()) port.m_primitiveCommandRefIn_.read();
  }

  // calc reference state (without ee. q, basepos and baserpy only)
  void calcReferenceRobot(const std::string& instance_name, const PrimitiveMotionLevelController::Ports& port, cnoid::BodyPtr& robot) {
    if(port.m_qRef_.data.length() == robot->numJoints()){
      for ( int i = 0; i < robot->numJoints(); i++ ){
        robot->joint(i)->q() = port.m_qRef_.data[i];
      }
    }
    robot->rootLink()->p()[0] = port.m_basePosRef_.data.x;
    robot->rootLink()->p()[1] = port.m_basePosRef_.data.y;
    robot->rootLink()->p()[2] = port.m_basePosRef_.data.z;
    robot->rootLink()->R() = cnoid::rotFromRpy(port.m_baseRpyRef_.data.r, port.m_baseRpyRef_.data.p, port.m_baseRpyRef_.data.y);
    robot->calcForwardKinematics();
  }

  void getPrimitiveCommand(const std::string& instance_name, const PrimitiveMotionLevelController::Ports& port, double dt, std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >::iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); ) {
      bool found = false;
      for(size_t i=0;i<port.m_primitiveCommandRef_.data.length();i++) {
        if(std::string(port.m_primitiveCommandRef_.data[i].name)==it->first) found = true;
      }
      if (!found) it = primitiveCommandMap.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(size_t i=0;i<port.m_primitiveCommandRef_.data.length();i++){
      if(primitiveCommandMap.find(std::string(port.m_primitiveCommandRef_.data[i].name))==primitiveCommandMap.end()){
        primitiveCommandMap[std::string(port.m_primitiveCommandRef_.data[i].name)] = std::make_shared<PrimitiveMotionLevel::PrimitiveCommand>(std::string(port.m_primitiveCommandRef_.data[i].name));
      }
    }
    // 各指令値の反映
    for(size_t i=0;i<port.m_primitiveCommandRef_.data.length();i++){
      const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveState& idl = port.m_primitiveCommandRef_.data[i];
      std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> state = primitiveCommandMap[std::string(idl.name)];
      state->updateFromIdl(idl);
      state->updateTargetForOneStep(dt);
    }
  }

  void processModeTransition(const std::string& instance_name, PrimitiveMotionLevelController::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> >& outputRatioInterpolator, const double dt){
    double tmp;
    switch(mode.now()){
    case PrimitiveMotionLevelController::ControlMode::MODE_SYNC_TO_CONTROL:
      if(mode.pre() == PrimitiveMotionLevelController::ControlMode::MODE_IDLE){
        outputRatioInterpolator->setGoal(1.0, 3.0);
      }
      if (!outputRatioInterpolator->isEmpty() ){
        outputRatioInterpolator->get(tmp, dt);
      }else{
        mode.setNextMode(PrimitiveMotionLevelController::ControlMode::MODE_CONTROL);
      }
      break;
    case PrimitiveMotionLevelController::ControlMode::MODE_SYNC_TO_IDLE:
      if(mode.pre() == PrimitiveMotionLevelController::ControlMode::MODE_CONTROL){
        outputRatioInterpolator->setGoal(0.0, 3.0, true);
      }
      if (outputRatioInterpolator->isEmpty()) {
        outputRatioInterpolator->get(tmp, dt);
      }else{
        mode.setNextMode(PrimitiveMotionLevelController::ControlMode::MODE_IDLE);
      }
      break;
    }
    mode.update();
  }

  void preProcessForControl(const std::string& instance_name, PrimitiveMotionLevel::PositionController& positionController) {
    positionController.reset();
  }

  void passThrough(const std::string& instance_name, const cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_com){
    robot_com->rootLink()->T() = robot_ref->rootLink()->T();
    for(size_t i=0;i<robot_com->numJoints();i++) {
      robot_com->joint(i)->q() = robot_ref->joint(i)->q();
      robot_com->joint(i)->dq() = 0.0;
      robot_com->joint(i)->ddq() = 0.0;
      robot_com->joint(i)->u() = 0.0;
    }

    robot_com->calcForwardKinematics();
  }

  void calcOutputPorts(const std::string& instance_name, PrimitiveMotionLevelController::Ports& port, double output_ratio, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com, std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap) {
    // qCom
    if (port.m_qRef_.data.length() == robot_com->numJoints()){
      port.m_qCom_.data.length(robot_com->numJoints());
      for (int i = 0; i < robot_com->numJoints(); i++ ){
        port.m_qCom_.data[i] = output_ratio * robot_com->joint(i)->q() + (1 - output_ratio) * robot_ref->joint(i)->q();
      }
      port.m_qCom_.tm = port.m_qRef_.tm;
      port.m_qComOut_.write();
    }
    // basePos
    cnoid::Vector3 outputBasePos = output_ratio * robot_com->rootLink()->p() + (1 - output_ratio) * robot_ref->rootLink()->p();
    port.m_basePosCom_.data.x = outputBasePos[0];
    port.m_basePosCom_.data.y = outputBasePos[1];
    port.m_basePosCom_.data.z = outputBasePos[2];
    port.m_basePosCom_.tm = port.m_qRef_.tm;
    port.m_basePosComOut_.write();
    // baseRpy
    cnoid::Matrix3 outputBaseR = cnoid::Matrix3(cnoid::Quaterniond(robot_ref->rootLink()->R()).slerp(output_ratio, cnoid::Quaterniond(robot_com->rootLink()->R())));
    cnoid::Vector3 outputBaseRpy = cnoid::rpyFromRot(outputBaseR);
    port.m_baseRpyCom_.data.r = outputBaseRpy[0];
    port.m_baseRpyCom_.data.p = outputBaseRpy[1];
    port.m_baseRpyCom_.data.y = outputBaseRpy[2];
    port.m_baseRpyCom_.tm = port.m_qRef_.tm;
    port.m_baseRpyComOut_.write();
    // m_baseTform
    port.m_baseTformCom_.data.length(12);
    for(int i=0;i<3;i++) port.m_baseTformCom_.data[i] = outputBasePos[i];
    for(int i=0;i<3;i++) {
      for(int j=0;j<3;j++) {
        port.m_baseTformCom_.data[3+i+j*3] = outputBaseR(i,j);// row major
      }
    }
    port.m_baseTformCom_.tm = port.m_qRef_.tm;
    port.m_baseTformComOut_.write();
    // primitiveCommandCom
    port.m_primitiveCommandCom_ = port.m_primitiveCommandRef_;
    port.m_primitiveCommandCom_.tm = port.m_qRef_.tm;
    for(int i=0;i<port.m_primitiveCommandCom_.data.length();i++){
      std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> primitiveCommand = primitiveCommandMap[std::string(port.m_primitiveCommandCom_.data[i].name)];
      cnoid::Position pose = robot_com->link(primitiveCommand->parentLinkName())->T() * primitiveCommand->localPose();
      port.m_primitiveCommandCom_.data[i].pose.position.x = pose.translation()[0];
      port.m_primitiveCommandCom_.data[i].pose.position.y = pose.translation()[1];
      port.m_primitiveCommandCom_.data[i].pose.position.z = pose.translation()[2];
      cnoid::Vector3 rpy = cnoid::rpyFromRot(pose.linear());
      port.m_primitiveCommandCom_.data[i].pose.orientation.r = rpy[0];
      port.m_primitiveCommandCom_.data[i].pose.orientation.p = rpy[1];
      port.m_primitiveCommandCom_.data[i].pose.orientation.y = rpy[2];
    }
    port.m_primitiveCommandComOut_.write();
  }

}

RTC::ReturnCode_t PrimitiveMotionLevelController::onExecute(RTC::UniqueId ec_id){

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  PrimitiveMotionLevelControllerImpl::readPorts(instance_name, this->ports_);

  // calc reference state from inport (without ee. q, basepos and baserpy only)
  PrimitiveMotionLevelControllerImpl::calcReferenceRobot(instance_name, this->ports_, this->m_robot_ref_);

  // get primitive motion level command
  PrimitiveMotionLevelControllerImpl::getPrimitiveCommand(instance_name, this->ports_, dt, this->primitiveCommandMap_);

  // mode遷移を実行
  PrimitiveMotionLevelControllerImpl::processModeTransition(instance_name, this->mode_, this->outputRatioInterpolator_, dt);

  if(this->mode_.isRunning()) {
    if(this->mode_.isInitialize()){
      PrimitiveMotionLevelControllerImpl::preProcessForControl(instance_name, this->positionController_);
    }

    this->positionController_.control(this->primitiveCommandMap_, this->m_robot_ref_, this->m_robot_com_, dt);

  } else {
    // robot_refがそのままrobot_comになる
    PrimitiveMotionLevelControllerImpl::passThrough(instance_name, this->m_robot_ref_, this->m_robot_com_);
  }

  // write outport
  double output_ratio; this->outputRatioInterpolator_->get(output_ratio);
  PrimitiveMotionLevelControllerImpl::calcOutputPorts(instance_name, this->ports_, output_ratio, this->m_robot_ref_, this->m_robot_com_, this->primitiveCommandMap_);

  this->loop_++;
  return RTC::RTC_OK;
}


bool PrimitiveMotionLevelController::startControl(){
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool PrimitiveMotionLevelController::stopControl(){
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool PrimitiveMotionLevelController::setParams(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  return true;
}


bool PrimitiveMotionLevelController::getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  return true;
}

RTC::ReturnCode_t PrimitiveMotionLevelController::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveMotionLevelController::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveMotionLevelController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void PrimitiveMotionLevelControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveMotionLevelController_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveMotionLevelController>, RTC::Delete<PrimitiveMotionLevelController>);
    }
};
