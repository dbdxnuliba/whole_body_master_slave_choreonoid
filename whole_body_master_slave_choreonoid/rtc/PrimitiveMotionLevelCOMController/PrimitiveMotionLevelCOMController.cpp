#include "PrimitiveMotionLevelCOMController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* PrimitiveMotionLevelCOMController_spec[] = {
  "implementation_id", "PrimitiveMotionLevelCOMController",
  "type_name",         "PrimitiveMotionLevelCOMController",
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

PrimitiveMotionLevelCOMController::PrimitiveMotionLevelCOMController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  m_debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t PrimitiveMotionLevelCOMController::onInitialize(){
  bindParameter("debugLevel", this->m_debugLevel_, "0");

  addInPort("qRef", this->ports_.m_qRefIn_);// from sh
  addInPort("basePosRef", this->ports_.m_basePosRefIn_);
  addInPort("baseRpyRef", this->ports_.m_baseRpyRefIn_);
  addInPort("primitiveCommandRefRef", this->ports_.m_primitiveCommandRefIn_);
  addInPort("qAct", this->ports_.m_qActIn_);
  addInPort("imuAct", this->ports_.m_imuActIn_);
  addOutPort("qCom", this->ports_.m_qComOut_);
  addOutPort("basePosCom", this->ports_.m_basePosComOut_);
  addOutPort("baseRpyCom", this->ports_.m_baseRpyComOut_);
  this->ports_.m_PrimitiveMotionLevelCOMControllerServicePort_.registerProvider("service0", "PrimitiveMotionLevelCOMControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_PrimitiveMotionLevelCOMControllerServicePort_);

  RTC::Properties& prop = getProperties();
  coil::stringTo(this->m_dt_, prop["dt"].c_str());

  this->loop_ = 0;

  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load(prop["model"]);
  if(!robot){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << prop["model"] << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  this->m_robot_ref_ = robot;
  this->m_robot_act_ = robot->clone();
  this->m_robot_com_ = robot->clone();

  this->outputRatioInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  return RTC::RTC_OK;
}

namespace PrimitiveMotionLevelCOMControllerImpl {

  void readPorts(const std::string& instance_name, PrimitiveMotionLevelCOMController::Ports& port) {
    if(port.m_qRefIn_.isNew()) port.m_qRefIn_.read();
    if(port.m_basePosRefIn_.isNew()) port.m_basePosRefIn_.read();
    if(port.m_baseRpyRefIn_.isNew()) port.m_baseRpyRefIn_.read();
    if(port.m_primitiveCommandRefIn_.isNew()) port.m_primitiveCommandRefIn_.read();
    if(port.m_qActIn_.isNew()) port.m_qActIn_.read();
    if(port.m_imuActIn_.isNew()) port.m_imuActIn_.read();
  }

  // calc reference state (without ee. q, basepos and baserpy only)
  void calcReferenceRobot(const std::string& instance_name, const PrimitiveMotionLevelCOMController::Ports& port, cnoid::BodyPtr& robot) {
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

  // calc actial state from inport
  void calcActualRobot(const std::string& instance_name, const PrimitiveMotionLevelCOMController::Ports& port, cnoid::BodyPtr& robot_act) {
    if(port.m_qAct_.data.length() == robot_act->numJoints()){
      for ( int i = 0; i < robot_act->numJoints(); i++ ){
        robot_act->joint(i)->q() = port.m_qAct_.data[i];
      }
      robot_act->calcForwardKinematics();

      cnoid::AccelerationSensorPtr imu = robot_act->findDevice<cnoid::AccelerationSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(port.m_imuAct_.data.r, port.m_imuAct_.data.p, port.m_imuAct_.data.y);
      robot_act->rootLink()->R() = (actR * (imuR.transpose() * robot_act->rootLink()->R())).eval();
      robot_act->calcForwardKinematics();
    }
  }

  void getPrimitiveCommand(const std::string& instance_name, const PrimitiveMotionLevelCOMController::Ports& port, double dt, std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap) {
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
      const whole_body_master_slave_choreonoid::PrimitiveStateIdl& idl = port.m_primitiveCommandRef_.data[i];
      std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> state = primitiveCommandMap[std::string(idl.name)];
      state->updateFromIdl(idl);
      state->updateTargetForOneStep(dt);
    }
  }

  void moveActualRobotToFixedFrame(const std::string& instance_name, const std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap, bool& fixedFrameWarped, cnoid::BodyPtr& robot_act) {
    for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
      if(it->second->supportCOMChanged()) fixedFrameWarped = true;
    }

    std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >::const_iterator fixedFrameIt;
    for(fixedFrameIt = primitiveCommandMap.begin(); fixedFrameIt != primitiveCommandMap.end(); fixedFrameIt++) {
      if(fixedFrameIt->second->supportCOM()) break;
    }
    if(fixedFrameIt == primitiveCommandMap.end()){
      std::cerr << "\x1b[31m[" << instance_name << "] " << "Fixed Frame not exists" << "\x1b[39m" << std::endl;
      return;
    }
    std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> fixedFrame = fixedFrameIt->second;

    if(!robot_act->link(fixedFrame->parentLinkName())) {
      std::cerr << "\x1b[31m[" << instance_name << "] " << "link " << fixedFrame->parentLinkName() << " not found" << "\x1b[39m" << std::endl;
      return;
    }
    cnoid::Position fixedFrameCom = fixedFrame->targetPose();
    cnoid::Position fixedFrameAct = robot_act->link(fixedFrame->parentLinkName())->T() * fixedFrame->localPose();
    cnoid::Position trans = fixedFrameCom * fixedFrameAct.inverse();
    {
      // 鉛直軸は修正しない
      Eigen::Quaterniond rot;
      rot.setFromTwoVectors(trans.linear() * Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
      trans.linear() = (rot * trans.linear()).eval();
    };
    robot_act->rootLink()->T() = trans * robot_act->rootLink()->T();
  }

  void processModeTransition(const std::string& instance_name, PrimitiveMotionLevelCOMController::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> >& outputRatioInterpolator, const double dt){
    double tmp;
    switch(mode.now()){
    case PrimitiveMotionLevelCOMController::ControlMode::MODE_SYNC_TO_CONTROL:
      if(mode.pre() == PrimitiveMotionLevelCOMController::ControlMode::MODE_IDLE){
        outputRatioInterpolator->setGoal(1.0, 3.0);
      }
      if (!outputRatioInterpolator->isEmpty() ){
        outputRatioInterpolator->get(tmp, dt);
      }else{
        mode.setNextMode(PrimitiveMotionLevelCOMController::ControlMode::MODE_CONTROL);
      }
      break;
    case PrimitiveMotionLevelCOMController::ControlMode::MODE_SYNC_TO_IDLE:
      if(mode.pre() == PrimitiveMotionLevelCOMController::ControlMode::MODE_CONTROL){
        outputRatioInterpolator->setGoal(0.0, 3.0, true);
      }
      if (outputRatioInterpolator->isEmpty()) {
        outputRatioInterpolator->get(tmp, dt);
      }else{
        mode.setNextMode(PrimitiveMotionLevelCOMController::ControlMode::MODE_IDLE);
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

  void calcOutputPorts(const std::string& instance_name, PrimitiveMotionLevelCOMController::Ports& port, double output_ratio, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com) {
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
    cnoid::Vector3 ouputBasePos = output_ratio * robot_com->rootLink()->p() + (1 - output_ratio) * robot_ref->rootLink()->p();
    port.m_basePosCom_.data.x = ouputBasePos[0];
    port.m_basePosCom_.data.y = ouputBasePos[1];
    port.m_basePosCom_.data.z = ouputBasePos[2];
    port.m_basePosCom_.tm = port.m_qRef_.tm;
    port.m_basePosComOut_.write();
    // baseRpy
    cnoid::Vector3 outputBaseRpy = cnoid::rpyFromRot(cnoid::Matrix3(cnoid::Quaterniond(robot_ref->rootLink()->R()).slerp(output_ratio, cnoid::Quaterniond(robot_com->rootLink()->R()))));
    port.m_baseRpyCom_.data.r = outputBaseRpy[0];
    port.m_baseRpyCom_.data.p = outputBaseRpy[1];
    port.m_baseRpyCom_.data.y = outputBaseRpy[2];
    port.m_baseRpyCom_.tm = port.m_qRef_.tm;
    port.m_baseRpyComOut_.write();

  }

}

RTC::ReturnCode_t PrimitiveMotionLevelCOMController::onExecute(RTC::UniqueId ec_id){

  std::string instance_name = std::string(this->m_profile.instance_name);

  // read ports
  PrimitiveMotionLevelCOMControllerImpl::readPorts(instance_name, this->ports_);

  // calc reference state from inport (without ee. q, basepos and baserpy only)
  PrimitiveMotionLevelCOMControllerImpl::calcReferenceRobot(instance_name, this->ports_, this->m_robot_ref_);

  // calc actial state from inport
  PrimitiveMotionLevelCOMControllerImpl::calcActualRobot(instance_name, this->ports_, this->m_robot_act_);

  // get primitive motion level command
  PrimitiveMotionLevelCOMControllerImpl::getPrimitiveCommand(instance_name, this->ports_, this->m_dt_, this->primitiveCommandMap_);

  // match frame of actual robot to command.
  bool fixedFrameWarped = false;
  PrimitiveMotionLevelCOMControllerImpl::moveActualRobotToFixedFrame(instance_name, this->primitiveCommandMap_, fixedFrameWarped, this->m_robot_act_);

  // mode遷移を実行
  PrimitiveMotionLevelCOMControllerImpl::processModeTransition(instance_name, this->mode_, this->outputRatioInterpolator_, this->m_dt_);

  if(this->mode_.isRunning()) {
    if(this->mode_.isInitialize()){
      PrimitiveMotionLevelCOMControllerImpl::preProcessForControl(instance_name, this->positionController_);
    }

    this->positionController_.control(this->primitiveCommandMap_, this->m_robot_ref_, this->m_robot_com_, fixedFrameWarped, this->m_dt_);

  } else {
    // robot_refがそのままrobot_comになる
    PrimitiveMotionLevelCOMControllerImpl::passThrough(instance_name, this->m_robot_ref_, this->m_robot_com_);
  }

  // write outport
  double output_ratio; this->outputRatioInterpolator_->get(output_ratio);
  PrimitiveMotionLevelCOMControllerImpl::calcOutputPorts(instance_name, this->ports_, output_ratio, this->m_robot_ref_, this->m_robot_com_);

  this->loop_++;
  return RTC::RTC_OK;
}


bool PrimitiveMotionLevelCOMController::startControl(){
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool PrimitiveMotionLevelCOMController::stopControl(){
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool PrimitiveMotionLevelCOMController::setParams(const whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  return true;
}


bool PrimitiveMotionLevelCOMController::getParams(whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  return true;
}

RTC::ReturnCode_t PrimitiveMotionLevelCOMController::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveMotionLevelCOMController::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveMotionLevelCOMController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void PrimitiveMotionLevelCOMControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveMotionLevelCOMController_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveMotionLevelCOMController>, RTC::Delete<PrimitiveMotionLevelCOMController>);
    }
};
