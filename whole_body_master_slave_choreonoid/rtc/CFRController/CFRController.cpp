#include "CFRController.h"

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* CFRController_spec[] = {
  "implementation_id", "CFRController",
  "type_name",         "CFRController",
  "description",       "wholebodymasterslave component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

CFRController::CFRController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t CFRController::onInitialize(){

  addInPort("primitiveCommandRefIn", this->ports_.m_primitiveCommandRefIn_);
  addOutPort("primitiveCommandComOut", this->ports_.m_primitiveCommandComOut_);
  this->ports_.m_CFRControllerServicePort_.registerProvider("service0", "CFRControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_CFRControllerServicePort_);

  this->loop_ = 0;

  return RTC::RTC_OK;
}

void CFRController::readPorts(const std::string& instance_name, CFRController::Ports& port) {
  if(port.m_primitiveCommandRefIn_.isNew()) port.m_primitiveCommandRefIn_.read();
}

void CFRController::getPrimitiveCommand(const std::string& instance_name, const CFRController::Ports& port, double dt, std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> >& primitiveCommandMap) {
  // 消滅したEndEffectorを削除
  for(std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> >::iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); ) {
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
      primitiveCommandMap[std::string(port.m_primitiveCommandRef_.data[i].name)] = std::make_shared<CFR::PrimitiveCommand>(std::string(port.m_primitiveCommandRef_.data[i].name));
    }
  }
  // 各指令値の反映
  for(size_t i=0;i<port.m_primitiveCommandRef_.data.length();i++){
    const primitive_motion_level_msgs::PrimitiveStateIdl& idl = port.m_primitiveCommandRef_.data[i];
    std::shared_ptr<CFR::PrimitiveCommand> state = primitiveCommandMap[std::string(idl.name)];
    state->updateFromIdl(idl);
    state->updateTargetForOneStep(dt);
  }
}

void CFRController::processModeTransition(const std::string& instance_name, CFRController::ControlMode& mode){
  switch(mode.now()){
  case CFRController::ControlMode::MODE_SYNC_TO_CONTROL:
    mode.setNextMode(CFRController::ControlMode::MODE_CONTROL);
    break;
  case CFRController::ControlMode::MODE_SYNC_TO_IDLE:
    mode.setNextMode(CFRController::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void CFRController::preProcessForControl(const std::string& instance_name) {
}

void CFRController::calcOutputPorts(const std::string& instance_name, CFRController::Ports& port, std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> >& primitiveCommandMap) {
  // primitiveCommandCom
  port.m_primitiveCommandCom_ = port.m_primitiveCommandRef_;
  port.m_primitiveCommandComOut_.write();
}

RTC::ReturnCode_t CFRController::onExecute(RTC::UniqueId ec_id){

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  CFRController::readPorts(instance_name, this->ports_);

  // get primitive motion level command
  CFRController::getPrimitiveCommand(instance_name, this->ports_, dt, this->primitiveCommandMap_);

  // mode遷移を実行
  CFRController::processModeTransition(instance_name, this->mode_);

  if(this->mode_.isRunning()) {
    if(this->mode_.isInitialize()){
      CFRController::preProcessForControl(instance_name);
    }

  }

  // write outport
  CFRController::calcOutputPorts(instance_name, this->ports_, this->primitiveCommandMap_);

  this->loop_++;
  return RTC::RTC_OK;
}


bool CFRController::startControl(){
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool CFRController::stopControl(){
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool CFRController::setParams(const whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;
  return true;
}


bool CFRController::getParams(whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  return true;
}

RTC::ReturnCode_t CFRController::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t CFRController::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t CFRController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void CFRControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(CFRController_spec);
        manager->registerFactory(profile, RTC::Create<CFRController>, RTC::Delete<CFRController>);
    }
};
