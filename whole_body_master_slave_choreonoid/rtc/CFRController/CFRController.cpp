#include "CFRController.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>

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
  addOutPort("verticesOut", this->ports_.m_verticesOut_);
  this->ports_.m_CFRControllerServicePort_.registerProvider("service0", "CFRControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_CFRControllerServicePort_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->loop_ = 0;

  this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);

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

void CFRController::calcOutputPorts(const std::string& instance_name,
                                    CFRController::Ports& port,
                                    std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> >& primitiveCommandMap,
                                    double dt,
                                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& M,// world frame
                                    const Eigen::VectorXd& l,// world frame
                                    const Eigen::VectorXd& u,// world frame
                                    const std::vector<Eigen::Vector2d>& vertices// world frame
                                    ){
  // primitiveCommand
  port.m_primitiveCommandCom_ = port.m_primitiveCommandRef_;
  for(int i=0;i<port.m_primitiveCommandCom_.data.length();i++){
    const cnoid::Position& targetPose = primitiveCommandMap[std::string(port.m_primitiveCommandCom_.data[i].name)]->targetPose();
    port.m_primitiveCommandCom_.data[i].time = dt;
    port.m_primitiveCommandCom_.data[i].pose.position.x = targetPose.translation()[0];
    port.m_primitiveCommandCom_.data[i].pose.position.y = targetPose.translation()[1];
    port.m_primitiveCommandCom_.data[i].pose.position.z = targetPose.translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPose.linear());
    port.m_primitiveCommandCom_.data[i].pose.orientation.r = rpy[0];
    port.m_primitiveCommandCom_.data[i].pose.orientation.p = rpy[1];
    port.m_primitiveCommandCom_.data[i].pose.orientation.y = rpy[2];
    const cnoid::Vector6& targetWrench = primitiveCommandMap[std::string(port.m_primitiveCommandCom_.data[i].name)]->targetWrench();
    for(size_t j=0;j<6;j++) port.m_primitiveCommandCom_.data[i].wrench[j] = targetWrench[j];
  }

  // com Feasible Region
  int comIdx = -1;
  for(int i=0;i<port.m_primitiveCommandCom_.data.length();i++){
    if(std::string(port.m_primitiveCommandCom_.data[i].name) == "com") comIdx = i;
  }
  if(comIdx == -1){
    comIdx = port.m_primitiveCommandCom_.data.length();
    port.m_primitiveCommandCom_.data.length(port.m_primitiveCommandCom_.data.length()+1);
    port.m_primitiveCommandCom_.data[comIdx].name="com";
    port.m_primitiveCommandCom_.data[comIdx].parentLinkName="com";
    port.m_primitiveCommandCom_.data[comIdx].localPose.position.x=0.0;
    port.m_primitiveCommandCom_.data[comIdx].localPose.position.y=0.0;
    port.m_primitiveCommandCom_.data[comIdx].localPose.position.z=0.0;
    port.m_primitiveCommandCom_.data[comIdx].localPose.orientation.r=0.0;
    port.m_primitiveCommandCom_.data[comIdx].localPose.orientation.p=0.0;
    port.m_primitiveCommandCom_.data[comIdx].localPose.orientation.y=0.0;
    port.m_primitiveCommandCom_.data[comIdx].time = dt;
    port.m_primitiveCommandCom_.data[comIdx].pose.position.x=0.0;
    port.m_primitiveCommandCom_.data[comIdx].pose.position.y=0.0;
    port.m_primitiveCommandCom_.data[comIdx].pose.position.z=0.0;
    port.m_primitiveCommandCom_.data[comIdx].pose.orientation.r=0.0;
    port.m_primitiveCommandCom_.data[comIdx].pose.orientation.p=0.0;
    port.m_primitiveCommandCom_.data[comIdx].pose.orientation.y=0.0;
    port.m_primitiveCommandCom_.data[comIdx].supportCOM = false;
    port.m_primitiveCommandCom_.data[comIdx].isWrenchCGlobal = false;
    for(int i=0;i<6;i++) port.m_primitiveCommandCom_.data[comIdx].wrench[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveCommandCom_.data[comIdx].M[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveCommandCom_.data[comIdx].D[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveCommandCom_.data[comIdx].K[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveCommandCom_.data[comIdx].poseFollowGain[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveCommandCom_.data[comIdx].wrenchFollowGain[i] = 0.0;
  }
  port.m_primitiveCommandCom_.data[comIdx].poseC.length(M.rows());
  port.m_primitiveCommandCom_.data[comIdx].poseld.length(l.rows());
  port.m_primitiveCommandCom_.data[comIdx].poseud.length(u.rows());
  cnoid::MatrixXd Mdense = M;
  for(int i=0;i<M.rows();i++){
    for(int j=0;j<3;j++){
      port.m_primitiveCommandCom_.data[comIdx].poseC[i][j] = Mdense(i,j);
      port.m_primitiveCommandCom_.data[comIdx].poseC[i][3+j] = 0.0;
    }
    port.m_primitiveCommandCom_.data[comIdx].poseld[i] = l[i];
    port.m_primitiveCommandCom_.data[comIdx].poseud[i] = u[i];
  }
  port.m_primitiveCommandCom_.data[comIdx].isPoseCGlobal = true;
  port.m_primitiveCommandComOut_.write();

  // vertices
  port.m_vertices_.tm = port.m_primitiveCommandRef_.tm;
  port.m_vertices_.data.length(vertices.size()*2);
  for(int i=0;i<vertices.size();i++){
    for(int j=0;j<2;j++) port.m_vertices_.data[i*2+j] = vertices[i][j];
  }
  port.m_verticesOut_.write();

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

  // 重心world XY座標のFR
  Eigen::SparseMatrix<double,Eigen::RowMajor> M(0,2);
  Eigen::VectorXd l;
  Eigen::VectorXd u;
  std::vector<Eigen::Vector2d> vertices;
  if(this->mode_.isRunning()) {
    if(this->mode_.isInitialize()){
      CFRController::preProcessForControl(instance_name);
    }

    if(cFRCalculator_.computeCFR(this->primitiveCommandMap_, this->robot_, this->debugLevel_)){
      M = this->cFRCalculator_.M();
      l = this->cFRCalculator_.l();
      u = this->cFRCalculator_.u();
      vertices = this->cFRCalculator_.vertices();
    }
  }

  // write outport
  CFRController::calcOutputPorts(instance_name, this->ports_, this->primitiveCommandMap_, dt,M,l,u, vertices);

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
