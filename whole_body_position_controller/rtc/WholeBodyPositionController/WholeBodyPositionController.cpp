#include "WholeBodyPositionController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* WholeBodyPositionController_spec[] = {
  "implementation_id", "WholeBodyPositionController",
  "type_name",         "WholeBodyPositionController",
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

WholeBodyPositionController::WholeBodyPositionController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t WholeBodyPositionController::onInitialize(){

  addInPort("qRefIn", this->ports_.m_qRefIn_);// from sh
  addInPort("basePosRefIn", this->ports_.m_basePosRefIn_);
  addInPort("baseRpyRefIn", this->ports_.m_baseRpyRefIn_);
  addInPort("primitiveCommandRefIn", this->ports_.m_primitiveCommandRefIn_);
  addInPort("collisionComIn", this->ports_.m_collisionComIn_);
  addOutPort("qComOut", this->ports_.m_qComOut_);
  addOutPort("basePosComOut", this->ports_.m_basePosComOut_);
  addOutPort("baseRpyComOut", this->ports_.m_baseRpyComOut_);
  addOutPort("basePoseComOut", this->ports_.m_basePoseComOut_);
  addOutPort("baseTformComOut", this->ports_.m_baseTformComOut_);
  addOutPort("primitiveCommandComOut", this->ports_.m_primitiveCommandComOut_);
  this->ports_.m_WholeBodyPositionControllerServicePort_.registerProvider("service0", "WholeBodyPositionControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_WholeBodyPositionControllerServicePort_);

  this->loop_ = 0;

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  cnoid::BodyPtr robot = bodyLoader.load(fileName);
  if(!robot){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  this->m_robot_ref_ = robot;
  this->m_robot_com_ = robot->clone();

  this->useJoints_.push_back(this->m_robot_com_->rootLink());
  for(int i=0;i<this->m_robot_com_->numJoints();i++) this->useJoints_.push_back(this->m_robot_com_->joint(i));

  this->outputOffsetInterpolators_.rootpInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  this->outputOffsetInterpolators_.rootRInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolatorSO3 >(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  for(int i=0;i<this->m_robot_com_->numJoints();i++) this->outputOffsetInterpolators_.jointInterpolatorMap_[this->m_robot_com_->joint(i)] = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  for(size_t i=0;i<this->m_robot_com_->numJoints();i++){
    // apply margin
    if(this->m_robot_ref_->joint(i)->q_upper() - this->m_robot_ref_->joint(i)->q_lower() > 0.002){
      this->m_robot_com_->joint(i)->setJointRange(this->m_robot_ref_->joint(i)->q_lower()+0.001,this->m_robot_ref_->joint(i)->q_upper()-0.001);
    }
    // 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい
    this->m_robot_com_->joint(i)->setJointVelocityRange(std::max(this->m_robot_ref_->joint(i)->dq_lower(), -1.0),
                                                        std::min(this->m_robot_ref_->joint(i)->dq_upper(), 1.0));
  }
  this->m_robot_com_->rootLink()->setJointVelocityRange(std::max(this->m_robot_ref_->rootLink()->dq_lower(), -1.0),
                                                        std::min(this->m_robot_ref_->rootLink()->dq_upper(), 1.0));

  std::string jointLimitTableStr;
  if(this->getProperties().hasKey("joint_limit_table")) jointLimitTableStr = std::string(this->getProperties()["joint_limit_table"]);
  else jointLimitTableStr = std::string(this->m_pManager->getConfig()["joint_limit_table"]); // 引数 -o で与えたプロパティを捕捉
  std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->m_robot_com_, jointLimitTableStr);
  std::cerr << "[" << this->m_profile.instance_name << "] joint_limit_table: " << jointLimitTableStr<<std::endl;
  for(size_t i=0;i<jointLimitTables.size();i++){
    // apply margin
    for(size_t j=0;j<jointLimitTables[i]->lLimitTable().size();j++){
      if(jointLimitTables[i]->uLimitTable()[j] - jointLimitTables[i]->lLimitTable()[j] > 0.002){
        jointLimitTables[i]->uLimitTable()[j] -= 0.001;
        jointLimitTables[i]->lLimitTable()[j] += 0.001;
      }
    }
    this->jointLimitTablesMap_[jointLimitTables[i]->getSelfJoint()].push_back(jointLimitTables[i]);
  }

  return RTC::RTC_OK;
}

void WholeBodyPositionController::readPorts(const std::string& instance_name, WholeBodyPositionController::Ports& port) {
  if(port.m_qRefIn_.isNew()) port.m_qRefIn_.read();
  if(port.m_basePosRefIn_.isNew()) port.m_basePosRefIn_.read();
  if(port.m_baseRpyRefIn_.isNew()) port.m_baseRpyRefIn_.read();
  if(port.m_primitiveCommandRefIn_.isNew()) port.m_primitiveCommandRefIn_.read();
  if(port.m_collisionComIn_.isNew()) port.m_collisionComIn_.read();
}

  // calc reference state (without ee. q, basepos and baserpy only)
void WholeBodyPositionController::calcReferenceRobot(const std::string& instance_name, const WholeBodyPositionController::Ports& port, cnoid::BodyPtr& robot) {
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
  robot->calcCenterOfMass();
}

void WholeBodyPositionController::getPrimitiveCommand(const std::string& instance_name, const WholeBodyPositionController::Ports& port, double dt, std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap) {
  // 消滅したEndEffectorを削除
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); ) {
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
      primitiveCommandMap[std::string(port.m_primitiveCommandRef_.data[i].name)] = std::make_shared<primitive_motion_level_tools::PrimitiveState>(std::string(port.m_primitiveCommandRef_.data[i].name));
    }
  }
  // 各指令値の反映
  for(size_t i=0;i<port.m_primitiveCommandRef_.data.length();i++){
    const primitive_motion_level_msgs::PrimitiveStateIdl& idl = port.m_primitiveCommandRef_.data[i];
    std::shared_ptr<primitive_motion_level_tools::PrimitiveState> state = primitiveCommandMap[std::string(idl.name)];
    state->updateFromIdl(idl);
    state->updateTargetForOneStep(dt);
  }
}

void WholeBodyPositionController::getCollision(const std::string& instance_name, const WholeBodyPositionController::Ports& port, std::vector<std::shared_ptr<WholeBodyPosition::Collision> >& collisions) {
  collisions.resize(port.m_collisionCom_.data.length());
  // 各collisionの反映
  for(size_t i=0;i<port.m_collisionCom_.data.length();i++){
    const collision_checker_msgs::CollisionIdl& idl = port.m_collisionCom_.data[i];
    if(!collisions[i]) collisions[i] = std::make_shared<WholeBodyPosition::Collision>();
    collisions[i]->updateFromIdl(idl);
  }
}


void WholeBodyPositionController::processModeTransition(const std::string& instance_name, WholeBodyPositionController::ControlMode& mode, const double dt, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com, WholeBodyPositionController::OutputOffsetInterpolators& outputOffsetInterpolators){
  switch(mode.now()){
  case WholeBodyPositionController::ControlMode::MODE_SYNC_TO_CONTROL:
    mode.setNextMode(WholeBodyPositionController::ControlMode::MODE_CONTROL);
    break;
  case WholeBodyPositionController::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == WholeBodyPositionController::ControlMode::MODE_CONTROL){
      outputOffsetInterpolators.rootpInterpolator_->reset(robot_com->rootLink()->p()-robot_ref->rootLink()->p(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
      outputOffsetInterpolators.rootpInterpolator_->setGoal(cnoid::Vector3::Zero(), 3.0);
      outputOffsetInterpolators.rootRInterpolator_->reset(robot_com->rootLink()->R()*robot_ref->rootLink()->R().transpose(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
      outputOffsetInterpolators.rootRInterpolator_->setGoal(cnoid::Matrix3d::Identity(), 3.0);
      for(int i=0;i<robot_com->numJoints();i++) {
        outputOffsetInterpolators.jointInterpolatorMap_[robot_com->joint(i)]->reset(robot_com->joint(i)->q()-robot_ref->joint(i)->q(),0.0,0.0);
        outputOffsetInterpolators.jointInterpolatorMap_[robot_com->joint(i)]->setGoal(0.0,3.0);
      }
    }
    mode.setNextMode(WholeBodyPositionController::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void WholeBodyPositionController::preProcessForControl(const std::string& instance_name, WholeBodyPosition::PositionController& positionController) {
  positionController.reset();
}

void WholeBodyPositionController::passThrough(const std::string& instance_name, const cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_com, WholeBodyPositionController::OutputOffsetInterpolators& outputOffsetInterpolators, double dt, const std::vector<cnoid::LinkPtr>& useJoints){
  if(std::find(useJoints.begin(), useJoints.end(), robot_com->rootLink()) == useJoints.end()){
    cnoid::Vector3 offsetp = cnoid::Vector3::Zero();
    if (!outputOffsetInterpolators.rootpInterpolator_->isEmpty()) outputOffsetInterpolators.rootpInterpolator_->get(offsetp, dt);
    robot_com->rootLink()->p() = robot_ref->rootLink()->p() + offsetp;
    cnoid::Matrix3 offsetR = cnoid::Matrix3d::Identity();
    if (!outputOffsetInterpolators.rootRInterpolator_->isEmpty()) outputOffsetInterpolators.rootRInterpolator_->get(offsetR, dt);
    robot_com->rootLink()->R() = offsetR*robot_ref->rootLink()->R();
  }

  for(size_t i=0;i<robot_com->numJoints();i++) {
    if(std::find(useJoints.begin(), useJoints.end(), robot_com->joint(i)) == useJoints.end()){
      double offset = 0.0, tmpv, tmpa;
      if (!outputOffsetInterpolators.jointInterpolatorMap_[robot_com->joint(i)]->isEmpty()) outputOffsetInterpolators.jointInterpolatorMap_[robot_com->joint(i)]->get(offset, tmpv, tmpa, dt);
      robot_com->joint(i)->q() = robot_ref->joint(i)->q() + offset;
      robot_com->joint(i)->dq() = 0.0;
      robot_com->joint(i)->ddq() = 0.0;
      robot_com->joint(i)->u() = 0.0;
    }
  }

  robot_com->calcForwardKinematics();
  robot_com->calcCenterOfMass();
}

void WholeBodyPositionController::calcOutputPorts(const std::string& instance_name, WholeBodyPositionController::Ports& port, const cnoid::BodyPtr& robot_com, std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap) {
  // qCom
  if (port.m_qRef_.data.length() == robot_com->numJoints()){
    port.m_qCom_.data.length(robot_com->numJoints());
    for (int i = 0; i < robot_com->numJoints(); i++ ){
      port.m_qCom_.data[i] = robot_com->joint(i)->q();
    }
    port.m_qCom_.tm = port.m_qRef_.tm;
    port.m_qComOut_.write();
  }
  // basePos
  port.m_basePosCom_.data.x = robot_com->rootLink()->p()[0];
  port.m_basePosCom_.data.y = robot_com->rootLink()->p()[1];
  port.m_basePosCom_.data.z = robot_com->rootLink()->p()[2];
  port.m_basePosCom_.tm = port.m_qRef_.tm;
  port.m_basePosComOut_.write();
  // baseRpy
  cnoid::Vector3 outputBaseRpy = cnoid::rpyFromRot(robot_com->rootLink()->R());
  port.m_baseRpyCom_.data.r = outputBaseRpy[0];
  port.m_baseRpyCom_.data.p = outputBaseRpy[1];
  port.m_baseRpyCom_.data.y = outputBaseRpy[2];
  port.m_baseRpyCom_.tm = port.m_qRef_.tm;
  port.m_baseRpyComOut_.write();
  // basePose
  port.m_basePoseCom_.tm = port.m_qRef_.tm;
  port.m_basePoseCom_.data.position.x = robot_com->rootLink()->p()[0];
  port.m_basePoseCom_.data.position.y = robot_com->rootLink()->p()[1];
  port.m_basePoseCom_.data.position.z = robot_com->rootLink()->p()[2];
  port.m_basePoseCom_.data.orientation.r = outputBaseRpy[0];
  port.m_basePoseCom_.data.orientation.p = outputBaseRpy[1];
  port.m_basePoseCom_.data.orientation.y = outputBaseRpy[2];
  port.m_basePoseComOut_.write();
  // m_baseTform
  port.m_baseTformCom_.data.length(12);
  for(int i=0;i<3;i++) port.m_baseTformCom_.data[i] = robot_com->rootLink()->p()[i];
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) {
      port.m_baseTformCom_.data[3+i*3+j] = robot_com->rootLink()->R()(i,j);// row major
    }
  }
  port.m_baseTformCom_.tm = port.m_qRef_.tm;
  port.m_baseTformComOut_.write();
  // primitiveCommandCom
  port.m_primitiveCommandCom_ = port.m_primitiveCommandRef_;
  port.m_primitiveCommandCom_.tm = port.m_qRef_.tm;
  for(int i=0;i<port.m_primitiveCommandCom_.data.length();i++){
    std::shared_ptr<primitive_motion_level_tools::PrimitiveState> primitiveCommand = primitiveCommandMap[std::string(port.m_primitiveCommandCom_.data[i].name)];
    cnoid::Position pose = cnoid::Position::Identity();
    if(std::string(port.m_primitiveCommandCom_.data[i].name) == "com") pose.translation() = robot_com->centerOfMass();
    else if (robot_com->link(primitiveCommand->parentLinkName())) pose = robot_com->link(primitiveCommand->parentLinkName())->T() * primitiveCommand->localPose();
    else std::cerr << "\x1b[31m[" << instance_name << "] " << "failed to find link[" << port.m_primitiveCommandCom_.data[i].name << "]" << "\x1b[39m" << std::endl;
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

RTC::ReturnCode_t WholeBodyPositionController::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  WholeBodyPositionController::readPorts(instance_name, this->ports_);

  // calc reference state from inport (without ee. q, basepos and baserpy only)
  WholeBodyPositionController::calcReferenceRobot(instance_name, this->ports_, this->m_robot_ref_);

  // get primitive motion level command
  WholeBodyPositionController::getPrimitiveCommand(instance_name, this->ports_, dt, this->primitiveCommandMap_);

  // get self collision states for collision avoidance
  WholeBodyPositionController::getCollision(instance_name, this->ports_, this->collisions_);

  // mode遷移を実行
  WholeBodyPositionController::processModeTransition(instance_name, this->mode_, dt, this->m_robot_ref_, this->m_robot_com_, this->outputOffsetInterpolators_);

  if(this->mode_.isRunning()) {
    // use_joints以外はrobot_refをrobot_comへ
    WholeBodyPositionController::passThrough(instance_name, this->m_robot_ref_, this->m_robot_com_, this->outputOffsetInterpolators_, dt, this->useJoints_);

    if(this->mode_.isInitialize()){
      WholeBodyPositionController::preProcessForControl(instance_name, this->positionController_);
    }

    this->positionController_.control(this->primitiveCommandMap_, this->collisions_, this->m_robot_ref_, this->useJoints_, this->jointLimitTablesMap_, this->m_robot_com_, dt, this->debugLevel_);

  } else {
    // robot_refをrobot_comへ
    WholeBodyPositionController::passThrough(instance_name, this->m_robot_ref_, this->m_robot_com_, this->outputOffsetInterpolators_, dt);
  }

  // write outport
  WholeBodyPositionController::calcOutputPorts(instance_name, this->ports_, this->m_robot_com_, this->primitiveCommandMap_);

  this->loop_++;
  return RTC::RTC_OK;
}


bool WholeBodyPositionController::startControl(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool WholeBodyPositionController::stopControl(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool WholeBodyPositionController::setParams(const whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;
  std::vector<cnoid::LinkPtr> useJointsNew;
  for(int i=0;i<i_param.useJoints.length();i++){
    cnoid::LinkPtr link = this->m_robot_com_->link(std::string(i_param.useJoints[i]));
    if(link && (link->isRoot() || link->jointId()>=0)) useJointsNew.push_back(link);
  }
  if(this->mode_.isRunning()){
    for(int i=0;i<this->useJoints_.size();i++){
      if(std::find(useJointsNew.begin(), useJointsNew.end(), this->useJoints_[i]) == useJointsNew.end()) {
        if(this->useJoints_[i] == this->m_robot_com_->rootLink()) {
          this->outputOffsetInterpolators_.rootpInterpolator_->reset(this->m_robot_com_->rootLink()->p()-this->m_robot_ref_->rootLink()->p(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
          this->outputOffsetInterpolators_.rootpInterpolator_->setGoal(cnoid::Vector3::Zero(), 3.0);
          this->outputOffsetInterpolators_.rootRInterpolator_->reset(this->m_robot_com_->rootLink()->R()*this->m_robot_ref_->rootLink()->R().transpose(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
          this->outputOffsetInterpolators_.rootRInterpolator_->setGoal(cnoid::Matrix3d::Identity(), 3.0);
        }else{
          this->outputOffsetInterpolators_.jointInterpolatorMap_[this->useJoints_[i]]->reset(this->useJoints_[i]->q()-this->m_robot_ref_->joint(this->useJoints_[i]->jointId())->q(),0.0,0.0);
          this->outputOffsetInterpolators_.jointInterpolatorMap_[this->useJoints_[i]]->setGoal(0.0,3.0);
        }
      }
    }
  }
  this->useJoints_ = useJointsNew;

  switch(i_param.solveMode){
  case whole_body_position_controller::WholeBodyPositionControllerService::SolveModeEnum::MODE_FULLBODY:
    this->positionController_.solveMode() = WholeBodyPosition::PositionController::MODE_FULLBODY;
    break;
  case whole_body_position_controller::WholeBodyPositionControllerService::SolveModeEnum::MODE_PRIORITIZED:
  default:
    this->positionController_.solveMode() = WholeBodyPosition::PositionController::MODE_PRIORITIZED;
    break;
  }
  this->positionController_.followRootLink() = i_param.followRootLink;
  return true;
}


bool WholeBodyPositionController::getParams(whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  i_param.useJoints.length(this->useJoints_.size());
  for(int i=0;i<this->useJoints_.size();i++) i_param.useJoints[i] = this->useJoints_[i]->name().c_str();

  switch(this->positionController_.solveMode()){
  case WholeBodyPosition::PositionController::MODE_FULLBODY:
    i_param.solveMode = whole_body_position_controller::WholeBodyPositionControllerService::SolveModeEnum::MODE_FULLBODY;
    break;
  case WholeBodyPosition::PositionController::MODE_PRIORITIZED:
  default:
    i_param.solveMode = whole_body_position_controller::WholeBodyPositionControllerService::SolveModeEnum::MODE_PRIORITIZED;
    break;
  }
  i_param.followRootLink = this->positionController_.followRootLink();
  return true;
}

RTC::ReturnCode_t WholeBodyPositionController::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t WholeBodyPositionController::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t WholeBodyPositionController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void WholeBodyPositionControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyPositionController_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyPositionController>, RTC::Delete<WholeBodyPositionController>);
    }
};
