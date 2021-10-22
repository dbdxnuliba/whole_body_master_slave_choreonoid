#include "WholeBodyTorqueController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>
#include <cnoid/ValueTree>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* WholeBodyTorqueController_spec[] = {
  "implementation_id", "WholeBodyTorqueController",
  "type_name",         "WholeBodyTorqueController",
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

WholeBodyTorqueController::WholeBodyTorqueController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t WholeBodyTorqueController::onInitialize(){

  addInPort("qRefIn", this->ports_.m_qRefIn_);
  addInPort("pgainPercentageRefIn", this->ports_.m_pgainPercentageRefIn_);
  addInPort("dgainPercentageRefIn", this->ports_.m_dgainPercentageRefIn_);
  addInPort("basePosRefIn", this->ports_.m_basePosRefIn_);
  addInPort("baseRpyRefIn", this->ports_.m_baseRpyRefIn_);
  addInPort("primitiveCommandRefIn", this->ports_.m_primitiveCommandRefIn_);
  addInPort("qActIn", this->ports_.m_qActIn_);
  addInPort("imuActIn", this->ports_.m_imuActIn_);
  addInPort("collisionActIn", this->ports_.m_collisionActIn_);
  addOutPort("tauComOut", this->ports_.m_tauComOut_);
  addOutPort("qComOut", this->ports_.m_qComOut_);
  addOutPort("pgainPercentageComIn", this->ports_.m_pgainPercentageComOut_);
  addOutPort("dgainPercentageComIn", this->ports_.m_dgainPercentageComOut_);
  addOutPort("basePosActOut", this->ports_.m_basePosActOut_);
  addOutPort("baseRpyActOut", this->ports_.m_baseRpyActOut_);
  addOutPort("basePoseActOut", this->ports_.m_basePoseActOut_);
  addOutPort("baseTformActOut", this->ports_.m_baseTformActOut_);
  this->ports_.m_WholeBodyTorqueControllerServicePort_.registerProvider("service0", "WholeBodyTorqueControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_WholeBodyTorqueControllerServicePort_);

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
  this->m_robot_act_ = robot->clone();
  this->m_robot_com_ = robot->clone();

  this->useJoints_.push_back(this->m_robot_com_->rootLink());
  for(int i=0;i<this->m_robot_com_->numJoints();i++) this->useJoints_.push_back(this->m_robot_com_->joint(i));

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

  for(int i=0;i<this->m_robot_com_->numJoints();i++){
    double climit, gearRatio, torqueConst;
    if(!this->m_robot_ref_->joint(i)->info()->read("climit",climit) ||
       !this->m_robot_ref_->joint(i)->info()->read("gearRatio",gearRatio) ||
       !this->m_robot_ref_->joint(i)->info()->read("torqueConst",torqueConst) ||
       gearRatio == 0.0 ||
       torqueConst == 0.0){
      std::cerr << "\x1b[31m[" << m_profile.instance_name << "] climit, gearRatio, torqueConst do not exist for [" <<this->m_robot_ref_->joint(i)->name() << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
  }

  {
    std::string maxTorqueStr;
    if(this->getProperties().hasKey("max_torque")) maxTorqueStr = std::string(this->getProperties()["max_torque"]);
    else maxTorqueStr = std::string(this->m_pManager->getConfig()["max_torque"]); // 引数 -o で与えたプロパティを捕捉
    std::stringstream ss(maxTorqueStr);
    std::vector<double> maxTorque;
    std::string tmp;
    while (std::getline(ss, tmp, ',')) maxTorque.push_back(std::stod(tmp));
    if(maxTorque.size()==this->m_robot_com_->numJoints()){
      std::cerr << "[" << this->m_profile.instance_name << "] max_torque: " << jointLimitTableStr<<std::endl;
      for(int i=0;i<this->m_robot_com_->numJoints();i++){
        double climit, gearRatio, torqueConst;
        this->m_robot_ref_->joint(i)->info()->read("climit",climit);
        this->m_robot_ref_->joint(i)->info()->read("gearRatio",gearRatio);
        this->m_robot_ref_->joint(i)->info()->read("torqueConst",torqueConst);
        this->m_robot_com_->joint(i)->setInfo("climit",std::min(climit, maxTorque[i] / gearRatio / torqueConst));
      }
    }
  }

  this->outputInterpolators_.qComFilter_hz_ = 1000.0; // テンポラリ. 後でperiodic_rateで上書きする
  for(int i=0;i<this->m_robot_com_->numJoints();i++) {
    this->outputInterpolators_.qOffsetInterpolatorMap_[this->m_robot_com_->joint(i)] = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);
    this->outputInterpolators_.outputRatioInterpolatorMap_[this->m_robot_com_->joint(i)] = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);
    std::shared_ptr<cpp_filters::IIRFilter<double> > filter = std::make_shared<cpp_filters::IIRFilter<double> >();
    filter->setParameterAsBiquad(500,1/sqrt(2),this->outputInterpolators_.qComFilter_hz_,this->m_robot_com_->joint(i)->q());
    this->outputInterpolators_.qComFilterMap_[this->m_robot_com_->joint(i)] = filter;
  }

  return RTC::RTC_OK;
}

void WholeBodyTorqueController::readPorts(const std::string& instance_name, WholeBodyTorqueController::Ports& port) {
  if(port.m_qRefIn_.isNew()) port.m_qRefIn_.read();
  if(port.m_pgainPercentageRefIn_.isNew()) port.m_pgainPercentageRefIn_.read();
  if(port.m_dgainPercentageRefIn_.isNew()) port.m_dgainPercentageRefIn_.read();
  if(port.m_basePosRefIn_.isNew()) port.m_basePosRefIn_.read();
  if(port.m_baseRpyRefIn_.isNew()) port.m_baseRpyRefIn_.read();
  if(port.m_primitiveCommandRefIn_.isNew()) port.m_primitiveCommandRefIn_.read();

  if(port.m_qActIn_.isNew()) port.m_qActIn_.read();
  if(port.m_imuActIn_.isNew()) port.m_imuActIn_.read();
  if(port.m_collisionActIn_.isNew()) port.m_collisionActIn_.read();
}

  // calc reference state (without ee. q, basepos and baserpy only)
void WholeBodyTorqueController::calcReferenceRobot(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, cnoid::BodyPtr& robot) {
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

void WholeBodyTorqueController::calcActualRobot(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, cnoid::BodyPtr& robot) {
  if(port.m_qAct_.data.length() == robot->numJoints()){
    for ( int i = 0; i < robot->numJoints(); i++ ){
      robot->joint(i)->q() = port.m_qAct_.data[i];
    }
    robot->calcForwardKinematics();

    if(!robot->rootLink()->isFixedJoint()){
      cnoid::AccelerationSensorPtr imu = robot->findDevice<cnoid::AccelerationSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(port.m_imuAct_.data.r, port.m_imuAct_.data.p, port.m_imuAct_.data.y);
      robot->rootLink()->R() = actR * (imuR.transpose() * robot->rootLink()->R());
      robot->calcForwardKinematics();
    }
  }
}

void WholeBodyTorqueController::getPrimitiveCommand(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, double dt, std::map<std::string, std::shared_ptr<WholeBodyTorque::PrimitiveCommand> >& primitiveCommandMap) {
  // 消滅したEndEffectorを削除
  for(std::map<std::string, std::shared_ptr<WholeBodyTorque::PrimitiveCommand> >::iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); ) {
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
      primitiveCommandMap[std::string(port.m_primitiveCommandRef_.data[i].name)] = std::make_shared<WholeBodyTorque::PrimitiveCommand>(std::string(port.m_primitiveCommandRef_.data[i].name));
    }
  }
  // 各指令値の反映
  for(size_t i=0;i<port.m_primitiveCommandRef_.data.length();i++){
    const primitive_motion_level_msgs::PrimitiveStateIdl& idl = port.m_primitiveCommandRef_.data[i];
    std::shared_ptr<WholeBodyTorque::PrimitiveCommand> state = primitiveCommandMap[std::string(idl.name)];
    state->updateFromIdl(idl);
    state->updateTargetForOneStep(dt);
  }
}

void WholeBodyTorqueController::getCollision(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, std::vector<std::shared_ptr<WholeBodyTorque::Collision> >& collisions) {
  collisions.resize(port.m_collisionAct_.data.length());
  // 各collisionの反映
  for(size_t i=0;i<port.m_collisionAct_.data.length();i++){
    const collision_checker_msgs::CollisionIdl& idl = port.m_collisionAct_.data[i];
    if(!collisions[i]) collisions[i] = std::make_shared<WholeBodyTorque::Collision>();
    collisions[i]->updateFromIdl(idl);
  }
}


void WholeBodyTorqueController::processModeTransition(const std::string& instance_name, WholeBodyTorqueController::ControlMode& mode, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com, WholeBodyTorqueController::OutputInterpolators& outputInterpolators, const std::vector<cnoid::LinkPtr>& useJoints){
  switch(mode.now()){
  case WholeBodyTorqueController::ControlMode::MODE_SYNC_TO_CONTROL:
    if(mode.pre() == WholeBodyTorqueController::ControlMode::MODE_IDLE){
      for(int i=0;i<useJoints.size();i++) {
        WholeBodyTorqueController::enableJoint(useJoints[i],outputInterpolators);
      }
    }
    mode.setNextMode(WholeBodyTorqueController::ControlMode::MODE_CONTROL);
    break;
  case WholeBodyTorqueController::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == WholeBodyTorqueController::ControlMode::MODE_CONTROL){
      for(int i=0;i<robot_com->numJoints();i++) {
        WholeBodyTorqueController::disableJoint(useJoints[i],robot_ref,outputInterpolators);
      }
    }
    mode.setNextMode(WholeBodyTorqueController::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void WholeBodyTorqueController::preProcessForControl(const std::string& instance_name, WholeBodyTorque::TorqueController& torqueController) {
  torqueController.reset();
}

void WholeBodyTorqueController::calcq(const std::string& instance_name, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_act, cnoid::BodyPtr& robot_com, WholeBodyTorqueController::OutputInterpolators& outputInterpolators, double dt, const std::vector<cnoid::LinkPtr>& useJoints){
  if(outputInterpolators.qComFilter_hz_ != 1.0 / dt) {
    outputInterpolators.qComFilter_hz_ = 1.0 / dt;
    for(std::unordered_map<cnoid::LinkPtr, std::shared_ptr<cpp_filters::IIRFilter<double> > >::iterator it=outputInterpolators.qComFilterMap_.begin(); it!=outputInterpolators.qComFilterMap_.end(); it++) {
      it->second->setParameterAsBiquad(500,1/sqrt(2),outputInterpolators.qComFilter_hz_,it->second->get());
    }
  }

  for(size_t i=0;i<robot_com->numJoints();i++) {
    if(std::find(useJoints.begin(), useJoints.end(), robot_com->joint(i)) == useJoints.end()){
      double offset = 0.0, tmpv, tmpa;
      if (!outputInterpolators.qOffsetInterpolatorMap_[robot_com->joint(i)]->isEmpty()) outputInterpolators.qOffsetInterpolatorMap_[robot_com->joint(i)]->get(offset, tmpv, tmpa, dt);
      robot_com->joint(i)->q() = robot_ref->joint(i)->q() + offset;
    }else{
      robot_com->joint(i)->q() = outputInterpolators.qComFilterMap_[robot_com->joint(i)]->passFilter(robot_act->joint(i)->q());
    }
    robot_com->joint(i)->dq() = 0.0;
    robot_com->joint(i)->ddq() = 0.0;
  }
}

void WholeBodyTorqueController::calcOutputPorts(const std::string& instance_name, WholeBodyTorqueController::Ports& port, const cnoid::BodyPtr& robot_com, const cnoid::BodyPtr& robot_act, WholeBodyTorqueController::OutputInterpolators& outputInterpolators, double dt) {
  // tau, q, pgainPercentage, dgainPercentage
  if (port.m_qRef_.data.length() == robot_com->numJoints() &&
      port.m_pgainPercentageRef_.data.length() == robot_com->numJoints() &&
      port.m_dgainPercentageRef_.data.length() == robot_com->numJoints() ){
    port.m_tauCom_.data.length(robot_com->numJoints());
    port.m_tauCom_.tm = port.m_qRef_.tm;
    port.m_qCom_.data.length(robot_com->numJoints());
    port.m_qCom_.tm = port.m_qRef_.tm;
    port.m_pgainPercentageCom_.data.length(robot_com->numJoints());
    port.m_pgainPercentageCom_.tm = port.m_qRef_.tm;
    port.m_dgainPercentageCom_.data.length(robot_com->numJoints());
    port.m_dgainPercentageCom_.tm = port.m_qRef_.tm;

    for (int i = 0; i < robot_com->numJoints(); i++ ){
      double outputRatio, tmpv, tmpa;
      outputInterpolators.outputRatioInterpolatorMap_[robot_com->joint(i)]->get(outputRatio, tmpv, tmpa, dt);

      port.m_tauCom_.data[i] = outputRatio * robot_com->joint(i)->u();
      port.m_qCom_.data[i] = robot_com->joint(i)->q();
      port.m_pgainPercentageCom_.data[i] = (1.0 - outputRatio) * port.m_pgainPercentageRef_.data[i];
      port.m_dgainPercentageCom_.data[i] = (1.0 - outputRatio) * port.m_dgainPercentageRef_.data[i];
    }

    port.m_tauComOut_.write();
    port.m_qComOut_.write();
    port.m_pgainPercentageComOut_.write();
    port.m_dgainPercentageComOut_.write();
  }

  // basePos
  port.m_basePosAct_.data.x = robot_act->rootLink()->p()[0];
  port.m_basePosAct_.data.y = robot_act->rootLink()->p()[1];
  port.m_basePosAct_.data.z = robot_act->rootLink()->p()[2];
  port.m_basePosAct_.tm = port.m_qRef_.tm;
  port.m_basePosActOut_.write();
  // baseRpy
  cnoid::Vector3 outputBaseRpy = cnoid::rpyFromRot(robot_act->rootLink()->R());
  port.m_baseRpyAct_.data.r = outputBaseRpy[0];
  port.m_baseRpyAct_.data.p = outputBaseRpy[1];
  port.m_baseRpyAct_.data.y = outputBaseRpy[2];
  port.m_baseRpyAct_.tm = port.m_qRef_.tm;
  port.m_baseRpyActOut_.write();
  // basePose
  port.m_basePoseAct_.tm = port.m_qRef_.tm;
  port.m_basePoseAct_.data.position.x = robot_act->rootLink()->p()[0];
  port.m_basePoseAct_.data.position.y = robot_act->rootLink()->p()[1];
  port.m_basePoseAct_.data.position.z = robot_act->rootLink()->p()[2];
  port.m_basePoseAct_.data.orientation.r = outputBaseRpy[0];
  port.m_basePoseAct_.data.orientation.p = outputBaseRpy[1];
  port.m_basePoseAct_.data.orientation.y = outputBaseRpy[2];
  port.m_basePoseActOut_.write();
  // m_baseTform
  port.m_baseTformAct_.data.length(12);
  for(int i=0;i<3;i++) port.m_baseTformAct_.data[i] = robot_act->rootLink()->p()[i];
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) {
      port.m_baseTformAct_.data[3+i*3+j] = robot_act->rootLink()->R()(i,j);// row major
    }
  }
  port.m_baseTformAct_.tm = port.m_qRef_.tm;
  port.m_baseTformActOut_.write();
}

void WholeBodyTorqueController::enableJoint(const cnoid::LinkPtr& joint_com, WholeBodyTorqueController::OutputInterpolators& outputInterpolators) {
  outputInterpolators.qComFilterMap_[joint_com]->reset(joint_com->q());
  outputInterpolators.outputRatioInterpolatorMap_[joint_com]->setGoal(1.0,3.0);
}

void WholeBodyTorqueController::disableJoint(const cnoid::LinkPtr& joint_com, const cnoid::BodyPtr& robot_ref, WholeBodyTorqueController::OutputInterpolators& outputInterpolators) {
  outputInterpolators.qOffsetInterpolatorMap_[joint_com]->reset(joint_com->q()-robot_ref->joint(joint_com->jointId())->q(),0.0,0.0);
  outputInterpolators.qOffsetInterpolatorMap_[joint_com]->setGoal(0.0,3.0);
  outputInterpolators.outputRatioInterpolatorMap_[joint_com]->setGoal(0.0,3.0);
}

RTC::ReturnCode_t WholeBodyTorqueController::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  WholeBodyTorqueController::readPorts(instance_name, this->ports_);

  // calc reference state from inport (without ee. q, basepos and baserpy only)
  WholeBodyTorqueController::calcReferenceRobot(instance_name, this->ports_, this->m_robot_ref_);

  // get primitive motion level command
  WholeBodyTorqueController::getPrimitiveCommand(instance_name, this->ports_, dt, this->primitiveCommandMap_);

  // calc actual state from inport
  WholeBodyTorqueController::calcActualRobot(instance_name, this->ports_, this->m_robot_act_);

  // get self collision states for collision avoidance
  WholeBodyTorqueController::getCollision(instance_name, this->ports_, this->collisions_);

  // mode遷移を実行
  WholeBodyTorqueController::processModeTransition(instance_name, this->mode_, this->m_robot_ref_, this->m_robot_com_, this->outputInterpolators_, this->useJoints_);

  if(this->mode_.isRunning()) {
    // q を計算
    WholeBodyTorqueController::calcq(instance_name, this->m_robot_ref_, this->m_robot_act_, this->m_robot_com_, this->outputInterpolators_, dt, this->useJoints_);

    if(this->mode_.isInitialize()){
      WholeBodyTorqueController::preProcessForControl(instance_name, this->torqueController_);
    }

    // tauを計算
    this->torqueController_.control(this->primitiveCommandMap_, this->collisions_, this->m_robot_ref_, this->jointLimitTablesMap_, this->m_robot_com_, dt, this->debugLevel_);

  } else {
    // q を計算
    WholeBodyTorqueController::calcq(instance_name, this->m_robot_ref_, this->m_robot_act_, this->m_robot_com_, this->outputInterpolators_, dt);
  }

  // write outport
  WholeBodyTorqueController::calcOutputPorts(instance_name, this->ports_, this->m_robot_com_, this->m_robot_act_, this->outputInterpolators_, dt);

  this->loop_++;
  return RTC::RTC_OK;
}


bool WholeBodyTorqueController::startControl(){
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


bool WholeBodyTorqueController::stopControl(){
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

bool WholeBodyTorqueController::setParams(const whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;
  std::vector<cnoid::LinkPtr> useJointsNew;
  for(int i=0;i<i_param.useJoints.length();i++){
    cnoid::LinkPtr link = this->m_robot_com_->link(std::string(i_param.useJoints[i]));
    if(link && link->jointId()>=0) useJointsNew.push_back(link);
  }
  if(this->mode_.isRunning()){
    for(int i=0;i<this->useJoints_.size();i++){
      if(std::find(useJointsNew.begin(), useJointsNew.end(), this->useJoints_[i]) == useJointsNew.end()) {
        WholeBodyTorqueController::enableJoint(this->useJoints_[i],this->outputInterpolators_);
      }
    }
    for(int i=0;i<useJointsNew.size();i++){
      if(std::find(this->useJoints_.begin(), this->useJoints_.end(), useJointsNew[i]) == this->useJoints_.end()) {
        WholeBodyTorqueController::disableJoint(useJointsNew[i],this->m_robot_ref_,this->outputInterpolators_);
      }
    }
  }
  this->useJoints_ = useJointsNew;

  return true;
}


bool WholeBodyTorqueController::getParams(whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  i_param.useJoints.length(this->useJoints_.size());
  for(int i=0;i<this->useJoints_.size();i++) i_param.useJoints[i] = this->useJoints_[i]->name().c_str();

  return true;
}

RTC::ReturnCode_t WholeBodyTorqueController::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t WholeBodyTorqueController::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t WholeBodyTorqueController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void WholeBodyTorqueControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyTorqueController_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyTorqueController>, RTC::Delete<WholeBodyTorqueController>);
    }
};
