#include "PrimitiveMotionLevelController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>

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

PrimitiveState::PrimitiveState(const std::string& name) :
  name_(name),
  parentLinkName(""),
  localPose_(cnoid::Position::identity()),
  targetPose_(cnoid::Position::identity()),
  targetPosePrev_(cnoid::Position::identity()),
  targetPosePrevPrev_(cnoid::Position::identity()),
  targetPositionInterpolator_(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),
  targetOrientationInterpolator_(cnoid::Matrix3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),
  targetWrench_(cnoid::Vector6::Zero()),
  targetWrenchInterpolator_(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB),
  M_(cnoid::Vector6::Zero()),
  D_(cnoid::Vector6::Zero()),
  K_(cnoid::Vector6::Zero()),
  actWrench_(cnoid::Vector6::Zero()),
  actWrenchGain_(cnoid::Vector6::Zero())
{
}

void PrimitiveState::updateFromIdl(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveState& idl) {
  this->parentLinkName_ = idl.parentLinkName;
  this->localPose_.translation()[0] = idl.localPose.position.x;
  this->localPose_.translation()[1] = idl.localPose.position.y;
  this->localPose_.translation()[2] = idl.localPose.position.z;
  this->localPose_.linear() = cnoid::rotFromRpy(idl.localPose.orientation.r,idl.localPose.orientation.p,idl.localPose.orientation.y);
  cnoid::Position pose;
  pose.translation[0] = idl.pose.x;
  pose.translation[1] = idl.pose.y;
  pose.translation[2] = idl.pose.z;
  pose.linear() = cnoid::rotFromRpy(idl.pose.orientation.r,idl.pose.orientation.p,idl.pose.orientation.y);
  this->targetPositionInterpolator_.setGoal(pose.translation(),idl.time);
  this->targetOrientationInterpolator_.setGoal(pose.linear(),idl.time);
  cnoid::Vector6 wrench; for(size_t i=0;i<6;i++) wrench[i] = idl.wrench[i];
  this->targetWrenchInterpolator_.setGoal(wrench,idl.time);
  for(size_t i=0;i<6;i++) this->M_[i] = idl.M[i];
  for(size_t i=0;i<6;i++) this->D_[i] = idl.D[i];
  for(size_t i=0;i<6;i++) this->K_[i] = idl.K[i];
  for(size_t i=0;i<6;i++) this->actWrench_[i] = idl.actWrench[i];
  for(size_t i=0;i<6;i++) this->actWrenchGain_[i] = idl.actWrenchGain[i];
}

PrimitiveMotionLevelController::PrimitiveMotionLevelController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  m_debugLevel_(0)
{
  this->ports_.m_service0_.wholebodymasterslavechoreonoid(this);
}

RTC::ReturnCode_t PrimitiveMotionLevelController::onInitialize(){
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
  this->ports_.m_PrimitiveMotionLevelControllerServicePort_.registerProvider("service0", "PrimitiveMotionLevelControllerService", this->ports_.m_service0_);
  addPort(this->ports_.m_PrimitiveMotionLevelControllerServicePort_);

  RTC::Properties& prop = getProperties();
  coil::stringTo(this->m_dt_, prop["dt"].c_str());

  this->loop_ = 0;

  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load(prop["model"]);
  if(!robot){
    RTC_WARN_STREAM("failed to load model[" << prop["model"] << "]");
    return RTC::RTC_ERROR;
  }
  this->m_robot_ref_ = robot;
  this->m_robot_act_ = robot->clone();
  this->m_robot_com_ = robot->clone();

  this->outputRatioInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,cpp_filters::HOFFARBIB);

  for(size_t i=0; i<this->m_robot_com_->numJoints(); i++){
    this->outputSmoothingInterpolatorMap_[this->m_robot_com_->joint[i]->name()] = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,cpp_filters::CUBICSPLINE); // or HOFFARBIB, QUINTICSPLINE
  }
  this->avg_q_vel_ = 4.0;// 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい
  this->avg_q_acc_ = 16.0; // all joint max avarage acc = 16.0 rad/s^2

  this->fixedFrame_.parentLinkName_ = this->m_robot_ref_->rootLink()->name();
  this->fixedFrame_.localPose = cnoid::Position::Identity();

  return RTC::RTC_OK;
}

namespace PrimitiveMotionLevelControllerImpl {

  void readPorts(PrimitiveMotionLevelController::Ports& port) {
    if(port.m_qRefIn_.isNew()) port.m_qRefIn_.read();
    if(port.m_basePosRefIn_.isNew()) port.m_basePosRefIn_.read();
    if(port.m_baseRpyRefIn_.isNew()) port.m_baseRpyRefIn_.read();
    if(port.m_primitiveCommandRefIn_.isNew()) port.m_primitiveCommandRefIn_.read();
    if(port.m_qActIn_.isNew()) port.m_qActIn_.read();
    if(port.m_imuActIn_.isNew()) port.m_imuActIn_.read();
  }

  // calc reference state (without ee. q, basepos and baserpy only)
  void calcReferenceRobot(const PrimitiveMotionLevelController::Ports& port, cnoid::BodyPtr& robot) {
    if(port.m_qRef_.length() == robot->numJoints()){
      for ( int i = 0; i < robot->numJoints(); i++ ){
        robot->joint(i)->q() = port.m_qRef_.data[i];
      }
    }
    robot->p()[0] = port.m_basePosRef_.data.x;
    robot->p()[1] = port.m_basePosRef_.data.y;
    robot->p()[2] = port.m_basePosRef_.data.z;
    robot->rootLink()->R() = cnoid::rotFromRpy(port.m_baseRpyRef_.data.r, port.m_baseRpyRef_.data.p, port.m_baseRpyRef_.data.y);
    robot->calcForwardKinematics();
  }

  // calc actial state from inport
  void calcActualRobot(const PrimitiveMotionLevelController::Ports& port, cnoid::BodyPtr& robot_act) {
    if(port.m_qAct_.length() == robot_act->numJoints()){
      for ( int i = 0; i < robot_act->numJoints(); i++ ){
        robot_act->joint(i)->q() = port.m_qAct_.data[i];
      }
      robot_act->calcForwardKinematics();

      cnoid::AccelerationSensorPtr imu = robot_act->findDevice<cnoid::AccelerationSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu_act->link()->R() * imu_act->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(port.m_imuAct_.data.r, port.m_imuAct_.data.p, port.m_imuAct_.data.y);
      robot_act->rootLink()->R = (actR * (imuR.transpose() * robot_act->rootLink()->R())).eval();
      robot_act->calcForwardKinematics();
    }
  }

  void getPrimitiveCommand(const PrimitiveMotionLevelController::Ports& port, PrimitiveMotionLevelController::frame& fixedFrame, bool& fixedFrameWarped, std::map<std::string, std::shared_ptr<PrimitiveMotionLevelController::PrimitiveState> >& primitiveStateMap) {

    // fixed frame
    if(fixedFrame.parentLinkName != port.m_primitiveCommandRef_.fixedFrame.parentLinkName){
      fixedFrameWarped = true;
    }else{
      fixedFrameWarped = false;
    }

    fixedFrame.parentLinkName = port.m_primitiveCommandRef_.fixedFrame.parentLinkName;
    fixedFrame.localPose.translation()[0] = port.m_primitiveCommandRef_.fixedFrame.localPose.position.x;
    fixedFrame.localPose.translation()[1] = port.m_primitiveCommandRef_.fixedFrame.localPose.position.y;
    fixedFrame.localPose.translation()[2] = port.m_primitiveCommandRef_.fixedFrame.localPose.position.z;
    fixedFrame.localPose.linear() = cnoid::rotFromRpy(port.m_primitiveCommandRef_.fixedFrame.localPose.orientation.r,port.m_primitiveCommandRef_.fixedFrame.localPose.orientation.p,port.m_primitiveCommandRef_.fixedFrame.localPose.orientation.y);

    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevelController::PrimitiveState> >::iterator it = primitiveStateMap.begin(); it != primitiveStateMap.end(); ) {
      if (std::find_if(port.m_primitiveCommandRef_.command.data.begin(),port.m_primitiveCommandRef_.command.data.end(),[&](WholeBodyMasterSlaveChoreonoidIdl::primitiveState x){return x.name==it->first;}) == port.m_primitiveCommandRef_.command.data.end()) {
        it = primitiveStateMap.erase(it);
      }
      else {
        ++it;
      }
    }
    // 増加したEndEffectorの反映
    for(size_t i=0;i<port.m_primitiveCommandRef_.command.data.length();i++){
      if(primitiveStateMap.find(port.m_primitiveCommandRef_.command.data[i])==primitiveStateMap.end()){
        primitiveStateMap[port.m_primitiveCommandRef_.command.data[i].name] = std::make_shared<PrimitiveMotionLevelController::PrimitiveState>(port.m_primitiveCommandRef_.command.data[i].name);
      }
    }
    // 各指令値の反映
    for(size_t i=0;i<port.m_primitiveCommandRef_.command.data.length();i++){
      WholeBodyMasterSlaveChoreonoidIdl::PrimitiveState& idl = primitiveStateMap[port.m_primitiveCommandRef_.command.data[i]];
      std::shared_ptr<PrimitiveMotionLevelController::PrimitiveState> state = primitiveStateMap[command.name];
      state->updateFromIdl(idl);
    }
  }

  void moveActualRobotToFixedFrame(const PrimitiveMotionLevelController::frame& fixedFrame, const cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_act) {
    if(!robot_ref->link(fixedFrame.parentLinkName_)) {
      std::cerr << "[PrimitiveMotionLevelController::moveActualRobotToFixedFrame] link " << fixedFrame.parentLinkName_ << " not found" << std::endl;
      return;
    }
    cnoid::Position fixedFrameRef = robot_ref->link(fixedFrame.parentLinkName_)->T() * fixedFrame.localPose_;
    cnoid::Position fixedFrameAct = robot_act->link(fixedFrame.parentLinkName_)->T() * fixedFrame.localPose_;
    cnoid::Position trans = fixedFrameRef * fixedFrameAct.inverse();
    {
      // 鉛直軸は修正しない
      Eigen::Quaterniond rot;
      rot.setFromTwoVectors(trans.linear() * Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
      trans.linear() = (rot * trans.linear()).eval();
    };
    robot_act->rootLink()->T() = (trans * robot_act->rootLink()->T()).eval();
  }

  void processTransition(PrimitiveMotionLevelController::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> >& outputRatioInterpolator, double dt){
    double tmp;
    switch(mode.now()){
    case PrimitiveMotionLevelController::ControlMode::MODE_SYNC_TO_CONTROL:
      if(mode.pre() == PrimitiveMotionLevelController::ControlMode::MODE_IDLE){
        outputRatioInterpolator->setGoal(1.0, 3.0);
      }
      if (!outputRatioInterpolator->isEmpty() ){
        outputRatioInterpolator->get(&tmp, dt);
      }else{
        mode.setNextMode(PrimitiveMotionLevelController::ControlMode::MODE_CONTROL);
      }
      break;
    case PrimitiveMotionLevelController::ControlMode::MODE_SYNC_TO_IDLE:
      if(mode.pre() == PrimitiveMotionLevelController::ControlMode::MODE_CONTROL){
        outputRatioInterpolator->setGoal(0.0, 3.0, true);
      }
      if (outputRatioInterpolator->isEmpty()) {
        outputRatioInterpolator->get(&tmp, dt);
      }else{
        mode.setNextMode(PrimitiveMotionLevelController::ControlMode::MODE_IDLE);
      }
      break;
    }
    mode.update();
  }

  void preProcessForControl() {

  }

  void passThrough(const cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_com, std::unordered_map<std::string, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > > outputSmoothingInterpolatorMap){
    robot_com->rootLink()->T() = robot_ref->rootLink()->T();
    for(size_t i=0;i<robot_com->numJoints();i++) {
      robot_com->joint(i)->q() = robot_ref->joint(i)->q();
      robot_com->joint(i)->dq() = 0.0;
      robot_com->joint(i)->ddq() = 0.0;
      robot_com->joint(i)->u() = 0.0;
      outputSmoothingInterpolatorMap[robot_com->joint(i)->name()]->reset(robot_com->joint(i)->q());
    }

    robot_com->calcForwardKinematics();
  }

  void calcOutputPorts(PrimitiveMotionLevelController::Ports& port, double output_ratio, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com) {
    // qCom
    if (port.m_qRef_.data.length() == robot_com->numJoints()){
      port.m_qCom_.data.length(robot_com->numJoints());
      for (int i = 0; i < robot_com->numJoints(); i++ ){
        port.m_qCom_.data[i] = output_ratio * robot_com->joint(i)->q() + (1 - output_ratio) * robot_ref->joint(i)->q();
      }
      port.m_qCom.tm = port.m_qRef.tm;
      port.m_qComOut_.write();
    }
    // basePos
    cnoid::Vector3 ouputBasePos = output_ratio * robot_com->rootLink()->p() + (1 - output_ratio) * robot_ref->rootLink()->p();
    port.m_basePosCom_.data.x = ouputBasePos[0];
    port.m_basePosCom_.data.y = ouputBasePos[1];
    port.m_basePosCom_.data.z = ouputBasePos[2];
    m_basePosCom.tm = m_qRef.tm;
    m_basePosComOut_.write();
    // baseRpy
    cnoid::Vector3 outputBaseRpy = cnoid::rpyFromRot(cnoid::Quaterniond(robot_ref->rootLink()->R()).slerp(output_ratio, robot_com->rootLink()->R()));
    port.m_baseRpyCom_.data.r = outputBaseRpy[0];
    port.m_baseRpyCom_.data.p = outputBaseRpy[1];
    port.m_baseRpyCom_.data.y = outputBaseRpy[2];
    port.m_baseRpyCom_.tm = m_qRef.tm;
    port.m_baseRpyComOut_.write();
    // eestate TODO
    port.m_eeStateAct_.tm = m_qRef.tm;
    port.m_eeStateActOut_.write();
  }

}

RTC::ReturnCode_t PrimitiveMotionLevelController::onExecute(RTC::UniqueId ec_id){

  time_report_str.clear();
  clock_gettime(CLOCK_REALTIME, &startT);

  // read ports
  PrimitiveMotionLevelControllerImpl::readPorts(this->port_);

  // calc reference state from inport (without ee. q, basepos and baserpy only)
  PrimitiveMotionLevelControllerImpl::calcReferenceRobot(this->port_, this->m_robot_ref_);

  // calc actial state from inport
  PrimitiveMotionLevelControllerImpl::calcActualRobot(this->port_, this->m_robot_act_);

  // get primitive motion level command
  bool fixedFrameWarped = false;
  PrimitiveMotionLevelControllerImpl::getPrimitiveCommand(this->port_, this->fixedFrame_, fixedFrameWarped, this->primitiveStateMap_);

  // match frame of actual robot to reference robot.
  PrimitiveMotionLevelControllerImpl::moveActualRobotToFixedFrame(this->fixedFrame_, this->m_robot_ref_, this->m_robot_act_);

  // mode遷移を実行
  PrimitiveMotionLevelControllerImpl::processTransition(this->mode_, this->outputRatioInterpolator_, this->m_dt_);

  if(this->mode_.isRunning()) {
    if(this->mode_.isInitialize()){
      PrimitiveMotionLevelControllerImpl::preProcessForControl();
    }\
    wbms->update();//////HumanSynchronizerの主要処理

    solveFullbodyIK(wbms->rp_ref_out);

    smoothingJointAngles(fik->m_robot, m_robot_vsafe);
  } else {
    // robot_refがそのままrobot_comになる
    PrimitiveMotionLevelControllerImpl::passThrough(this->m_robot_ref_, this->m_robot_com_, this->outputSmoothingInterpolatorMap_);
  }

  // write outport
  double output_ratio; this->outputRatioInterpolator_->get(output_ratio);
  PrimitiveMotionLevelControllerImpl::calcOutputPorts(this->port_, output_ratio, this->m_robot_ref_, this->m_robot_com_);

  loop++;
  return RTC::RTC_OK;
}


void PrimitiveMotionLevelController::solveFullbodyIK(HumanPose& ref){
    std::vector<IKConstraint> ikc_list;
    if(wbms->legged){ // free baselink, lleg, rleg, larm, rarm setting
        {
            IKConstraint tmp;
            tmp.target_link_name    = fik->m_robot->rootLink()->name;
            tmp.localPos            = hrp::Vector3::Zero();
            tmp.localR              = hrp::Matrix33::Identity();
            tmp.targetPos           = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
            tmp.targetRpy           = ref.stgt("com").abs.rpy();
            tmp.constraint_weight   << 0, 0, 0, 0.1, 0.1, 0.1;
            tmp.rot_precision       = deg2rad(3);
            ikc_list.push_back(tmp);
        }
        for(auto leg : {"lleg","rleg"}){
            if(has(wbms->wp.use_targets, leg)){
                IKConstraint tmp;
                tmp.target_link_name    = ee_ikc_map[leg].target_link_name;
                tmp.localPos            = ee_ikc_map[leg].localPos;
                tmp.localR              = ee_ikc_map[leg].localR;
                tmp.targetPos           = ref.stgt(leg).abs.p;
                tmp.targetRpy           = ref.stgt(leg).abs.rpy();
                tmp.constraint_weight   = wbms->rp_ref_out.tgt[rf].is_contact() ? hrp::dvector6::Constant(3) : hrp::dvector6::Constant(0.1);
                ikc_list.push_back(tmp);
            }
        }
        for(auto arm : {"larm","rarm"}){
            if(has(wbms->wp.use_targets, arm)){
                IKConstraint tmp;
                tmp.target_link_name    = ee_ikc_map[arm].target_link_name;
                tmp.localPos            = ee_ikc_map[arm].localPos;
                tmp.localR              = ee_ikc_map[arm].localR;
                tmp.targetPos           = ref.stgt(arm).abs.p;
                tmp.targetRpy           = ref.stgt(arm).abs.rpy();
                tmp.constraint_weight   = hrp::dvector6::Constant(0.1);
                tmp.pos_precision       = 3e-3;
                tmp.rot_precision       = deg2rad(3);
                ikc_list.push_back(tmp);
            }
        }
        if(has(wbms->wp.use_targets, "com")){
            IKConstraint tmp;
            tmp.target_link_name    = "COM";
            tmp.localPos            = hrp::Vector3::Zero();
            tmp.localR              = hrp::Matrix33::Identity();
            tmp.targetPos           = ref.stgt("com").abs.p + static_balancing_com_offset;// COM height will not be constraint
            tmp.targetRpy           = hrp::Vector3::Zero();//reference angular momentum
            tmp.constraint_weight   << 3,3,0.01,0,0,0;
            ikc_list.push_back(tmp);
        }
    }else{ // fixed baselink, larm, rarm setting
        {
            IKConstraint tmp;
            tmp.target_link_name    = fik->m_robot->rootLink()->name;
            tmp.localPos            = hrp::Vector3::Zero();
            tmp.localR              = hrp::Matrix33::Identity();
            tmp.targetPos           = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
            tmp.targetRpy           = hrp::Vector3::Zero();
            tmp.constraint_weight   << hrp::dvector6::Constant(1);
            tmp.rot_precision       = deg2rad(3);
            ikc_list.push_back(tmp);
        }
        for(auto arm : {"larm","rarm"}){
            if(has(wbms->wp.use_targets, arm)){
                IKConstraint tmp;
                tmp.target_link_name    = ee_ikc_map[arm].target_link_name;
                tmp.localPos            = ee_ikc_map[arm].localPos;
                tmp.localR              = ee_ikc_map[arm].localR;
                tmp.targetPos           = ref.stgt(arm).abs.p;
                tmp.targetRpy           = ref.stgt(arm).abs.rpy();
                tmp.constraint_weight   << 1, 1, 1, 0.1, 0.1, 0.1;
                tmp.pos_precision       = 3e-3;
                tmp.rot_precision       = deg2rad(3);
                ikc_list.push_back(tmp);
            }
        }
    }
    // common head setting
    if(has(wbms->wp.use_targets, "head")){
        if(fik->m_robot->link("HEAD_JOINT1") != NULL){
            IKConstraint tmp;
            tmp.target_link_name = "HEAD_JOINT1";
            tmp.targetRpy = ref.stgt("head").abs.rpy();
            tmp.constraint_weight << 0,0,0,0,0.1,0.1;
            tmp.rot_precision = deg2rad(1);
            ikc_list.push_back(tmp);
        }
        if(fik->m_robot->link("HEAD_P") != NULL){
            IKConstraint tmp;
            tmp.target_link_name = "HEAD_P";
            tmp.targetRpy = ref.stgt("head").abs.rpy();
            tmp.constraint_weight << 0,0,0,0,0.1,0.1;
            tmp.rot_precision = deg2rad(1);
            ikc_list.push_back(tmp);
        }
    }
    if(wbms->rp_ref_out.tgt[rf].is_contact()){
        sccp->avoid_priority.head(12).head(6).fill(4);
    }else{
        sccp->avoid_priority.head(12).head(6).fill(3);
    }
    if(wbms->rp_ref_out.tgt[lf].is_contact()){
        sccp->avoid_priority.head(12).tail(6).fill(4);
    }else{
        sccp->avoid_priority.head(12).tail(6).fill(3);
    }

//    sccp->checkCollision();

    for(int i=0;i<sccp->collision_info_list.size();i++){
        IKConstraint tmp;
        double val_w = (sccp->collision_info_list[i].dist_safe - sccp->collision_info_list[i].dist_cur)*1e2;
        LIMIT_MAX(val_w, 3);
        tmp.constraint_weight << val_w,val_w,val_w,0,0,0;
//        tmp.constraint_weight << 3,3,3,0,0,0;
        double margin = 1e-3;
        if(sccp->avoid_priority(sccp->collision_info_list[i].id0) > sccp->avoid_priority(sccp->collision_info_list[i].id1)){
            tmp.localPos = sccp->collision_info_list[i].cp1_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id1)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }else if(sccp->avoid_priority(sccp->collision_info_list[i].id0) < sccp->avoid_priority(sccp->collision_info_list[i].id1)){
            tmp.localPos = sccp->collision_info_list[i].cp0_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id0)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }else{
            tmp.localPos = sccp->collision_info_list[i].cp1_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id1)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
            tmp.localPos = sccp->collision_info_list[i].cp0_local;
            tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id0)->name;
            tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
            ikc_list.push_back(tmp);
        }
//        ikc_list[3].constraint_weight =  hrp::dvector6::Constant(1e-4);
//        ikc_list[4].constraint_weight =  hrp::dvector6::Constant(1e-4);
    }

    if(loop%20==0){
        if(sccp->collision_info_list.size()>0){
            std::cout<<"pair:"<<std::endl;
            for(int i=0;i<sccp->collision_info_list.size();i++){
                std::cout<<fik->m_robot->joint(sccp->collision_info_list[i].id0)->name<<" "<<fik->m_robot->joint(sccp->collision_info_list[i].id1)->name<<endl;
            }
        }
    }

    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT0")->jointId) = 1e3;//JAXON
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT1")->jointId) = 1e3;
//    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 10;
    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 0;//実機修理中

    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->m_robot->link("CHEST_JOINT0")->llimit = deg2rad(-8);
    if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->m_robot->link("CHEST_JOINT0")->ulimit = deg2rad(8);
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->m_robot->link("CHEST_JOINT1")->llimit = deg2rad(1);
    if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->m_robot->link("CHEST_JOINT1")->ulimit = deg2rad(32);

    if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->llimit = deg2rad(-20);
    if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->ulimit = deg2rad(20);
    if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->llimit = deg2rad(-15);
    if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->ulimit = deg2rad(35);

    if( fik->m_robot->link("RARM_JOINT6") != NULL) fik->m_robot->link("RARM_JOINT6")->llimit = deg2rad(-59);
    if( fik->m_robot->link("RARM_JOINT6") != NULL) fik->m_robot->link("RARM_JOINT6")->ulimit = deg2rad(59);
    if( fik->m_robot->link("RARM_JOINT7") != NULL) fik->m_robot->link("RARM_JOINT7")->llimit = deg2rad(-61);
    if( fik->m_robot->link("RARM_JOINT7") != NULL) fik->m_robot->link("RARM_JOINT7")->ulimit = deg2rad(58);

    if( fik->m_robot->link("LARM_JOINT6") != NULL) fik->m_robot->link("LARM_JOINT6")->llimit = deg2rad(-59);
    if( fik->m_robot->link("LARM_JOINT6") != NULL) fik->m_robot->link("LARM_JOINT6")->ulimit = deg2rad(59);
    if( fik->m_robot->link("LARM_JOINT7") != NULL) fik->m_robot->link("LARM_JOINT7")->llimit = deg2rad(-61);
    if( fik->m_robot->link("LARM_JOINT7") != NULL) fik->m_robot->link("LARM_JOINT7")->ulimit = deg2rad(58);

    if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->ulimit = deg2rad(-45);//脇内側の干渉回避
    if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->llimit = deg2rad(45);
    if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->llimit = deg2rad(-89);//肩グルン防止
    if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->ulimit = deg2rad(89);
    if( fik->m_robot->link("RARM_JOINT4") != NULL) fik->m_robot->link("RARM_JOINT4")->ulimit = deg2rad(1);//肘逆折れ
    if( fik->m_robot->link("LARM_JOINT4") != NULL) fik->m_robot->link("LARM_JOINT4")->ulimit = deg2rad(1);
    if( fik->m_robot->link("RLEG_JOINT3") != NULL) fik->m_robot->link("RLEG_JOINT3")->llimit = deg2rad(40);//膝伸びきり防止のため
    if( fik->m_robot->link("LLEG_JOINT3") != NULL) fik->m_robot->link("LLEG_JOINT3")->llimit = deg2rad(40);

    if( fik->m_robot->link("CHEST_Y") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_Y")->jointId) = 10;//K
    if( fik->m_robot->link("CHEST_P") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_P")->jointId) = 10;
    if( fik->m_robot->link("R_KNEE_P") != NULL) fik->m_robot->link("R_KNEE_P")->llimit = deg2rad(30);//膝伸びきり防止
    if( fik->m_robot->link("L_KNEE_P") != NULL) fik->m_robot->link("L_KNEE_P")->llimit = deg2rad(30);
    // if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->ulimit = deg2rad(40);
    // if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->ulimit = deg2rad(40);
    // if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-40);
    // if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(20);
    // if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(20);
    if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-80);
    if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-80);
    if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(45);
    if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(45);
    if( fik->m_robot->link("CHEST_Y") != NULL) fik->m_robot->link("CHEST_Y")->llimit = deg2rad(-20);
    if( fik->m_robot->link("CHEST_Y") != NULL) fik->m_robot->link("CHEST_Y")->ulimit = deg2rad(20);
    if( fik->m_robot->link("CHEST_P") != NULL) fik->m_robot->link("CHEST_P")->llimit = deg2rad(0);
    if( fik->m_robot->link("CHEST_P") != NULL) fik->m_robot->link("CHEST_P")->ulimit = deg2rad(60);
    if( fik->m_robot->link("HEAD_Y") != NULL) fik->m_robot->link("HEAD_Y")->llimit = deg2rad(-5);
    if( fik->m_robot->link("HEAD_Y") != NULL) fik->m_robot->link("HEAD_Y")->ulimit = deg2rad(5);
    if( fik->m_robot->link("HEAD_P") != NULL) fik->m_robot->link("HEAD_P")->llimit = deg2rad(0);
    if( fik->m_robot->link("HEAD_P") != NULL) fik->m_robot->link("HEAD_P")->ulimit = deg2rad(60);
    for(int i=0;i<fik->m_robot->numJoints();i++){
        LIMIT_MINMAX(fik->m_robot->joint(i)->q, fik->m_robot->joint(i)->llimit, fik->m_robot->joint(i)->ulimit);
    }

    fik->q_ref.head(m_qRef.data.length()) = hrp::to_dvector(m_qRef.data);//あえてseqからのbaselink poseは信用しない

    for(int i=0; i<fik->m_robot->numJoints(); i++){
        if(!has(wbms->wp.use_joints, fik->m_robot->joint(i)->name)){
            fik->dq_weight_all(i) = 0;
            fik->m_robot->joint(i)->q = m_qRef.data[i];
        }
    }


    if(fik->m_robot->name().find("JAXON") != std::string::npos){
        for(int i=0; i<fik->m_robot->numJoints(); i++){
            if(fik->m_robot->joint(i)->name.find("ARM") != std::string::npos){
                fik->q_ref_constraint_weight(i) = 1e-3;//腕だけ
            }
        }
    }

    const int IK_MAX_LOOP = 1;
    int loop_result = fik->solveFullbodyIKLoop(ikc_list, IK_MAX_LOOP);
}

void PrimitiveMotionLevelController::smoothingJointAngles(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe){
    double goal_time = 0.0;
    const double min_goal_time_offset = 0.3;

    static hrp::dvector ans_state_vel = hrp::dvector::Zero(fik->numStates());

    const hrp::dvector estimated_times_from_vel_limit = (hrp::getRobotStateVec(_robot) - hrp::getRobotStateVec(_robot_safe)).array().abs() / avg_q_vel.array();
    const hrp::dvector estimated_times_from_acc_limit = ans_state_vel.array().abs() / avg_q_acc.array();
    const double longest_estimated_time_from_vel_limit = estimated_times_from_vel_limit.maxCoeff();
    const double longest_estimated_time_from_acc_limit = estimated_times_from_acc_limit.maxCoeff();
    goal_time = hrp::Vector3(longest_estimated_time_from_vel_limit, longest_estimated_time_from_acc_limit, min_goal_time_offset).maxCoeff();

    q_ip->setGoal(hrp::getRobotStateVec(_robot).data(), goal_time, true);
    double tmp[fik->numStates()], tmpv[fik->numStates()];
    if (!q_ip->isEmpty() ){  q_ip->get(tmp, tmpv, true);}
    hrp::dvector ans_state = Eigen::Map<hrp::dvector>(tmp, fik->numStates());
    ans_state_vel = Eigen::Map<hrp::dvector>(tmpv, fik->numStates());

    hrp::setRobotStateVec(_robot_safe, ans_state);
    for(int i=0; i<_robot_safe->numJoints(); i++){
        LIMIT_MINMAX(_robot_safe->joint(i)->q, _robot_safe->joint(i)->llimit, _robot_safe->joint(i)->ulimit);
    }

    _robot_safe->calcForwardKinematics();
}


bool PrimitiveMotionLevelController::startControl(){
  if(this->mode_.now() == MODE_IDLE){
    RTC_INFO_STREAM("startControl");
    this->mode_.setNextMode(MODE_SYNC_TO_WBMS);
    return true;
  }else{
    RTC_WARN_STREAM("Invalid context to startControl");
    return false;
  }
}


bool PrimitiveMotionLevelController::stopControl(){
    if(this->mode_.now() == MODE_WBMS ){
        RTC_INFO_STREAM("stopControl");
        this->mode_.setNextMode(MODE_SYNC_TO_IDLE);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to stopControl");
        return false;
    }
}

bool PrimitiveMotionLevelController::setParams(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param){
    RTC_INFO_STREAM("setPrimitiveMotionLevelControllerParam");
    return true;
}


bool PrimitiveMotionLevelController::getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param){
    RTC_INFO_STREAM("getPrimitiveMotionLevelControllerParam");
    return true;
}

RTC::ReturnCode_t PrimitiveMotionLevelController::onActivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onActivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t PrimitiveMotionLevelController::onDeactivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onDeactivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t PrimitiveMotionLevelController::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void PrimitiveMotionLevelControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveMotionLevelController_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveMotionLevelController>, RTC::Delete<PrimitiveMotionLevelController>);
    }
};
