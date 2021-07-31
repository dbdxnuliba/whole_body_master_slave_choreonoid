#include "WholeBodyMasterSlaveChoreonoid.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* WholeBodyMasterSlaveChoreonoid_spec[] = {
  "implementation_id", "WholeBodyMasterSlaveChoreonoid",
  "type_name",         "WholeBodyMasterSlaveChoreonoid",
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

inline std::vector<std::string> to_string_vector (const OpenHRP::WholeBodyMasterSlaveChoreonoidService::StrSequence& in) {
  std::vector<std::string> ret(in.length()); for(int i=0; i<in.length(); i++){ ret[i] = in[i]; } return ret;
}
inline OpenHRP::WholeBodyMasterSlaveChoreonoidService::StrSequence to_StrSequence  (const std::vector<std::string>& in){
  OpenHRP::WholeBodyMasterSlaveChoreonoidService::StrSequence ret; ret.length(in.size()); for(int i=0; i<in.size(); i++){ ret[i] = in[i].c_str(); } return ret;
}

WholeBodyMasterSlaveChoreonoid::Ports::Ports() :
  m_qRefIn_("qRef", m_qRef_),// from sh
  m_basePosRefIn_("basePosRefIn", m_basePosRef_),// from sh
  m_baseRpyRefIn_("baseRpyRefIn", m_baseRpyRef_),// from sh
  m_eeStateRefIn_("eeStateRefIn", m_eeStateRefIn_),

  m_qActIn_("qAct", m_qAct_),
  m_imuActIn_("imuAct", m_imuAct_),

  m_qComOut_("qCom", m_qCom_),
  m_basePosComOut_("basePosComOut", m_basePosCom_),
  m_baseRpyComOut_("baseRpyComOut", m_baseRpyCom_),

  m_eeStateActOut_("eeStateActOut", m_eeStateActOut_),

  m_delayCheckPacketInboundIn("delay_check_packet_inbound", m_delayCheckPacket),
  m_delayCheckPacketOutboundOut("delay_check_packet_outbound", m_delayCheckPacket),

  m_WholeBodyMasterSlaveChoreonoidServicePort_("WholeBodyMasterSlaveChoreonoidService") {
}


WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoid(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  m_debugLevel_(0)
{
  this->ports_.m_service0_.wholebodymasterslavechoreonoid(this);
}

RTC::ReturnCode_t WholeBodyMasterSlaveChoreonoid::onInitialize(){
  bindParameter("debugLevel", m_debugLevel_, "0");

  addInPort("qRef", this->ports_.m_qRefIn_);// from sh
  addInPort("basePosRef", this->ports_.m_basePosRefIn_);
  addInPort("baseRpyRef", this->ports_.m_baseRpyRefIn_);
  addInPort("eeStateRef", this->ports_.m_eeStateRefIn_);
  addInPort("qAct", this->ports_.m_qActIn_);
  addInPort("imuAct", this->ports_.m_imuActIn_);
  addOutPort("qCom", this->ports_.m_qComOut_);
  addOutPort("basePosCom", this->ports_.m_basePosComOut_);
  addOutPort("baseRpyCom", this->ports_.m_baseRpyComOut_);
  addOutPort("eeStateAct". this->ports_.m_eeStateActOut_);
  addInPort("delay_check_packet_inbound", this->ports_.m_delayCheckPacketInboundIn_);
  addOutPort("delay_check_packet_outbound", this->ports_.m_delayCheckPacketOutboundOut_);
  this->ports_.m_WholeBodyMasterSlaveChoreonoidServicePort_.registerProvider("service0", "WholeBodyMasterSlaveChoreonoidService", this->ports_.m_service0_);
  addPort(this->ports_.m_WholeBodyMasterSlaveChoreonoidServicePort_);

  RTC::Properties& prop = getProperties();
  coil::stringTo(this->m_dt_, prop["dt"].c_str());

  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load(prop["model"]);
  if(!robot){
    RTC_WARN_STREAM("failed to load model[" << prop["model"] << "]");
    return RTC::RTC_ERROR;
  }
  this->m_robot_ref_ = robot;
  this->m_robot_act_ = robot->clone();
  this->m_robot_com_ = robot->clone();

  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot->devices());
  for(size_t i=0;i<forceSensors.size();i++){
    this->ports_.m_fsensorActIn_[forceSensors[i]->name()] = std::make_shared<RTC::InPort <RTC::TimedDoubleSeq> > >((forceSensors[i]+"In").c_str(), m_fsensorAct_[forceSensors[i]->name()]);
  }

  RTC_INFO_STREAM("setup fullbody ik finished");

  wbms = boost::shared_ptr<WBMSCore>(new WBMSCore(m_dt));
  for(size_t i=0;i<this->robot_for_ik_->numJoints();i++){
    wbms->wp.use_joints.push_back(this->robot_for_ik_->joint(i)->name());
  }

  //TODO
  //sccp = boost::shared_ptr<CapsuleCollisionChecker>(new CapsuleCollisionChecker(fik->m_robot));

  this->outputRatioInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,cpp_filters::TwoPointInterpolator<double>::HOFFARBIB);

  this->outputSmoothingInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::VectorX> >(cnoid::VectorX::Zero(robot->numJoints()),cpp_filters::TwoPointInterpolator<double>::CUBICSPLINE); // or HOFFARBIB, QUINTICSPLINE
  this->avg_q_vel_ = cnoid::VectorX::Constant(robot->numJoints(),4.0);// 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい
  this->avg_q_acc_ = hrp::dvector::Constant(robot->numJoints(), 16.0); // all joint max avarage acc = 16.0 rad/s^2

  this->staticBalancingCOMOffsetFilter_.setParameterAsBiquad(1, 0.707106781, 1/m_dt, cnoid::Vector3::Zero());// 1/sqrt(2) = Butterworth

  loop = 0;
  return RTC::RTC_OK;
}

void readPorts(WholeBodyMasterSlaveChoreonoid::Ports& port) {
  if(port.m_qRefIn_.isNew()) port.m_qRefIn_.read();
  if(port.m_basePosRefIn_.isNew()) port.m_basePosRefIn_.read();
  if(port.m_baseRpyRefIn_.isNew()) port.m_baseRpyRefIn_.read();
  if(port.m_eeStateRefIn_.isNew()) port.m_eeStateRefIn_.read();
  if(port.m_qActIn_.isNew()) port.m_qActIn_.read();
  if(port.m_imuActIn_.isNew()) port.m_imuActIn_.read();
  for(std::map<std::string, std::shared_ptr<RTC::InPort <RTC::TimedDoubleSeq> > >::Iterator it = port.m_fsensorActIn_.begin(); it != port.m_fsensorActIn_.end(); it++) {
    if(it->second->isNew()) it->second->read();
  }
  if(port.m_delayCheckPacketInboundIn_.isNew()){
    port.m_delayCheckPacketInboundIn_.read();
    port.m_delayCheckPacketOutboundOut_.write();
  }
}

// calc reference state (without ee. q, basepos and baserpy only)
void calcReferenceRobot(cnoid::BodyPtr& robot, const WholeBodyMasterSlaveChoreonoid::Ports& port) {
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
void calcActualRobot(cnoid::BodyPtr& robot, const WholeBodyMasterSlaveChoreonoid::Ports& port) {
  if(port.m_qAct_.length() == robot->numJoints()){
    for ( int i = 0; i < robot->numJoints(); i++ ){
      robot->joint(i)->q() = port.m_qAct_.data[i];
    }
    robot->calcForwardKinematics();

    cnoid::AccelerationSensorPtr imu = robot->findDevice<cnoid::AccelerationSensor>("gyrometer");
    cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
    cnoid::Matrix3 actR = cnoid::rotFromRpy(port.m_imuAct_.data.r, port.m_imuAct_.data.p, port.m_imuAct_.data.y);
    robot->rootLink()->R = actR * (imuR.transpose() * robot->rootLink()->R());
    robot->calcForwardKinematics();
  }
}

// 支持脚以外のエンドエフェクタの受けている力に釣り合う重心位置のオフセットを求める
cnoid::Vector3 calcStaticBalancingCOMOffset(){
  // for(auto ee : {"larm","rarm"}){
  //   const hrp::Vector3 use_f = hrp::Vector3::UnitZ() * m_slaveEEWrenches[ee].data[Z];
  //   hrp::Vector3 use_f_filtered = ee_f_filter[ee].passFilter(use_f);
  //   const hrp::Vector3 ee_pos_from_com = ee_ikc_map[ee].getCurrentTargetPos(m_robot_vsafe) - wbms->rp_ref_out.tgt[com].abs.p;
  //   static_balancing_com_offset.head(XY) += - ee_pos_from_com.head(XY) * use_f_filtered(Z) / (-m_robot_vsafe->totalMass() * G);
  // }

  return cnoid::Vector3::Zero(); //TODO
}

void processTransition(WholeBodyMasterSlaveChoreonoid::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> >& outputRatioInterpolator, double dt){
  double tmp;
  switch(mode.now()){
  case WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_SYNC_TO_WBMS:
    if(mode.pre() == WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_IDLE){
      outputRatioInterpolator->setGoal(1.0, 3.0);
    }
    if (!outputRatioInterpolator->isEmpty() ){
      outputRatioInterpolator->get(&tmp, dt);
    }else{
      mode.setNextMode(WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_WBMS);
    }
    break;
  case WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_WBMS ||
       mode.pre() == WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_PAUSE){
      outputRatioInterpolator->setGoal(0.0, 3.0, true);
    }
    if (outputRatioInterpolator->isEmpty()) {
      outputRatioInterpolator->get(&tmp, dt);
    }else{
      mode.setNextMode(WholeBodyMasterSlaveChoreonoid::ControlMode::MODE_IDLE);
    }
    break;
  }
  mode.update();
}

void calcOutputPorts(WholeBodyMasterSlaveChoreonoid::Ports& port, double output_ratio, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com) {
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

RTC::ReturnCode_t WholeBodyMasterSlaveChoreonoid::onExecute(RTC::UniqueId ec_id){

  time_report_str.clear();
  clock_gettime(CLOCK_REALTIME, &startT);

  // read ports
  readPorts(this->port_);

  // calc reference state from inport (without ee. q, basepos and baserpy only)
  calcReferenceRobot(this->m_robot_ref_, this->port_);

  // calc actial state from inport
  calcActualRobot(this->m_robot_act_, this->port_);

  // calc static balancing com offset
  cnoid::Vector3 staticBalancingCOMOffset = calcStaticBalancingCOMOffset();
  staticBalancingCOMOffset = this->staticBalancingCOMOffsetFilter_.passFilter(staticBalancingCOMOffset);

  // wbms->act_rs.act_foot_wrench[0] = hrp::to_dvector(m_localEEWrenches["rleg"].data);
  // wbms->act_rs.act_foot_wrench[1] = hrp::to_dvector(m_localEEWrenches["lleg"].data);
  // wbms->act_rs.act_foot_pose[0] = ee_ikc_map["rleg"].getCurrentTargetPose(m_robot_vsafe);
  // wbms->act_rs.act_foot_pose[1] = ee_ikc_map["lleg"].getCurrentTargetPose(m_robot_vsafe);

  //   for(auto ee : ee_names){
  //       hrp::ForceSensor* sensor = m_robot_act->sensor<hrp::ForceSensor>(to_sname[ee]);
  //       hrp::dvector6 w_ee_wld = hrp::dvector6::Zero();
  //       if(sensor){
  //           hrp::Matrix33 sensorR_wld = sensor->link->R * sensor->localR;
  //           hrp::Matrix33 sensorR_from_base = m_robot_act->rootLink()->R.transpose() * sensorR_wld;
  //           const hrp::Vector3 f_sensor_wld = sensorR_from_base * hrp::to_dvector(m_localEEWrenches[ee].data).head(3);
  //           const hrp::Vector3 t_sensor_wld = sensorR_from_base * hrp::to_dvector(m_localEEWrenches[ee].data).tail(3);
  //           const hrp::Vector3 sensor_to_ee_vec_wld = ee_ikc_map[ee].getCurrentTargetPos(m_robot_act) - sensor->link->p;
  //           w_ee_wld << f_sensor_wld, t_sensor_wld - sensor_to_ee_vec_wld.cross(f_sensor_wld);
  //       }
  //   }

  addTimeReport("InPort");

  // mode遷移を実行
  processTransition(this->mode_, this->outputRatioInterpolator_, this->m_dt_);

  if(this->mode_.isRunning()) {
    if(this->mode_.mode.isInitialize()){
      preProcessForWholeBodyMasterSlave();
    }
    wbms->update();//////HumanSynchronizerの主要処理
    if(DEBUGP)RTC_INFO_STREAM(wbms->rp_ref_out);
    addTimeReport("MainFunc");

    solveFullbodyIK(wbms->rp_ref_out);
    addTimeReport("IK");

    smoothingJointAngles(fik->m_robot, m_robot_vsafe);
  } else {
    // robot_refがそのままrobot_comになる
    this->m_robot_com_->rootLink()->T() = this->m_robot_ref_->rootLink()->T();
    for(size_t i=0;i<this->m_robot_ref_->numJoints();i++){
      this->m_robot_com_->joint(i)->q() = this->m_robot_ref_->joint(i)->q();
    }
    this->m_robot_com_->calcForwardKinematics();
  }
  wbms->baselinkpose.p = fik->m_robot->rootLink()->p;
  wbms->baselinkpose.R = fik->m_robot->rootLink()->R;

  // write outport
  double output_ratio; this->outputRatioInterpolator_->get(output_ratio);
  calcOutputPorts(this->port_, output_ratio, this->m_robot_ref_, this->m_robot_com_);
  addTimeReport("OutPort");

  if(DEBUGP)RTC_INFO_STREAM(time_report_str);
  if(DEBUGP)RTC_INFO_STREAM(wbms->ws);

  loop++;
  return RTC::RTC_OK;
}

void WholeBodyMasterSlaveChoreonoid::preProcessForWholeBodyMasterSlave(){
    fik->m_robot->rootLink()->p = hrp::to_Vector3(m_basePos.data);
    hrp::setQAll(fik->m_robot, hrp::to_dvector(m_qRef.data));
    fik->m_robot->calcForwardKinematics();
    double current_foot_height_from_world = wbms->legged ? ::min(ee_ikc_map["rleg"].getCurrentTargetPos(fik->m_robot)(Z), ee_ikc_map["lleg"].getCurrentTargetPos(fik->m_robot)(Z)) : 0;
    RTC_INFO_STREAM("current_foot_height_from_world = "<<current_foot_height_from_world<<" will be modified to 0");
    for(auto b : {fik->m_robot, m_robot_vsafe}){//初期姿勢でBodyをFK
        hrp::setRobotStateVec(b, hrp::to_dvector(m_qRef.data), hrp::to_Vector3(m_basePos.data) - hrp::Vector3::UnitZ() * current_foot_height_from_world, hrp::rotFromRpy(hrp::to_Vector3(m_baseRpy.data)));
        b->calcForwardKinematics();
    }
    fik->q_ref = hrp::getRobotStateVec(fik->m_robot);
    q_ip->set(fik->q_ref.data());
    wbms->initializeRequest(fik->m_robot, ee_ikc_map);
}


void WholeBodyMasterSlaveChoreonoid::solveFullbodyIK(HumanPose& ref){
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

void WholeBodyMasterSlaveChoreonoid::smoothingJointAngles(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe){
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


bool WholeBodyMasterSlaveChoreonoid::startWholeBodyMasterSlave(){
  if(this->mode_.now() == MODE_IDLE){
    RTC_INFO_STREAM("startWholeBodyMasterSlave");
    this->mode_.setNextMode(MODE_SYNC_TO_WBMS);
    return true;
  }else{
    RTC_WARN_STREAM("Invalid context to startWholeBodyMasterSlave");
    return false;
  }
}


bool WholeBodyMasterSlaveChoreonoid::stopWholeBodyMasterSlave(){
    if(this->mode_.now() == MODE_WBMS ){
        RTC_INFO_STREAM("stopWholeBodyMasterSlave");
        this->mode_.setNextMode(MODE_SYNC_TO_IDLE);
        return true;
    }else{
        RTC_WARN_STREAM("Invalid context to stopWholeBodyMasterSlave");
        return false;
    }
}

OpenHRP::WholeBodyMasterSlaveChoreonoidService::DblSequence2 to_DblSequence2(const hrp::Vector2& in){
  OpenHRP::WholeBodyMasterSlaveChoreonoidService::DblSequence2 ret; ret.length(in.size()); hrp::Vector2::Map(ret.get_buffer()) = in; return ret; }
OpenHRP::WholeBodyMasterSlaveChoreonoidService::DblSequence3 to_DblSequence3(const hrp::Vector3& in){
  OpenHRP::WholeBodyMasterSlaveChoreonoidService::DblSequence3 ret; ret.length(in.size()); hrp::Vector3::Map(ret.get_buffer()) = in; return ret; }
OpenHRP::WholeBodyMasterSlaveChoreonoidService::DblSequence4 to_DblSequence4(const hrp::Vector4& in){
  OpenHRP::WholeBodyMasterSlaveChoreonoidService::DblSequence4 ret; ret.length(in.size()); hrp::Vector4::Map(ret.get_buffer()) = in; return ret; }


bool WholeBodyMasterSlaveChoreonoid::setParams(const OpenHRP::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param){
    RTC_INFO_STREAM("setWholeBodyMasterSlaveChoreonoidParam");
    wbms->wp.auto_com_mode                      = i_param.auto_com_mode;
    wbms->wp.auto_floor_h_mode                  = i_param.auto_floor_h_mode;
    wbms->wp.auto_foot_landing_by_act_cp        = i_param.auto_foot_landing_by_act_cp;
    wbms->wp.auto_foot_landing_by_act_zmp       = i_param.auto_foot_landing_by_act_zmp;
    wbms->wp.additional_double_support_time     = i_param.additional_double_support_time;
    wbms->wp.auto_com_foot_move_detect_height   = i_param.auto_com_foot_move_detect_height;
    wbms->wp.auto_floor_h_detect_fz             = i_param.auto_floor_h_detect_fz;
    wbms->wp.auto_floor_h_reset_fz              = i_param.auto_floor_h_reset_fz;
    wbms->wp.base_to_hand_min_distance          = i_param.base_to_hand_min_distance;
    wbms->wp.capture_point_extend_ratio         = i_param.capture_point_extend_ratio;
    wbms->wp.com_filter_cutoff_hz               = i_param.com_filter_cutoff_hz;
    wbms->wp.foot_collision_avoidance_distance  = i_param.foot_collision_avoidance_distance;
    wbms->wp.foot_landing_vel                   = i_param.foot_landing_vel;
    wbms->wp.force_double_support_com_h         = i_param.force_double_support_com_h;
    wbms->wp.human_to_robot_ratio               = i_param.human_to_robot_ratio;
    wbms->wp.max_double_support_width           = i_param.max_double_support_width;
    wbms->wp.upper_body_rmc_ratio               = i_param.upper_body_rmc_ratio;
    wbms->wp.single_foot_zmp_safety_distance    = i_param.single_foot_zmp_safety_distance;
    wbms->wp.swing_foot_height_offset           = i_param.swing_foot_height_offset;
    wbms->wp.com_offset                         = hrp::Vector3::Map(i_param.com_offset.get_buffer());
    wbms->wp.actual_foot_vert_fbio              = hrp::Vector4::Map(i_param.actual_foot_vert_fbio.get_buffer());
    wbms->wp.safety_foot_vert_fbio              = hrp::Vector4::Map(i_param.safety_foot_vert_fbio.get_buffer());
    wbms->wp.use_joints                         = hrp::to_string_vector(i_param.use_joints);
    wbms->wp.use_targets                        = hrp::to_string_vector(i_param.use_targets);
    wbms->wp.CheckSafeLimit();
    return true;
}


bool WholeBodyMasterSlaveChoreonoid::getParams(OpenHRP::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param){
    RTC_INFO_STREAM("getWholeBodyMasterSlaveChoreonoidParam");
    i_param.auto_com_mode                       = wbms->wp.auto_com_mode;
    i_param.auto_floor_h_mode                   = wbms->wp.auto_floor_h_mode;
    i_param.auto_foot_landing_by_act_cp         = wbms->wp.auto_foot_landing_by_act_cp;
    i_param.auto_foot_landing_by_act_zmp        = wbms->wp.auto_foot_landing_by_act_zmp;
    i_param.additional_double_support_time      = wbms->wp.additional_double_support_time;
    i_param.auto_com_foot_move_detect_height    = wbms->wp.auto_com_foot_move_detect_height;
    i_param.auto_floor_h_detect_fz              = wbms->wp.auto_floor_h_detect_fz;
    i_param.auto_floor_h_reset_fz               = wbms->wp.auto_floor_h_reset_fz;
    i_param.base_to_hand_min_distance           = wbms->wp.base_to_hand_min_distance;
    i_param.capture_point_extend_ratio          = wbms->wp.capture_point_extend_ratio;
    i_param.com_filter_cutoff_hz                = wbms->wp.com_filter_cutoff_hz;
    i_param.foot_collision_avoidance_distance   = wbms->wp.foot_collision_avoidance_distance;
    i_param.foot_landing_vel                    = wbms->wp.foot_landing_vel;
    i_param.force_double_support_com_h          = wbms->wp.force_double_support_com_h;
    i_param.human_to_robot_ratio                = wbms->wp.human_to_robot_ratio;
    i_param.max_double_support_width            = wbms->wp.max_double_support_width;
    i_param.upper_body_rmc_ratio                = wbms->wp.upper_body_rmc_ratio;
    i_param.single_foot_zmp_safety_distance     = wbms->wp.single_foot_zmp_safety_distance;
    i_param.swing_foot_height_offset            = wbms->wp.swing_foot_height_offset;
    i_param.com_offset                          = hrp::to_DblSequence3(wbms->wp.com_offset);
    i_param.actual_foot_vert_fbio               = hrp::to_DblSequence4(wbms->wp.actual_foot_vert_fbio);
    i_param.safety_foot_vert_fbio               = hrp::to_DblSequence4(wbms->wp.safety_foot_vert_fbio);
    i_param.use_joints                          = hrp::to_StrSequence(wbms->wp.use_joints);
    i_param.use_targets                         = hrp::to_StrSequence(wbms->wp.use_targets);
    return true;
}

RTC::ReturnCode_t WholeBodyMasterSlaveChoreonoid::onActivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onActivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlaveChoreonoid::onDeactivated(RTC::UniqueId ec_id){ RTC_INFO_STREAM("onDeactivated(" << ec_id << ")"); return RTC::RTC_OK; }
RTC::ReturnCode_t WholeBodyMasterSlaveChoreonoid::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void WholeBodyMasterSlaveChoreonoidInit(RTC::Manager* manager) {
        RTC::Properties profile(WholeBodyMasterSlaveChoreonoid_spec);
        manager->registerFactory(profile, RTC::Create<WholeBodyMasterSlaveChoreonoid>, RTC::Delete<WholeBodyMasterSlaveChoreonoid>);
    }
};
