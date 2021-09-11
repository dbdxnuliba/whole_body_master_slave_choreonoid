#include "PositionController.h"
#include <cnoid/EigenUtil>

namespace PrimitiveMotionLevel {
  PositionController::PositionTask::PositionTask(const std::string& name) :
    name_(name),
    offset_(cnoid::Position::Identity()),
    dOffsetPrev_(cnoid::Vector6::Zero())
  {
  }

  void PositionController::PositionTask::calcImpedanceControl(double dt){
    cnoid::Matrix3 eeR = this->offset_.linear() * this->primitiveCommand_->targetPose().linear();

    cnoid::Vector6 offsetPrev; //world系
    offsetPrev.head<3>() = this->offset_.translation();
    offsetPrev.tail<3>() = cnoid::omegaFromRot(this->offset_.linear());

    cnoid::Vector6 offsetPrevLocal; //local系
    offsetPrevLocal.head<3>() = eeR.transpose() * offsetPrev.head<3>();
    offsetPrevLocal.tail<3>() = eeR.transpose() * offsetPrev.tail<3>();

    cnoid::Vector6 dOffsetPrevLocal; //local系
    offsetPrevLocal.head<3>() = eeR.transpose() * this->dOffsetPrev_.head<3>();
    offsetPrevLocal.tail<3>() = eeR.transpose() * this->dOffsetPrev_.tail<3>();

    cnoid::Vector6 dOffsetLocal; //local系
    for(size_t i=0;i<6;i++){
      if(this->primitiveCommand_->M()[i] == 0.0 && this->primitiveCommand_->D()[i] == 0.0 && this->primitiveCommand_->K()[i]==0.0){
        dOffsetLocal[i] = 0.0;
        continue;
      }

      dOffsetLocal[i] =
        (this->primitiveCommand_->actWrench()[i] * this->primitiveCommand_->wrenchGain()[i] * dt * dt
         - this->primitiveCommand_->K()[i] * offsetPrevLocal[i] * dt * dt
         + this->primitiveCommand_->M()[i] * dOffsetPrevLocal[i]*dt)
        / (this->primitiveCommand_->M()[i] + this->primitiveCommand_->D()[i] * dt + this->primitiveCommand_->K()[i] * dt * dt);
    }

    cnoid::Vector6 dOffset; //world系
    dOffset.head<3>() = eeR * dOffsetLocal.head<3>();
    dOffset.tail<3>() = eeR * dOffsetLocal.tail<3>();

    this->offset_.translation() += dOffset.head<3>();
    this->offset_.linear() = (cnoid::AngleAxisd(dOffset.tail<3>().norm(), dOffset.tail<3>().norm()>0 ? dOffset.tail<3>().normalized() : cnoid::Vector3::UnitX()) * this->offset_.linear()).eval();
    this->dOffsetPrev_ = dOffset / dt;
  }

  void PositionController::reset() {
    this->positionTaskMap_.clear();
  }

  namespace PositionControllerImpl {
    void getPrimitiveCommand(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<PositionController::PositionTask> >& positionTaskMap) {
      // 消滅したEndEffectorを削除
      for(std::map<std::string, std::shared_ptr<PositionController::PositionTask> >::iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); ) {
        if (primitiveCommandMap.find(it->first) == primitiveCommandMap.end()) it = positionTaskMap.erase(it);
        else ++it;
      }
      // 増加したEndEffectorの反映
      for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
        if(positionTaskMap.find(it->first)==positionTaskMap.end()){
          positionTaskMap[it->first] = std::make_shared<PositionController::PositionTask>(it->first);
        }
      }
      // 各指令値の反映
      for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++){
        positionTaskMap[it->first]->updateFromPrimitiveCommand(it->second);
      }
    }

    void modifyTargetWrenchForCOMControl(std::shared_ptr<PositionController::PositionTask>& comTask, std::vector<std::shared_ptr<PositionController::PositionTask> >& supportEEF, const cnoid::BodyPtr& robot_act, const bool& frameWarped) {
      if(!comTask) return;
      if(supportEEF.size() ==0) return;

      // TODO
    }

    void calcCOMPostureControl(std::shared_ptr<PositionController::PositionTask>& comTask, const cnoid::BodyPtr& robot_act, const bool& frameWarped) {
      if(!comTask) return;
    }

    void getPrimitiveMotionLevelIKConstraints(const std::map<std::string, std::shared_ptr<PositionController::PositionTask> >& PositionTaskMap, std::vector<std::shared_ptr<IK::IKConstraint> >& primitiveMotionLevelIKConstraints) {
    }

    void getCommandLevelIKConstraints(const cnoid::BodyPtr& robot_ref, std::vector<std::shared_ptr<IK::IKConstraint> >& commandLevelIKConstraints) {
    }

    void solveFullbodyIK(cnoid::BodyPtr& robot_com, std::vector<std::shared_ptr<IK::IKConstraint> >& primitiveMotionLevelIKConstraints, std::vector<std::shared_ptr<IK::IKConstraint> >& commandLevelIKConstraints) {
    }
  }

  void PositionController::control(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap, // primitive motion level target
                                   const cnoid::BodyPtr& robot_ref, // command level target
                                   const cnoid::BodyPtr& robot_act, //actual
                                   cnoid::BodyPtr& robot_com, //output
                                   const bool& frameWarped, // if true, not calculate velocity
                                   double dt
                                   ) {
    // 目標値を反映
    PositionControllerImpl::getPrimitiveCommand(primitiveCommandMap, this->positionTaskMap_);

    std::shared_ptr<PositionController::PositionTask> comTask = nullptr;
    std::vector<std::shared_ptr<PositionController::PositionTask> > noCOMTask;
    for(std::map<std::string, std::shared_ptr<PositionController::PositionTask> >::iterator it = this->positionTaskMap_.begin(); it != this->positionTaskMap_.end(); it++) {
      if(it->first=="com") comTask = it->second;
      else noCOMTask.push_back(it->second);
    }

    if(comTask) {
      // 重心のエラーに応じてsupportCOM=trueのエンドエフェクタの目標反力を修正
      std::vector<std::shared_ptr<PositionController::PositionTask> > supportEEF;
      for(size_t i=0;i<noCOMTask.size();i++) {
        if(noCOMTask[i]->primitiveCommand()->supportCOM()) supportEEF.push_back(noCOMTask[i]);
      }
      if(supportEEF.size() !=0) PositionControllerImpl::modifyTargetWrenchForCOMControl(comTask, supportEEF, robot_act, frameWarped);

      // 重心とbaseリンク傾きのIK目標値を計算
      PositionControllerImpl::calcCOMPostureControl(comTask, robot_act, frameWarped);
    }

    // 重心以外のIK目標値を計算
    for(size_t i=0;i<noCOMTask.size();i++){
      noCOMTask[i]->calcImpedanceControl(dt);
    }

    // primitive motion levelのIKConstraintを取得
    std::vector<std::shared_ptr<IK::IKConstraint> > primitiveMotionLevelIKConstraints;
    PositionControllerImpl::getPrimitiveMotionLevelIKConstraints(this->positionTaskMap_, primitiveMotionLevelIKConstraints);

    // command levelのIKConstraintを取得
    std::vector<std::shared_ptr<IK::IKConstraint> > commandLevelIKConstraints;
    PositionControllerImpl::getCommandLevelIKConstraints(robot_ref, commandLevelIKConstraints);

    // solve ik
    PositionControllerImpl::solveFullbodyIK(robot_com, primitiveMotionLevelIKConstraints, commandLevelIKConstraints);
  }

}


// void PrimitiveMotionLevelController::solveFullbodyIK(HumanPose& ref){
//     std::vector<IKConstraint> ikc_list;
//     if(wbms->legged){ // free baselink, lleg, rleg, larm, rarm setting
//         {
//             IKConstraint tmp;
//             tmp.target_link_name    = fik->m_robot->rootLink()->name;
//             tmp.localPos            = hrp::Vector3::Zero();
//             tmp.localR              = hrp::Matrix33::Identity();
//             tmp.targetPos           = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
//             tmp.targetRpy           = ref.stgt("com").abs.rpy();
//             tmp.constraint_weight   << 0, 0, 0, 0.1, 0.1, 0.1;
//             tmp.rot_precision       = deg2rad(3);
//             ikc_list.push_back(tmp);
//         }
//         for(auto leg : {"lleg","rleg"}){
//             if(has(wbms->wp.use_targets, leg)){
//                 IKConstraint tmp;
//                 tmp.target_link_name    = ee_ikc_map[leg].target_link_name;
//                 tmp.localPos            = ee_ikc_map[leg].localPos;
//                 tmp.localR              = ee_ikc_map[leg].localR;
//                 tmp.targetPos           = ref.stgt(leg).abs.p;
//                 tmp.targetRpy           = ref.stgt(leg).abs.rpy();
//                 tmp.constraint_weight   = wbms->rp_ref_out.tgt[rf].is_contact() ? hrp::dvector6::Constant(3) : hrp::dvector6::Constant(0.1);
//                 ikc_list.push_back(tmp);
//             }
//         }
//         for(auto arm : {"larm","rarm"}){
//             if(has(wbms->wp.use_targets, arm)){
//                 IKConstraint tmp;
//                 tmp.target_link_name    = ee_ikc_map[arm].target_link_name;
//                 tmp.localPos            = ee_ikc_map[arm].localPos;
//                 tmp.localR              = ee_ikc_map[arm].localR;
//                 tmp.targetPos           = ref.stgt(arm).abs.p;
//                 tmp.targetRpy           = ref.stgt(arm).abs.rpy();
//                 tmp.constraint_weight   = hrp::dvector6::Constant(0.1);
//                 tmp.pos_precision       = 3e-3;
//                 tmp.rot_precision       = deg2rad(3);
//                 ikc_list.push_back(tmp);
//             }
//         }
//         if(has(wbms->wp.use_targets, "com")){
//             IKConstraint tmp;
//             tmp.target_link_name    = "COM";
//             tmp.localPos            = hrp::Vector3::Zero();
//             tmp.localR              = hrp::Matrix33::Identity();
//             tmp.targetPos           = ref.stgt("com").abs.p + static_balancing_com_offset;// COM height will not be constraint
//             tmp.targetRpy           = hrp::Vector3::Zero();//reference angular momentum
//             tmp.constraint_weight   << 3,3,0.01,0,0,0;
//             ikc_list.push_back(tmp);
//         }
//     }else{ // fixed baselink, larm, rarm setting
//         {
//             IKConstraint tmp;
//             tmp.target_link_name    = fik->m_robot->rootLink()->name;
//             tmp.localPos            = hrp::Vector3::Zero();
//             tmp.localR              = hrp::Matrix33::Identity();
//             tmp.targetPos           = hrp::to_Vector3(m_basePos.data);// will be ignored by selection_vec
//             tmp.targetRpy           = hrp::Vector3::Zero();
//             tmp.constraint_weight   << hrp::dvector6::Constant(1);
//             tmp.rot_precision       = deg2rad(3);
//             ikc_list.push_back(tmp);
//         }
//         for(auto arm : {"larm","rarm"}){
//             if(has(wbms->wp.use_targets, arm)){
//                 IKConstraint tmp;
//                 tmp.target_link_name    = ee_ikc_map[arm].target_link_name;
//                 tmp.localPos            = ee_ikc_map[arm].localPos;
//                 tmp.localR              = ee_ikc_map[arm].localR;
//                 tmp.targetPos           = ref.stgt(arm).abs.p;
//                 tmp.targetRpy           = ref.stgt(arm).abs.rpy();
//                 tmp.constraint_weight   << 1, 1, 1, 0.1, 0.1, 0.1;
//                 tmp.pos_precision       = 3e-3;
//                 tmp.rot_precision       = deg2rad(3);
//                 ikc_list.push_back(tmp);
//             }
//         }
//     }
//     // common head setting
//     if(has(wbms->wp.use_targets, "head")){
//         if(fik->m_robot->link("HEAD_JOINT1") != NULL){
//             IKConstraint tmp;
//             tmp.target_link_name = "HEAD_JOINT1";
//             tmp.targetRpy = ref.stgt("head").abs.rpy();
//             tmp.constraint_weight << 0,0,0,0,0.1,0.1;
//             tmp.rot_precision = deg2rad(1);
//             ikc_list.push_back(tmp);
//         }
//         if(fik->m_robot->link("HEAD_P") != NULL){
//             IKConstraint tmp;
//             tmp.target_link_name = "HEAD_P";
//             tmp.targetRpy = ref.stgt("head").abs.rpy();
//             tmp.constraint_weight << 0,0,0,0,0.1,0.1;
//             tmp.rot_precision = deg2rad(1);
//             ikc_list.push_back(tmp);
//         }
//     }
//     if(wbms->rp_ref_out.tgt[rf].is_contact()){
//         sccp->avoid_priority.head(12).head(6).fill(4);
//     }else{
//         sccp->avoid_priority.head(12).head(6).fill(3);
//     }
//     if(wbms->rp_ref_out.tgt[lf].is_contact()){
//         sccp->avoid_priority.head(12).tail(6).fill(4);
//     }else{
//         sccp->avoid_priority.head(12).tail(6).fill(3);
//     }

// //    sccp->checkCollision();

//     for(int i=0;i<sccp->collision_info_list.size();i++){
//         IKConstraint tmp;
//         double val_w = (sccp->collision_info_list[i].dist_safe - sccp->collision_info_list[i].dist_cur)*1e2;
//         LIMIT_MAX(val_w, 3);
//         tmp.constraint_weight << val_w,val_w,val_w,0,0,0;
// //        tmp.constraint_weight << 3,3,3,0,0,0;
//         double margin = 1e-3;
//         if(sccp->avoid_priority(sccp->collision_info_list[i].id0) > sccp->avoid_priority(sccp->collision_info_list[i].id1)){
//             tmp.localPos = sccp->collision_info_list[i].cp1_local;
//             tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id1)->name;
//             tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
//             ikc_list.push_back(tmp);
//         }else if(sccp->avoid_priority(sccp->collision_info_list[i].id0) < sccp->avoid_priority(sccp->collision_info_list[i].id1)){
//             tmp.localPos = sccp->collision_info_list[i].cp0_local;
//             tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id0)->name;
//             tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
//             ikc_list.push_back(tmp);
//         }else{
//             tmp.localPos = sccp->collision_info_list[i].cp1_local;
//             tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id1)->name;
//             tmp.targetPos = sccp->collision_info_list[i].cp0_wld + (sccp->collision_info_list[i].cp1_wld - sccp->collision_info_list[i].cp0_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
//             ikc_list.push_back(tmp);
//             tmp.localPos = sccp->collision_info_list[i].cp0_local;
//             tmp.target_link_name = fik->m_robot->joint(sccp->collision_info_list[i].id0)->name;
//             tmp.targetPos = sccp->collision_info_list[i].cp1_wld + (sccp->collision_info_list[i].cp0_wld - sccp->collision_info_list[i].cp1_wld).normalized() * (sccp->collision_info_list[i].dist_safe + margin);
//             ikc_list.push_back(tmp);
//         }
// //        ikc_list[3].constraint_weight =  hrp::dvector6::Constant(1e-4);
// //        ikc_list[4].constraint_weight =  hrp::dvector6::Constant(1e-4);
//     }

//     if(loop%20==0){
//         if(sccp->collision_info_list.size()>0){
//             std::cout<<"pair:"<<std::endl;
//             for(int i=0;i<sccp->collision_info_list.size();i++){
//                 std::cout<<fik->m_robot->joint(sccp->collision_info_list[i].id0)->name<<" "<<fik->m_robot->joint(sccp->collision_info_list[i].id1)->name<<endl;
//             }
//         }
//     }

//     if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT0")->jointId) = 1e3;//JAXON
//     if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT1")->jointId) = 1e3;
// //    if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 10;
//     if( fik->m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_JOINT2")->jointId) = 0;//実機修理中

//     if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->m_robot->link("CHEST_JOINT0")->llimit = deg2rad(-8);
//     if( fik->m_robot->link("CHEST_JOINT0") != NULL) fik->m_robot->link("CHEST_JOINT0")->ulimit = deg2rad(8);
//     if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->m_robot->link("CHEST_JOINT1")->llimit = deg2rad(1);
//     if( fik->m_robot->link("CHEST_JOINT1") != NULL) fik->m_robot->link("CHEST_JOINT1")->ulimit = deg2rad(32);

//     if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->llimit = deg2rad(-20);
//     if( fik->m_robot->link("HEAD_JOINT0") != NULL) fik->m_robot->link("HEAD_JOINT0")->ulimit = deg2rad(20);
//     if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->llimit = deg2rad(-15);
//     if( fik->m_robot->link("HEAD_JOINT1") != NULL) fik->m_robot->link("HEAD_JOINT1")->ulimit = deg2rad(35);

//     if( fik->m_robot->link("RARM_JOINT6") != NULL) fik->m_robot->link("RARM_JOINT6")->llimit = deg2rad(-59);
//     if( fik->m_robot->link("RARM_JOINT6") != NULL) fik->m_robot->link("RARM_JOINT6")->ulimit = deg2rad(59);
//     if( fik->m_robot->link("RARM_JOINT7") != NULL) fik->m_robot->link("RARM_JOINT7")->llimit = deg2rad(-61);
//     if( fik->m_robot->link("RARM_JOINT7") != NULL) fik->m_robot->link("RARM_JOINT7")->ulimit = deg2rad(58);

//     if( fik->m_robot->link("LARM_JOINT6") != NULL) fik->m_robot->link("LARM_JOINT6")->llimit = deg2rad(-59);
//     if( fik->m_robot->link("LARM_JOINT6") != NULL) fik->m_robot->link("LARM_JOINT6")->ulimit = deg2rad(59);
//     if( fik->m_robot->link("LARM_JOINT7") != NULL) fik->m_robot->link("LARM_JOINT7")->llimit = deg2rad(-61);
//     if( fik->m_robot->link("LARM_JOINT7") != NULL) fik->m_robot->link("LARM_JOINT7")->ulimit = deg2rad(58);

//     if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->ulimit = deg2rad(-45);//脇内側の干渉回避
//     if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->llimit = deg2rad(45);
//     if( fik->m_robot->link("RARM_JOINT2") != NULL) fik->m_robot->link("RARM_JOINT2")->llimit = deg2rad(-89);//肩グルン防止
//     if( fik->m_robot->link("LARM_JOINT2") != NULL) fik->m_robot->link("LARM_JOINT2")->ulimit = deg2rad(89);
//     if( fik->m_robot->link("RARM_JOINT4") != NULL) fik->m_robot->link("RARM_JOINT4")->ulimit = deg2rad(1);//肘逆折れ
//     if( fik->m_robot->link("LARM_JOINT4") != NULL) fik->m_robot->link("LARM_JOINT4")->ulimit = deg2rad(1);
//     if( fik->m_robot->link("RLEG_JOINT3") != NULL) fik->m_robot->link("RLEG_JOINT3")->llimit = deg2rad(40);//膝伸びきり防止のため
//     if( fik->m_robot->link("LLEG_JOINT3") != NULL) fik->m_robot->link("LLEG_JOINT3")->llimit = deg2rad(40);

//     if( fik->m_robot->link("CHEST_Y") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_Y")->jointId) = 10;//K
//     if( fik->m_robot->link("CHEST_P") != NULL) fik->dq_weight_all(fik->m_robot->link("CHEST_P")->jointId) = 10;
//     if( fik->m_robot->link("R_KNEE_P") != NULL) fik->m_robot->link("R_KNEE_P")->llimit = deg2rad(30);//膝伸びきり防止
//     if( fik->m_robot->link("L_KNEE_P") != NULL) fik->m_robot->link("L_KNEE_P")->llimit = deg2rad(30);
//     // if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->llimit = deg2rad(-40);
//     // if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->llimit = deg2rad(-40);
//     // if( fik->m_robot->link("R_WRIST_R") != NULL) fik->m_robot->link("R_WRIST_R")->ulimit = deg2rad(40);
//     // if( fik->m_robot->link("L_WRIST_R") != NULL) fik->m_robot->link("L_WRIST_R")->ulimit = deg2rad(40);
//     // if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-40);
//     // if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-40);
//     // if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(20);
//     // if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(20);
//     if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->llimit = deg2rad(-80);
//     if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->llimit = deg2rad(-80);
//     if( fik->m_robot->link("R_WRIST_P") != NULL) fik->m_robot->link("R_WRIST_P")->ulimit = deg2rad(45);
//     if( fik->m_robot->link("L_WRIST_P") != NULL) fik->m_robot->link("L_WRIST_P")->ulimit = deg2rad(45);
//     if( fik->m_robot->link("CHEST_Y") != NULL) fik->m_robot->link("CHEST_Y")->llimit = deg2rad(-20);
//     if( fik->m_robot->link("CHEST_Y") != NULL) fik->m_robot->link("CHEST_Y")->ulimit = deg2rad(20);
//     if( fik->m_robot->link("CHEST_P") != NULL) fik->m_robot->link("CHEST_P")->llimit = deg2rad(0);
//     if( fik->m_robot->link("CHEST_P") != NULL) fik->m_robot->link("CHEST_P")->ulimit = deg2rad(60);
//     if( fik->m_robot->link("HEAD_Y") != NULL) fik->m_robot->link("HEAD_Y")->llimit = deg2rad(-5);
//     if( fik->m_robot->link("HEAD_Y") != NULL) fik->m_robot->link("HEAD_Y")->ulimit = deg2rad(5);
//     if( fik->m_robot->link("HEAD_P") != NULL) fik->m_robot->link("HEAD_P")->llimit = deg2rad(0);
//     if( fik->m_robot->link("HEAD_P") != NULL) fik->m_robot->link("HEAD_P")->ulimit = deg2rad(60);
//     for(int i=0;i<fik->m_robot->numJoints();i++){
//         LIMIT_MINMAX(fik->m_robot->joint(i)->q, fik->m_robot->joint(i)->llimit, fik->m_robot->joint(i)->ulimit);
//     }

//     fik->q_ref.head(m_qRef.data.length()) = hrp::to_dvector(m_qRef.data);//あえてseqからのbaselink poseは信用しない

//     for(int i=0; i<fik->m_robot->numJoints(); i++){
//         if(!has(wbms->wp.use_joints, fik->m_robot->joint(i)->name)){
//             fik->dq_weight_all(i) = 0;
//             fik->m_robot->joint(i)->q = m_qRef.data[i];
//         }
//     }


//     if(fik->m_robot->name().find("JAXON") != std::string::npos){
//         for(int i=0; i<fik->m_robot->numJoints(); i++){
//             if(fik->m_robot->joint(i)->name.find("ARM") != std::string::npos){
//                 fik->q_ref_constraint_weight(i) = 1e-3;//腕だけ
//             }
//         }
//     }

//     const int IK_MAX_LOOP = 1;
//     int loop_result = fik->solveFullbodyIKLoop(ikc_list, IK_MAX_LOOP);
// }

