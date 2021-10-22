#include "TorqueController.h"
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>
#include <cnoid/ValueTree>

namespace WholeBodyTorque {
  TorqueController::PrimitiveTask::PrimitiveTask(const std::string& name) :
    name_(name),
    offset_(cnoid::Position::Identity()),
    dOffsetPrev_(cnoid::Vector6::Zero())
  {
  }

  void TorqueController::PrimitiveTask::calcTorque(const cnoid::BodyPtr& robot_act, cnoid::Vector6& rootWrench, double dt, const std::vector<cnoid::LinkPtr>& useJOints){
    cnoid::Matrix3 eeR = this->offset_.linear() * this->primitiveCommand_->targetPose().linear();

    cnoid::Vector6 targetWrenchLocal; //local系
    targetWrenchLocal.head<3>() = eeR.transpose() * this->primitiveCommand_->targetWrench().head<3>();
    targetWrenchLocal.tail<3>() = eeR.transpose() * this->primitiveCommand_->targetWrench().tail<3>();

    cnoid::Vector6 offsetPrev; //world系
    offsetPrev.head<3>() = this->offset_.translation();
    offsetPrev.tail<3>() = cnoid::omegaFromRot(this->offset_.linear());

    cnoid::Vector6 offsetPrevLocal; //local系
    offsetPrevLocal.head<3>() = eeR.transpose() * offsetPrev.head<3>();
    offsetPrevLocal.tail<3>() = eeR.transpose() * offsetPrev.tail<3>();

    cnoid::Vector6 dOffsetPrevLocal; //local系
    dOffsetPrevLocal.head<3>() = eeR.transpose() * this->dOffsetPrev_.head<3>();
    dOffsetPrevLocal.tail<3>() = eeR.transpose() * this->dOffsetPrev_.tail<3>();

    cnoid::Vector6 dOffsetLocal; //local系
    for(size_t i=0;i<6;i++){
      if(this->primitiveCommand_->M()[i] == 0.0 && this->primitiveCommand_->D()[i] == 0.0 && this->primitiveCommand_->K()[i]==0.0){
        dOffsetLocal[i] = 0.0;
        continue;
      }

      dOffsetLocal[i] =
        ((this->primitiveCommand_->actWrench()[i] - targetWrenchLocal[i]) * this->primitiveCommand_->wrenchFollowGain()[i] * dt * dt
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

  void TorqueController::reset() {
    this->positionTaskMap_.clear();
  }

  void TorqueController::calcGravityCompensation(cnoid::BodyPtr& robot_act, cnoid::Vector6& rootWrench, const std::vector<cnoid::LinkPtr>& useJoints) {
    cnoid::Vector3 vorg = robot_act->rootLink()->v();
    cnoid::Vector3 worg = robot_act->rootLink()->w();
    cnoid::Vector3 dvorg = robot_act->rootLink()->dv();
    cnoid::Vector3 dworg = robot_act->rootLink()->dw();
    robot_act->rootLink()->v() << 0.0, 0.0, 0.0;
    robot_act->rootLink()->w() << 0.0, 0.0, 0.0;
    robot_act->rootLink()->dv() << 0.0, 0.0, 9.80665;
    robot_act->rootLink()->dw() << 0.0, 0.0, 0.0;
    cnoid::VectorX dqorg(robot_act->numAllJoints());
    cnoid::VectorX ddqorg(robot_act->numAllJoints());
    for(size_t i=0;i<robot_act->numAllJoints();i++){
      dqorg[i]=robot_act->joint(i)->dq();
      robot_act->joint(i)->dq() = 0.0;
      ddqorg[i]=robot_act->joint(i)->ddq();
      robot_act->joint(i)->ddq() = 0.0;
    }
    robot_act->calcForwardKinematics(true,true);
    cnoid::Vector6 rootGravityCompensationWrench = cnoid::calcInverseDynamics(robot_act->rootLink());

    for(int i=0;i<useJoints.size();i++) {
      if(useJoints[i]->isRoot()) rootWrench += rootGravityCompensationWrench;
      else useJoints[i]->u() += robot_act->joint(useJoints[i]->jointId())->u();
    }

    robot_act->rootLink()->v() = vorg;
    robot_act->rootLink()->w() = worg;
    robot_act->rootLink()->dv() = dvorg;
    robot_act->rootLink()->dw() = dworg;
    for(size_t i=0;i<robot_act->numAllJoints();i++){
      robot_act->joint(i)->dq() = dqorg[i];
      robot_act->joint(i)->ddq() = ddqorg[i];
    }
  }

  void TorqueController::getPrimitiveCommand(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >& positionTaskMap) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >::iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); ) {
      if (primitiveCommandMap.find(it->first) == primitiveCommandMap.end()) it = positionTaskMap.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
      if(positionTaskMap.find(it->first)==positionTaskMap.end()){
        positionTaskMap[it->first] = std::make_shared<TorqueController::PrimitiveTask>(it->first);
      }
    }
    // 各指令値の反映
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++){
      positionTaskMap[it->first]->updateFromPrimitiveCommand(it->second);
    }
  }

  void TorqueController::control(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, // primitive motion level target
                                 const std::vector<std::shared_ptr<WholeBodyTorque::Collision> >& collisions, // current self collision state
                                 const cnoid::BodyPtr& robot_ref, // command level target
                                 cnoid::BodyPtr& robot_act,
                                 const std::vector<cnoid::LinkPtr>& useJoints, // input and output
                                 std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                                 double dt,
                                 int debugLevel
                                 ) {
    for(int i=0;i<useJoints.size();i++) useJoints[i]->u() = 0.0;
    cnoid::Vector6 rootWrench = cnoid::Vector6::Zero();

    // 重力補償トルクを足す
    TorqueController::calcGravityCompensation(robot_act, rootWrench, useJoints);

    // command level指令の関節角度に追従するトルクを足す
    //  モータドライバで行う TODO
    for(size_t i=0;i<useJoints.size();i++) if(useJoints[i]->jointId()>=0) useJoints[i]->q() = robot_ref->joint(useJoints[i]->jointId())->q();

    // 目標primitive commandを取得
    TorqueController::getPrimitiveCommand(primitiveCommandMap, this->positionTaskMap_);

    // 各primitive command実現のためのトルクを足す (supportCOM, comを除く)
    for(std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >::iterator it = this->positionTaskMap_.begin(); it != this->positionTaskMap_.end(); it++) {
      it->second->calcTorque(robot_act, rootWrench, dt, useJoints);
    }

    // 各primitive command実現のためのトルクを足す (supportCOM, com)
    // TODO

    // トルク上限でリミット
    for(int i=0;i<useJoints.size();i++){
      double climit, gearRatio, torqueConst;
      useJoints[i]->info()->read("climit",climit);
      useJoints[i]->info()->read("gearRatio",gearRatio);
      useJoints[i]->info()->read("torqueConst",torqueConst);
      double maxTorque = climit * gearRatio * torqueConst;
      useJoints[i]->u() = std::min(std::max(useJoints[i]->u(), -maxTorque), maxTorque);
    }
  }



}
