#include "TorqueController.h"
#include <cnoid/EigenUtil>

namespace PrimitiveMotionLevelTorque {
  TorqueController::PositionTask::PositionTask(const std::string& name) :
    name_(name),
    offset_(cnoid::Position::Identity()),
    dOffsetPrev_(cnoid::Vector6::Zero())
  {
  }

  void TorqueController::PositionTask::calcImpedanceControl(double dt){
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

  void TorqueController::getPrimitiveCommand(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevelTorque::PrimitiveCommand> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<TorqueController::PositionTask> >& positionTaskMap) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<TorqueController::PositionTask> >::iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); ) {
      if (primitiveCommandMap.find(it->first) == primitiveCommandMap.end()) it = positionTaskMap.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevelTorque::PrimitiveCommand> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
      if(positionTaskMap.find(it->first)==positionTaskMap.end()){
        positionTaskMap[it->first] = std::make_shared<TorqueController::PositionTask>(it->first);
      }
    }
    // 各指令値の反映
    for(std::map<std::string, std::shared_ptr<PrimitiveMotionLevelTorque::PrimitiveCommand> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++){
      positionTaskMap[it->first]->updateFromPrimitiveCommand(it->second);
    }
  }

  void TorqueController::control(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevelTorque::PrimitiveCommand> >& primitiveCommandMap, // primitive motion level target
                                   const std::vector<std::shared_ptr<PrimitiveMotionLevelTorque::Collision> >& collisions, // current self collision state
                                   const cnoid::BodyPtr& robot_ref, // command level target
                                   std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                                   cnoid::BodyPtr& robot_com, //output
                                   double dt,
                                   int debugLevel
                                   ) {
    // 目標値を反映
    TorqueController::getPrimitiveCommand(primitiveCommandMap, this->positionTaskMap_);

    // IK目標値を計算
    for(std::map<std::string, std::shared_ptr<TorqueController::PositionTask> >::iterator it = this->positionTaskMap_.begin(); it != this->positionTaskMap_.end(); it++) {
      it->second->calcImpedanceControl(dt);
    }

  }



}
