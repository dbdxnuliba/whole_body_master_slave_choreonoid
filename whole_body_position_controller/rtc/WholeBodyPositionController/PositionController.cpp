#include "PositionController.h"
#include <cnoid/EigenUtil>

namespace WholeBodyPosition {
  PositionController::PositionTask::PositionTask(const std::string& name) :
    name_(name),
    offset_(cnoid::Position::Identity()),
    dOffsetPrev_(cnoid::Vector6::Zero())
  {
  }

  void PositionController::PositionTask::calcImpedanceControl(double dt){
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

  void PositionController::PositionTask::getIKConstraintsforSupportEEF(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight) {
    if(this->name_ != "com" && this->primitiveCommand_->supportCOM()){
      PositionTask::calcPositionConstraint(robot_com, this->primitiveCommand_, this->offset_, weight, this->positionConstraint_, dt);
      if(this->positionConstraint_) ikConstraints.push_back(this->positionConstraint_);
    }
  }

  void PositionController::PositionTask::getIKConstraintsforInteractEEF(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight) {
    if(this->name_ != "com" && !this->primitiveCommand_->supportCOM()){
      PositionTask::calcPositionConstraint(robot_com, this->primitiveCommand_, this->offset_, weight, this->positionConstraint_, dt);
      if(this->positionConstraint_) ikConstraints.push_back(this->positionConstraint_);
    }
  }

  void PositionController::PositionTask::calcPositionConstraint(const cnoid::BodyPtr& robot_com,
                                                                const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                                                const cnoid::Position& offset,
                                                                double weight,
                                                                std::shared_ptr<IK::PositionConstraint>& positionConstraint,
                                                                double dt){
    if(!robot_com->link(primitiveCommand->parentLinkName())){
      std::cerr << "link " << primitiveCommand->parentLinkName() << " not exist" << std::endl;
      positionConstraint = nullptr;
      return;
    }
    if(!positionConstraint) positionConstraint = std::make_shared<IK::PositionConstraint>();
    positionConstraint->A_link() = robot_com->link(primitiveCommand->parentLinkName());
    positionConstraint->A_localpos() = primitiveCommand->localPose();
    positionConstraint->B_link() = nullptr;
    positionConstraint->B_localpos().translation() = offset.translation() + primitiveCommand->targetPose().translation();
    positionConstraint->B_localpos().linear() = offset.linear() * primitiveCommand->targetPose().linear();
    positionConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    positionConstraint->precision() << 1e-4, 1e-4, 1e-4, 0.001745, 0.001745, 0.001745;
    positionConstraint->weight() = primitiveCommand->poseFollowGain();
    positionConstraint->weight() *= weight;
    positionConstraint->eval_link() = nullptr;
    positionConstraint->eval_localR() = positionConstraint->B_localpos().linear();

  }

  void PositionController::PositionTask::getIKConstraintsforCOM(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight) {
    if(this->name_ == "com"){
      PositionTask::calcCOMConstraint(robot_com, this->primitiveCommand_, weight, this->comConstraint_, dt);
      if(this->comConstraint_) ikConstraints.push_back(this->comConstraint_);
    }
  }

  void PositionController::PositionTask::getIKConstraintsforCOMRegion(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight) {
    if(this->name_ == "com"){
      PositionTask::calcCOMRegionConstraint(robot_com, this->primitiveCommand_, weight, this->comRegionConstraint_, dt);
      if(this->comRegionConstraint_) ikConstraints.push_back(this->comRegionConstraint_);
    }
  }

  void PositionController::PositionTask::calcCOMConstraint(const cnoid::BodyPtr& robot_com,
                                                           const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                                           double weight,
                                                           std::shared_ptr<IK::COMConstraint>& comConstraint,
                                                           double dt){
    if(!comConstraint) comConstraint = std::make_shared<IK::COMConstraint>();

    comConstraint->A_robot() = robot_com;
    comConstraint->B_localp() = primitiveCommand->targetPose().translation();
    comConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    comConstraint->precision() << 1e-4, 1e-4, 1e-4;
    comConstraint->weight() = primitiveCommand->poseFollowGain().head<3>();
    comConstraint->weight() *= weight;
    comConstraint->eval_R() = primitiveCommand->targetPose().linear();
  }

  void PositionController::PositionTask::calcCOMRegionConstraint(const cnoid::BodyPtr& robot_com,
                                                                 const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                                                 double weight,
                                                                 std::shared_ptr<IK::COMConstraint>& comConstraint,
                                                                 double dt){
    if(primitiveCommand->poseC().rows() == 0) {
      comConstraint = nullptr;
      return;
    }
    if(!comConstraint) comConstraint = std::make_shared<IK::COMConstraint>();
    comConstraint->A_robot() = robot_com;
    if(primitiveCommand->isPoseCGlobal()){
      comConstraint->B_localp() = cnoid::Vector3::Zero();
      comConstraint->eval_R() = cnoid::Matrix3d::Identity();
    }else{
      comConstraint->B_localp() = primitiveCommand->targetPose().translation();
      comConstraint->eval_R() = primitiveCommand->targetPose().linear();
    }
    comConstraint->C() = Eigen::SparseMatrix<double,Eigen::ColMajor>(primitiveCommand->poseC()).leftCols<3>() * weight;
    comConstraint->du() = primitiveCommand->poseud() * weight;
    comConstraint->dl() = primitiveCommand->poseld() * weight;
    comConstraint->maxCError() = cnoid::VectorX::Ones(comConstraint->C().rows()) * 10.0*dt * weight;
    comConstraint->CPrecision() = cnoid::VectorX::Ones(comConstraint->C().rows()) * 1e-4 * weight;
    comConstraint->weight() = cnoid::Vector3::Zero(); //disable equality
  }

  void PositionController::reset() {
    this->positionTaskMap_.clear();
    this->fullbodyIKSolver_ = FullbodyIKSolver();
    this->prioritizedIKSolver_ = PrioritizedIKSolver();
  }

  void PositionController::getPrimitiveCommand(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<PositionController::PositionTask> >& positionTaskMap) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<PositionController::PositionTask> >::iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); ) {
      if (primitiveCommandMap.find(it->first) == primitiveCommandMap.end()) it = positionTaskMap.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
      if(positionTaskMap.find(it->first)==positionTaskMap.end()){
        positionTaskMap[it->first] = std::make_shared<PositionController::PositionTask>(it->first);
      }
    }
    // 各指令値の反映
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++){
      positionTaskMap[it->first]->updateFromPrimitiveCommand(it->second);
    }
  }

  void PositionController::getCommandLevelIKConstraints(const cnoid::BodyPtr& robot_ref, std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> >& jointAngleConstraint, std::shared_ptr<IK::PositionConstraint>& rootLinkConstraint, std::vector<std::shared_ptr<IK::IKConstraint> >& commandLevelIKConstraints, const cnoid::BodyPtr& robot_com, double dt, bool followRootLink, double weight) {
    for(size_t i=0;i<robot_com->numJoints();i++){
      if(jointAngleConstraint.find(robot_com->joint(i))==jointAngleConstraint.end()){
        std::shared_ptr<IK::JointAngleConstraint> jac = std::make_shared<IK::JointAngleConstraint>();
        jac->joint() = robot_com->joint(i);
        jac->maxError() = 1.0 * dt;
        jac->weight() = weight;
        jointAngleConstraint[robot_com->joint(i)] = jac;
      }
      std::shared_ptr<IK::JointAngleConstraint>& jac = jointAngleConstraint[robot_com->joint(i)];
      jac->targetq() = robot_ref->joint(i)->q();
      commandLevelIKConstraints.push_back(jac);
    }
    if(followRootLink && robot_com->rootLink()->jointType() != cnoid::Link::FIXED_JOINT){
      if(!rootLinkConstraint){
        rootLinkConstraint = std::make_shared<IK::PositionConstraint>();
        rootLinkConstraint->A_link() = robot_com->rootLink();
        rootLinkConstraint->A_localpos() = cnoid::Position::Identity();
        rootLinkConstraint->B_link() = nullptr;
        rootLinkConstraint->maxError() << 1*dt, 1*dt, 1*dt, 1*dt, 1*dt, 1*dt;
        rootLinkConstraint->precision() << 1e-4, 1e-4, 1e-4, 0.001745, 0.001745, 0.001745;
        rootLinkConstraint->weight() << 1.0,1.0,1.0,1.0,1.0,1.0;
        rootLinkConstraint->weight() *= weight;
      }
      rootLinkConstraint->B_localpos() = robot_ref->rootLink()->T();
      commandLevelIKConstraints.push_back(rootLinkConstraint);
    }
  }

  void PositionController::getJointLimitIKConstraints(std::unordered_map<cnoid::LinkPtr,std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> >& jointLimitConstraintMap,
                                                      std::vector<std::shared_ptr<IK::IKConstraint> >& jointLimitIKConstraints,
                                                      const cnoid::BodyPtr& robot_com,
                                                      std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                                                      double dt,
                                                      double weight){
    for(size_t i=0;i<robot_com->numJoints();i++){
      if(jointLimitConstraintMap.find(robot_com->joint(i))==jointLimitConstraintMap.end()){
        std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> jac = std::make_shared<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>();
        jac->joint() = robot_com->joint(i);
        jac->maxError() = 1.0*dt;
        jac->weight() = weight;
        jointLimitConstraintMap[robot_com->joint(i)] = jac;
      }
      std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>& jac = jointLimitConstraintMap[robot_com->joint(i)];
      if(jointLimitTablesMap.find(robot_com->joint(i)) != jointLimitTablesMap.end()){
        jac->jointLimitTables() = jointLimitTablesMap[robot_com->joint(i)];
      }else{
        jac->jointLimitTables().clear();
      }
      jointLimitIKConstraints.push_back(jac);
    }
  }

  void PositionController::getJointVelocityIKConstraints(std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointVelocityConstraint> >& jointVelocityConstraintMap, std::vector<std::shared_ptr<IK::IKConstraint> >& jointVelocityIKConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight){
    for(size_t i=0;i<robot_com->numJoints();i++){
      if(jointVelocityConstraintMap.find(robot_com->joint(i))==jointVelocityConstraintMap.end()){
        std::shared_ptr<IK::JointVelocityConstraint> jac = std::make_shared<IK::JointVelocityConstraint>();
        jac->joint() = robot_com->joint(i);
        jac->maxError() = 0.1;
        jac->weight() = weight;
        jointVelocityConstraintMap[robot_com->joint(i)] = jac;
      }
      std::shared_ptr<IK::JointVelocityConstraint>& jac = jointVelocityConstraintMap[robot_com->joint(i)];
      jac->dt() = dt;
      jointVelocityIKConstraints.push_back(jac);
    }
    if(robot_com->rootLink()->jointType() != cnoid::Link::FIXED_JOINT){
      if(jointVelocityConstraintMap.find(robot_com->rootLink())==jointVelocityConstraintMap.end()){
        std::shared_ptr<IK::JointVelocityConstraint> jac = std::make_shared<IK::JointVelocityConstraint>();
        jac->joint() = robot_com->rootLink();
        jac->maxError() = 0.1;
        jac->weight() = weight;
        jointVelocityConstraintMap[robot_com->rootLink()] = jac;
      }
      std::shared_ptr<IK::JointVelocityConstraint>& jac = jointVelocityConstraintMap[robot_com->rootLink()];
      jac->dt() = dt;
      jointVelocityIKConstraints.push_back(jac);
    }
  }

  void PositionController::getCOMVelocityIKConstraints(std::shared_ptr<IK::COMVelocityConstraint> cOMVelocityConstraint,  std::vector<std::shared_ptr<IK::IKConstraint> >& iKConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight) {
    if(!cOMVelocityConstraint) cOMVelocityConstraint = std::make_shared<IK::COMVelocityConstraint>();

    cOMVelocityConstraint->robot() = robot_com;
    cOMVelocityConstraint->maxVel() = cnoid::Vector3::Ones() * 0.05;
    cOMVelocityConstraint->minVel() = - cnoid::Vector3::Ones() * 0.05;
    cOMVelocityConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    cOMVelocityConstraint->precision() << 1e-4, 1e-4, 1e-4;
    cOMVelocityConstraint->weight() = cnoid::Vector3::Ones()*weight;
    cOMVelocityConstraint->dt() = dt;

    iKConstraints.push_back(cOMVelocityConstraint);
  }

  void PositionController::getCollisionIKConstraints(std::vector<std::shared_ptr<IK::ClientCollisionConstraint> >& collisionConstraints, std::vector<std::shared_ptr<IK::IKConstraint> >& collisionIKConstraints, const cnoid::BodyPtr& robot_com, const std::vector<std::shared_ptr<WholeBodyPosition::Collision> >& collisions, double dt, double weight){
    collisionConstraints.resize(collisions.size());
    for(size_t i=0;i<collisionConstraints.size();i++){
      if(!collisionConstraints[i]) collisionConstraints[i] = std::make_shared<IK::ClientCollisionConstraint>();
      collisionConstraints[i]->A_link() = robot_com->link(collisions[i]->link1());
      collisionConstraints[i]->B_link() = robot_com->link(collisions[i]->link2());
      collisionConstraints[i]->tolerance() = 0.01; // 1cm
      collisionConstraints[i]->maxError() = 1.0*dt;
      collisionConstraints[i]->weight() = weight;
      collisionConstraints[i]->A_localp() = collisions[i]->point1();
      collisionConstraints[i]->B_localp() = collisions[i]->point2();
      collisionConstraints[i]->direction() = collisions[i]->direction21();

      // 全自己干渉情報を与えると計算コストが膨大になるため、距離が近いもののみ与える
      if(collisions[i]->distance() < 0.01 + 0.05){
        collisionIKConstraints.push_back(collisionConstraints[i]);
      }
    }
  }

  void PositionController::control(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, // primitive motion level target
                                   const std::vector<std::shared_ptr<WholeBodyPosition::Collision> >& collisions, // current self collision state
                                   const cnoid::BodyPtr& robot_ref, // command level target
                                   const std::vector<cnoid::LinkPtr>& useJoints,
                                   std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                                   cnoid::BodyPtr& robot_com, //output
                                   double dt,
                                   int debugLevel
                                   ) {
    // 目標値を反映
    PositionController::getPrimitiveCommand(primitiveCommandMap, this->positionTaskMap_);

    // IK目標値を計算
    for(std::map<std::string, std::shared_ptr<PositionController::PositionTask> >::iterator it = this->positionTaskMap_.begin(); it != this->positionTaskMap_.end(); it++) {
      it->second->calcImpedanceControl(dt);
    }

    // solve ik
    switch(this->solveMode_){
    case MODE_FULLBODY:
      this->fullbodyIKSolver_.solveFullbodyIK(robot_com, robot_ref, this->positionTaskMap_, useJoints, dt, this->followRootLink_, debugLevel);
      break;
    case MODE_PRIORITIZED:
    default:
      this->prioritizedIKSolver_.solvePrioritizedIK(robot_com, robot_ref, this->positionTaskMap_, jointLimitTablesMap, collisions, useJoints, dt, this->followRootLink_, debugLevel);
      break;
    }
  }


  void PositionController::FullbodyIKSolver::solveFullbodyIK(cnoid::BodyPtr& robot_com,
                                                             const cnoid::BodyPtr& robot_ref,
                                                             const std::map<std::string, std::shared_ptr<PositionTask> >& positionTaskMap,
                                                             const std::vector<cnoid::LinkPtr>& useJoints,
                                                             double dt,
                                                             bool followRootLink,
                                                             int debugLevel){
    if(this->jlim_avoid_weight_old_.size() != 6+robot_com->numJoints()) this->jlim_avoid_weight_old_ = cnoid::VectorX::Zero(6+robot_com->numJoints());
    cnoid::VectorX dq_weight_all = cnoid::VectorX::Zero(6+robot_com->numJoints());
    for(int i=0;i<useJoints.size();i++){
      if(useJoints[i]->isRoot() && !useJoints[i]->isFixedJoint()) dq_weight_all.head<6>() = cnoid::Vector6::Ones();
      else dq_weight_all[6+useJoints[i]->jointId()] = 1.0;
    }

    std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint;

    // primitive motion levelのIKConstraintを取得
    for(std::map<std::string, std::shared_ptr<PositionController::PositionTask> >::const_iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); it++) {
      it->second->getIKConstraintsforSupportEEF(ikConstraint, robot_com, dt, 3.0);
      it->second->getIKConstraintsforInteractEEF(ikConstraint, robot_com, dt, 0.1);
      it->second->getIKConstraintsforCOM(ikConstraint, robot_com, dt, 3.0);
    }

    // command levelのIKConstraintを取得
    PositionController::getCommandLevelIKConstraints(robot_ref, this->jointAngleConstraint_, this->rootLinkConstraint_, ikConstraint, robot_com, dt, followRootLink, 0.01);

    for(int i=0;i<ikConstraint.size();i++) ikConstraint[i]->debuglevel() = debugLevel;
    fik::solveFullbodyIKLoopFast(robot_com,
                                 ikConstraint,
                                 this->jlim_avoid_weight_old_,
                                 dq_weight_all,
                                 1,//loop
                                 1e-6,
                                 debugLevel//debug
                                 );

  }

  void PositionController::PrioritizedIKSolver::solvePrioritizedIK(cnoid::BodyPtr& robot_com,
                                                                   const cnoid::BodyPtr& robot_ref,
                                                                   const std::map<std::string, std::shared_ptr<PositionTask> >& positionTaskMap,
                                                                   std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                                                                   const std::vector<std::shared_ptr<WholeBodyPosition::Collision> >& collisions, // current self collision state
                                                                   const std::vector<cnoid::LinkPtr>& useJoints,
                                                                   double dt,
                                                                   bool followRootLink,
                                                                   int debugLevel){

    // 関節角速度上下限を取得
    std::vector<std::shared_ptr<IK::IKConstraint> > jointVelocityConstraint;
    PositionController::getJointVelocityIKConstraints(this->jointVelocityConstraint_, jointVelocityConstraint, robot_com, dt);

    // 関節角度上下限を取得
    std::vector<std::shared_ptr<IK::IKConstraint> > limitConstraint;
    PositionController::getJointLimitIKConstraints(this->jointLimitConstraint_, limitConstraint, robot_com, jointLimitTablesMap, dt);
    PositionController::getCOMVelocityIKConstraints(this->cOMVelocityConstraint_, limitConstraint, robot_com, dt);

    // 関節角度上下限を取得
    PositionController::getCollisionIKConstraints(this->collisionConstraint_, limitConstraint, robot_com, collisions, dt, 3.0); //weightはweを増やしている

    // primitive motion levelのIKConstraintを取得
    std::vector<std::shared_ptr<IK::IKConstraint> > supportEEFConstraint;
    std::vector<std::shared_ptr<IK::IKConstraint> > COMConstraint;
    std::vector<std::shared_ptr<IK::IKConstraint> > COMRegionConstraint;
    std::vector<std::shared_ptr<IK::IKConstraint> > interactEEFConstraint;
    for(std::map<std::string, std::shared_ptr<PositionController::PositionTask> >::const_iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); it++) {
      it->second->getIKConstraintsforSupportEEF(supportEEFConstraint, robot_com, dt, 5.0);//weightはweを増やしている
      it->second->getIKConstraintsforCOM(COMConstraint, robot_com, dt, 5.0);//weightはweを増やしている
      it->second->getIKConstraintsforCOMRegion(COMRegionConstraint, robot_com, dt, 5.0);//weightはweを増やしている
      it->second->getIKConstraintsforInteractEEF(interactEEFConstraint, robot_com, dt, 5.0);//weightはweを増やしている
    }

    // command levelのIKConstraintを取得
    std::vector<std::shared_ptr<IK::IKConstraint> > commandLevelConstraint;
    PositionController::getCommandLevelIKConstraints(robot_ref, this->jointAngleConstraint_, this->rootLinkConstraint_, commandLevelConstraint, robot_com, dt, followRootLink);

    std::vector<std::vector<std::shared_ptr<IK::IKConstraint> > > ikConstraint;
    ikConstraint.push_back(jointVelocityConstraint);
    ikConstraint.push_back(limitConstraint);
    ikConstraint.push_back(supportEEFConstraint);
    ikConstraint.push_back(COMRegionConstraint);
    ikConstraint.push_back(interactEEFConstraint);
    ikConstraint.push_back(COMConstraint);
    ikConstraint.push_back(commandLevelConstraint);

    for(int i=0;i<ikConstraint.size();i++) for(size_t j=0;j<ikConstraint[i].size();j++) ikConstraint[i][j]->debuglevel() = debugLevel;

    prioritized_inverse_kinematics_solver::solveIKLoop(useJoints,
                                                       ikConstraint,
                                                       this->prevTasks_,
                                                       1,//loop
                                                       1e-3,
                                                       debugLevel//debug
                                                       );

  }

}
// void WholeBodyPositionController::solveFullbodyIK(HumanPose& ref){
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

