#include "PrimitiveCommand.h"

#include <cnoid/EigenUtil>

namespace PrimitiveMotionLevel {

  PrimitiveCommand::PrimitiveCommand(const std::string& name) :
    name_(name),
    parentLinkName_(""),
    localPose_(cnoid::Position::Identity()),
    targetPose_(cnoid::Position::Identity()),
    targetPosePrev_(cnoid::Position::Identity()),
    targetPosePrevPrev_(cnoid::Position::Identity()),
    targetPositionInterpolator_(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),
    targetOrientationInterpolator_(cnoid::Matrix3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),
    targetWrench_(cnoid::Vector6::Zero()),
    targetWrenchInterpolator_(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB),
    M_(cnoid::Vector6::Zero()),
    D_(cnoid::Vector6::Zero()),
    K_(cnoid::Vector6::Zero()),
    actWrench_(cnoid::Vector6::Zero()),
    wrenchGain_(cnoid::Vector6::Zero()),
    supportCOM_(false),
    supportCOMChanged_(false)
  {
  }

  void PrimitiveCommand::updateFromIdl(const whole_body_master_slave_choreonoid::PrimitiveStateIdl& idl) {
    this->parentLinkName_ = idl.parentLinkName;
    this->localPose_.translation()[0] = idl.localPose.position.x;
    this->localPose_.translation()[1] = idl.localPose.position.y;
    this->localPose_.translation()[2] = idl.localPose.position.z;
    this->localPose_.linear() = cnoid::rotFromRpy(idl.localPose.orientation.r,idl.localPose.orientation.p,idl.localPose.orientation.y);
    cnoid::Position pose;
    pose.translation()[0] = idl.pose.position.x;
    pose.translation()[1] = idl.pose.position.y;
    pose.translation()[2] = idl.pose.position.z;
    pose.linear() = cnoid::rotFromRpy(idl.pose.orientation.r,idl.pose.orientation.p,idl.pose.orientation.y);
    this->targetPositionInterpolator_.setGoal(pose.translation(),idl.time);
    this->targetOrientationInterpolator_.setGoal(pose.linear(),idl.time);
    cnoid::Vector6 wrench; for(size_t i=0;i<6;i++) wrench[i] = idl.wrench[i];
    this->targetWrenchInterpolator_.setGoal(wrench,idl.time);
    for(size_t i=0;i<6;i++) this->M_[i] = idl.M[i];
    for(size_t i=0;i<6;i++) this->D_[i] = idl.D[i];
    for(size_t i=0;i<6;i++) this->K_[i] = idl.K[i];
    for(size_t i=0;i<6;i++) this->actWrench_[i] = idl.actWrench[i];
    for(size_t i=0;i<6;i++) this->wrenchGain_[i] = idl.wrenchGain[i];
    if(this->supportCOM_ != idl.supportCOM) this->supportCOMChanged_ = true;
    else this->supportCOMChanged_ = false;
    this->supportCOM_ = idl.supportCOM;
  }

  void PrimitiveCommand::updateTargetForOneStep(double dt) {
    this->targetPosePrevPrev_ = this->targetPosePrev_;
    this->targetPosePrev_ = this->targetPose_;
    cnoid::Vector3 trans;
    this->targetPositionInterpolator_.get(trans, dt);
    this->targetPose_.translation() = trans;
    cnoid::Matrix3 R;
    this->targetOrientationInterpolator_.get(R, dt);
    this->targetPose_.linear() = R;
    this->targetWrenchInterpolator_.get(this->targetWrench_, dt);
  }

};
