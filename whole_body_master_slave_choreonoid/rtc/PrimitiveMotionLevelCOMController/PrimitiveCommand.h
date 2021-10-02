#ifndef PrimitiveMotionLevelController_PrimitiveCommand_H
#define PrimitiveMotionLevelController_PrimitiveCommand_H

#include <cnoid/Body>
#include <whole_body_master_slave_choreonoid/idl/PrimitiveState.hh>
#include <cpp_filters/TwoPointInterpolator.h>

namespace PrimitiveMotionLevel {
  class PrimitiveCommand {
  public:
    PrimitiveCommand(const std::string& name);
    void updateFromIdl(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveState& idl);
    void updateTargetForOneStep(double dt);
    const std::string& name() const { return name_;}
    const std::string& parentLinkName() const { return parentLinkName_;}
    const cnoid::Position& localPose() const { return localPose_;}
    const cnoid::Position& targetPose() const { return targetPose_;}
    const cnoid::Vector6& targetWrench() const { return targetWrench_;}
    const cnoid::Vector6& M() const { return M_;}
    const cnoid::Vector6& D() const { return D_;}
    const cnoid::Vector6& K() const { return K_;}
    const cnoid::Vector6& actWrench() const { return actWrench_;}
    const cnoid::Vector6& wrenchGain() const { return wrenchGain_;}
    const bool& supportCOM() const { return supportCOM_; }
    const bool& supportCOMChanged() const { return supportCOMChanged_;}
  protected:
    std::string name_;
    std::string parentLinkName_;
    cnoid::Position localPose_;

    cnoid::Position targetPose_; //world frame
    cnoid::Position targetPosePrev_; //world frame
    cnoid::Position targetPosePrevPrev_; //world frame
    cpp_filters::TwoPointInterpolator<cnoid::Vector3> targetPositionInterpolator_; //world frame
    cpp_filters::TwoPointInterpolatorSO3 targetOrientationInterpolator_; //world frame
    cnoid::Vector6 targetWrench_; //world frame
    cpp_filters::TwoPointInterpolator<cnoid::Vector6> targetWrenchInterpolator_; //world frame

    cnoid::Vector6 M_, D_, K_;// local frame

    cnoid::Vector6 actWrench_; // local frame // Position control only
    cnoid::Vector6 wrenchGain_; // local frame // Position control only

    bool supportCOM_;
    bool supportCOMChanged_;
  };
}

#endif
