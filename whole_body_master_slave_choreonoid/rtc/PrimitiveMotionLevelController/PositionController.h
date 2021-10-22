#ifndef PrimitiveMotionLevelController_PositionControl_H
#define PrimitiveMotionLevelController_PositionControl_H

#include <unordered_map>
#include <cnoid/Body>
#include <joint_limit_table/JointLimitTable.h>
#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/COMVelocityConstraint.h>
#include <ik_constraint/JointVelocityConstraint.h>
#include <ik_constraint/ClientCollisionConstraint.h>
#include <ik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include "Collision.h"

namespace PrimitiveMotionLevel {

  class PositionController {
  public:
    enum SolveMode_enum{ MODE_FULLBODY, MODE_PRIORITIZED};

    void reset();
    void control(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, // primitive motion level target
                 const std::vector<std::shared_ptr<PrimitiveMotionLevel::Collision> >& collisions, // current self collision state
                 const cnoid::BodyPtr& robot_ref, // command level target
                 std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                 cnoid::BodyPtr& robot_com, //output
                 double dt,
                 int debugLevel
                 );
    const SolveMode_enum& solveMode() const { return solveMode_;}
    SolveMode_enum& solveMode() { return solveMode_;}
    const bool& followRootLink() const { return followRootLink_;}
    bool& followRootLink() { return followRootLink_;}

  protected:
    class PositionTask {
    public:
      PositionTask(const std::string& name);
      void updateFromPrimitiveCommand(const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand) {primitiveCommand_ = primitiveCommand;}
      void calcImpedanceControl(double dt);
      void getIKConstraintsforSupportEEF(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight=1.0);
      void getIKConstraintsforInteractEEF(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight=1.0);
      void getIKConstraintsforCOM(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight=1.0);
      void getIKConstraintsforCOMRegion(std::vector<std::shared_ptr<IK::IKConstraint> >& ikConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight=1.0);
      const std::string& name() const { return name_;}
      const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand() const {return primitiveCommand_;}
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
      static void calcPositionConstraint(const cnoid::BodyPtr& robot_com,
                                         const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                         const cnoid::Position& offset,
                                         double weight,
                                         std::shared_ptr<IK::PositionConstraint>& positionConstraint,
                                         double dt);
      static void calcCOMConstraint(const cnoid::BodyPtr& robot_com,
                                    const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                    double weight,
                                    std::shared_ptr<IK::COMConstraint>& comConstraint,
                                    double dt);
      static void calcCOMRegionConstraint(const cnoid::BodyPtr& robot_com,
                                          const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                          double weight,
                                          std::shared_ptr<IK::COMConstraint>& comConstraint,
                                          double dt);

      std::string name_;
      std::shared_ptr<const primitive_motion_level_tools::PrimitiveState> primitiveCommand_;

      cnoid::Position offset_; // world系
      cnoid::Vector6 dOffsetPrev_; // world系

      std::shared_ptr<IK::PositionConstraint> positionConstraint_;
      std::shared_ptr<IK::COMConstraint> comConstraint_;
      std::shared_ptr<IK::COMConstraint> comRegionConstraint_;
    };

    class FullbodyIKSolver {
    public:
      void solveFullbodyIK(cnoid::BodyPtr& robot_com,
                           const cnoid::BodyPtr& robot_ref,
                           const std::map<std::string, std::shared_ptr<PositionTask> >& positionTaskMap_,
                           double dt,
                           bool followRootLink,
                           int debugLevel);
    protected:
      cnoid::VectorX jlim_avoid_weight_old_;

      std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> > jointAngleConstraint_;
      std::shared_ptr<IK::PositionConstraint> rootLinkConstraint_;
    };

    class PrioritizedIKSolver {
    public:
      void solvePrioritizedIK(cnoid::BodyPtr& robot_com,
                              const cnoid::BodyPtr& robot_ref,
                              const std::map<std::string, std::shared_ptr<PositionTask> >& positionTaskMap_,
                              std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                              const std::vector<std::shared_ptr<PrimitiveMotionLevel::Collision> >& collisions, // current self collision state
                              double dt,
                              bool followRootLink,
                              int debugLevel);
    protected:
      std::vector<std::shared_ptr<prioritized_qp::Task> > prevTasks_;

      std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointVelocityConstraint> > jointVelocityConstraint_;
      std::shared_ptr<IK::COMVelocityConstraint> cOMVelocityConstraint_;
      std::unordered_map<cnoid::LinkPtr,std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > jointLimitConstraint_;
      std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> > jointAngleConstraint_;
      std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > collisionConstraint_;
      std::shared_ptr<IK::PositionConstraint> rootLinkConstraint_;
    };
  protected:
    SolveMode_enum solveMode_ = MODE_PRIORITIZED;
    bool followRootLink_ = false;

    std::map<std::string, std::shared_ptr<PositionTask> > positionTaskMap_;

    FullbodyIKSolver fullbodyIKSolver_;
    PrioritizedIKSolver prioritizedIKSolver_;

    // static functions
    static void getPrimitiveCommand(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<PositionController::PositionTask> >& positionTaskMap);
    static void getCommandLevelIKConstraints(const cnoid::BodyPtr& robot_ref, std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> >& jointAngleConstraint, std::shared_ptr<IK::PositionConstraint>& rootLinkConstraint, std::vector<std::shared_ptr<IK::IKConstraint> >& commandLevelIKConstraints, const cnoid::BodyPtr& robot_com, double dt, bool followRootLink, double weight = 1.0);
    static void getJointLimitIKConstraints(std::unordered_map<cnoid::LinkPtr,std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> >& jointLimitConstraintMap, std::vector<std::shared_ptr<IK::IKConstraint> >& jointLimitIKConstraints, const cnoid::BodyPtr& robot_com, std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap, double dt, double weight = 1.0);
    static void getCOMVelocityIKConstraints(std::shared_ptr<IK::COMVelocityConstraint> cOMVelocityConstraint,  std::vector<std::shared_ptr<IK::IKConstraint> >& iKConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight = 1.0);
    static void getCollisionIKConstraints(std::vector<std::shared_ptr<IK::ClientCollisionConstraint> >& collisionConstraints, std::vector<std::shared_ptr<IK::IKConstraint> >& collisionIKConstraints, const cnoid::BodyPtr& robot_com, const std::vector<std::shared_ptr<PrimitiveMotionLevel::Collision> >& collisions, double dt, double weight = 1.0);
    static void getJointVelocityIKConstraints(std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointVelocityConstraint> >& jointVelocityConstraintMap, std::vector<std::shared_ptr<IK::IKConstraint> >& jointVelocityIKConstraints, const cnoid::BodyPtr& robot_com, double dt, double weight = 1.0);
  };
}

#endif
