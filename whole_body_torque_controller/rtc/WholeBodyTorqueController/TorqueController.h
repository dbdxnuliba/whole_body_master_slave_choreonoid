#ifndef WholeBodyTorqueController_PositionControl_H
#define WholeBodyTorqueController_PositionControl_H

#include <unordered_map>
#include <cnoid/Body>
#include <joint_limit_table/JointLimitTable.h>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include "Collision.h"

namespace WholeBodyTorque {

  class TorqueController {
  public:
    void reset();
    void control(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, // primitive motion level target
                 const std::vector<std::shared_ptr<WholeBodyTorque::Collision> >& collisions, // current self collision state
                 const cnoid::BodyPtr& robot_ref, // command level target
                 cnoid::BodyPtr& robot_act,
                 const std::vector<cnoid::LinkPtr>& useJoints, // input and output
                 std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                 double dt,
                 int debugLevel
                 );

  protected:
    class PrimitiveTask {
    public:
      PrimitiveTask(const std::string& name);
      void updateFromPrimitiveCommand(const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand) {primitiveCommand_ = primitiveCommand;}
      void calcTorque(const cnoid::BodyPtr& robot_act, cnoid::Vector6& rootWrench, double dt, const std::vector<cnoid::LinkPtr>& useJOints);
      const std::string& name() const { return name_;}
      const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand() const {return primitiveCommand_;}
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
      std::string name_;
      std::shared_ptr<const primitive_motion_level_tools::PrimitiveState> primitiveCommand_;

      cnoid::Position offset_; // world系
      cnoid::Vector6 dOffsetPrev_; // world系
    };

  protected:
    std::map<std::string, std::shared_ptr<PrimitiveTask> > positionTaskMap_;

    // static functions
    static void calcGravityCompensation(cnoid::BodyPtr& robot_act, cnoid::Vector6& rootWrench, const std::vector<cnoid::LinkPtr>& useJoints);
    static void calcQRefPDTorque(cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_act, const std::vector<cnoid::LinkPtr>& useJoints);
    static void getPrimitiveCommand(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >& positionTaskMap);
  };
}

#endif
