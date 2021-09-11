#ifndef PrimitiveMotionLevelController_PositionControl_H
#define PrimitiveMotionLevelController_PositionControl_H

#include <cnoid/Body>
#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include "PrimitiveCommand.h"

namespace PrimitiveMotionLevel {

  class PositionController {
  public:
    void reset();
    void control(const std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap, // primitive motion level target
                 const cnoid::BodyPtr& robot_ref, // command level target
                 const cnoid::BodyPtr& robot_act, //actual
                 cnoid::BodyPtr& robot_com, //output
                 const bool& frameWarped // if true, not calculate velocity
                 );

    class PositionTask {
    public:
      PositionTask(const std::string& name) : name_(name) {}
      void updateFromPrimitiveCommand(const std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand>& primitiveCommand) {primitiveCommand_ = primitiveCommand;}
      const std::string& name() const { return name_;}
      const std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand>& primitiveCommand() const {return primitiveCommand_;}
    protected:
      std::string name_;
      std::shared_ptr<const PrimitiveMotionLevel::PrimitiveCommand> primitiveCommand_;
    };
  protected:
    std::map<std::string, std::shared_ptr<PositionTask> > positionTaskMap_;
  };
}

#endif
