#ifndef PrimitiveMotionLevelController_H
#define PrimitiveMotionLevelController_H

#include <memory>
#include <map>
#include <time.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>

#include <cnoid/Body>

#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/IIRFilter.h>
#include <joint_limit_table/JointLimitTable.h>

#include <collision_checker_msgs/idl/Collision.hh>
#include <whole_body_master_slave_choreonoid/idl/PrimitiveStateIdl.hh>
#include "PrimitiveMotionLevelControllerService_impl.h"
#include "PrimitiveCommand.h"
#include "PositionController.h"
#include "Collision.h"

class PrimitiveMotionLevelController : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_qRefIn_("qRefIn", m_qRef_),// from sh
      m_basePosRefIn_("basePosRefIn", m_basePosRef_),// from sh
      m_baseRpyRefIn_("baseRpyRefIn", m_baseRpyRef_),// from sh
      m_primitiveCommandRefIn_("primitiveCommandRefIn", m_primitiveCommandRef_),
      m_collisionComIn_("collisionComIn", m_collisionCom_),

      m_qComOut_("qComOut", m_qCom_),
      m_basePosComOut_("basePosComOut", m_basePosCom_),
      m_baseRpyComOut_("baseRpyComOut", m_baseRpyCom_),
      m_baseTformComOut_("baseTformComOut", m_baseTformCom_),
      m_primitiveCommandComOut_("primitiveCommandComOut", m_primitiveCommandCom_),

      m_PrimitiveMotionLevelControllerServicePort_("PrimitiveMotionLevelControllerService") {
    }

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedPoint3D m_basePosRef_;
    RTC::InPort<RTC::TimedPoint3D> m_basePosRefIn_;
    RTC::TimedOrientation3D m_baseRpyRef_;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyRefIn_;
    whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq m_primitiveCommandRef_;
    RTC::InPort <whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq> m_primitiveCommandRefIn_;
    collision_checker_msgs::TimedCollisionSeq m_collisionCom_;
    RTC::InPort <collision_checker_msgs::TimedCollisionSeq> m_collisionComIn_;

    RTC::TimedDoubleSeq m_qCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qComOut_;
    RTC::TimedPoint3D m_basePosCom_;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosComOut_;
    RTC::TimedOrientation3D m_baseRpyCom_;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyComOut_;
    RTC::TimedDoubleSeq m_baseTformCom_; // for HrpsysSeqStateROSBridge
    RTC::OutPort<RTC::TimedDoubleSeq> m_baseTformComOut_; // for HrpsysSeqStateROSBridge
    whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq m_primitiveCommandCom_;
    RTC::OutPort <whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq> m_primitiveCommandComOut_;

    PrimitiveMotionLevelControllerService_impl m_service0_;
    RTC::CorbaPort m_PrimitiveMotionLevelControllerServicePort_;
  };

  class ControlMode{
  public:
    enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_CONTROL, MODE_CONTROL, MODE_SYNC_TO_IDLE};
  private:
    mode_enum current, previous, next;
  public:
    ControlMode(){ current = previous = next = MODE_IDLE;}
    ~ControlMode(){}
    bool setNextMode(const mode_enum _request){
      switch(_request){
      case MODE_SYNC_TO_CONTROL:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_CONTROL; return true; }else{ return false; }
      case MODE_CONTROL:
        if(current == MODE_SYNC_TO_CONTROL){ next = MODE_CONTROL; return true; }else{ return false; }
      case MODE_SYNC_TO_IDLE:
        if(current == MODE_CONTROL){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      case MODE_IDLE:
        if(current == MODE_SYNC_TO_IDLE ){ next = MODE_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(){ previous = current; current = next; }
    mode_enum now(){ return current; }
    mode_enum pre(){ return previous; }
    bool isRunning(){ return (current==MODE_SYNC_TO_CONTROL) || (current==MODE_CONTROL) || (current==MODE_SYNC_TO_IDLE) ;}
    bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_CONTROL) ;}
  };


public:
  PrimitiveMotionLevelController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_master_slave_choreonoid::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);
  bool getParams(whole_body_master_slave_choreonoid::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);

protected:

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;
  std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > outputRatioInterpolator_;

  cnoid::BodyPtr m_robot_ref_; // reference (q, basepos and baserpy only)
  cnoid::BodyPtr m_robot_com_; // command<


  // 0. robotの設定
  std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTablesMap_;

  // 1. portから受け取ったprimitive motion level 指令など
  std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> > primitiveCommandMap_;
  std::vector<std::shared_ptr<PrimitiveMotionLevel::Collision> > collisions_;

  // 2. primitiveCommandMap_を受け取り、m_robot_comを計算する
  PrimitiveMotionLevel::PositionController positionController_;

  // static functions
  static void readPorts(const std::string& instance_name, PrimitiveMotionLevelController::Ports& port);
  static void calcReferenceRobot(const std::string& instance_name, const PrimitiveMotionLevelController::Ports& port, cnoid::BodyPtr& robot);
  static void getPrimitiveCommand(const std::string& instance_name, const PrimitiveMotionLevelController::Ports& port, double dt, std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap);
  static void getCollision(const std::string& instance_name, const PrimitiveMotionLevelController::Ports& port, std::vector<std::shared_ptr<PrimitiveMotionLevel::Collision> >& collisions);
  static void processModeTransition(const std::string& instance_name, PrimitiveMotionLevelController::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> >& outputRatioInterpolator, const double dt);
  static void preProcessForControl(const std::string& instance_name, PrimitiveMotionLevel::PositionController& positionController);
  static void passThrough(const std::string& instance_name, const cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_com);
  static void calcOutputPorts(const std::string& instance_name, PrimitiveMotionLevelController::Ports& port, double output_ratio, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com, std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> >& primitiveCommandMap);
};


extern "C"
{
  void PrimitiveMotionLevelControllerInit(RTC::Manager* manager);
};

#endif // PrimitiveMotionLevelController_H
