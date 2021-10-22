#ifndef WholeBodyTorqueController_H
#define WholeBodyTorqueController_H

#include <memory>
#include <map>
#include <time.h>
#include <mutex>

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

#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include <collision_checker_msgs/idl/Collision.hh>
#include "WholeBodyTorqueControllerService_impl.h"
#include "PrimitiveCommand.h"
#include "TorqueController.h"
#include "Collision.h"

class WholeBodyTorqueController : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_qRefIn_("qRefIn", m_qRef_),
      m_pgainPercentageRefIn_("pgainPercentageRefIn", m_pgainPercentageRef_),
      m_dgainPercentageRefIn_("dgainPercentageRefIn", m_dgainPercentageRef_),
      m_basePosRefIn_("basePosRefIn", m_basePosRef_),
      m_baseRpyRefIn_("baseRpyRefIn", m_baseRpyRef_),
      m_primitiveCommandRefIn_("primitiveCommandRefIn", m_primitiveCommandRef_),

      m_qActIn_("qActIn", m_qAct_),
      m_imuActIn_("imuActIn", m_imuAct_),// required if rootlink is not fixed joint
      m_collisionActIn_("collisionActIn", m_collisionAct_),

      m_tauComOut_("tauComOut", m_qCom_),
      m_qComOut_("qComOut", m_qCom_),
      m_pgainPercentageComOut_("pgainPercentageComOut", m_pgainPercentageCom_),
      m_dgainPercentageComOut_("dgainPercentageComOut", m_dgainPercentageCom_),
      m_basePosActOut_("basePosActOut", m_basePosAct_),
      m_baseRpyActOut_("baseRpyActOut", m_baseRpyAct_),
      m_basePoseActOut_("basePoseActOut", m_basePoseAct_),
      m_baseTformActOut_("baseTformActOut", m_baseTformAct_),

      m_WholeBodyTorqueControllerServicePort_("WholeBodyTorqueControllerService") {
    }

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedDoubleSeq m_pgainPercentageRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_pgainPercentageRefIn_;
    RTC::TimedDoubleSeq m_dgainPercentageRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dgainPercentageRefIn_;
    RTC::TimedPoint3D m_basePosRef_;
    RTC::InPort<RTC::TimedPoint3D> m_basePosRefIn_;
    RTC::TimedOrientation3D m_baseRpyRef_;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyRefIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandRefIn_;

    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedOrientation3D m_imuAct_;
    RTC::InPort<RTC::TimedOrientation3D> m_imuActIn_;
    collision_checker_msgs::TimedCollisionSeq m_collisionAct_;
    RTC::InPort <collision_checker_msgs::TimedCollisionSeq> m_collisionActIn_;

    RTC::TimedDoubleSeq m_tauCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tauComOut_;
    RTC::TimedDoubleSeq m_qCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qComOut_;
    RTC::TimedDoubleSeq m_pgainPercentageCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_pgainPercentageComOut_;
    RTC::TimedDoubleSeq m_dgainPercentageCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_dgainPercentageComOut_;
    RTC::TimedPoint3D m_basePosAct_;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosActOut_;
    RTC::TimedOrientation3D m_baseRpyAct_;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyActOut_;
    RTC::TimedPose3D m_basePoseAct_;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseActOut_;
    RTC::TimedDoubleSeq m_baseTformAct_; // for HrpsysSeqStateROSBridge
    RTC::OutPort<RTC::TimedDoubleSeq> m_baseTformActOut_; // for HrpsysSeqStateROSBridge

    WholeBodyTorqueControllerService_impl m_service0_;
    RTC::CorbaPort m_WholeBodyTorqueControllerServicePort_;
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

  class OutputInterpolators {
  public:
    std::unordered_map<cnoid::LinkPtr, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > > qOffsetInterpolatorMap_; // controlで上書きしない関節について、refereceのqに加えるoffset
    std::unordered_map<cnoid::LinkPtr, std::shared_ptr<cpp_filters::IIRFilter<double> > > qComFilterMap_; double qComFilter_hz_; // controlで上書きする関節について、actualの角度にこのフィルタを適用した値を指令する
    std::unordered_map<cnoid::LinkPtr, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > > outputRatioInterpolatorMap_; // gain, torque用. 0ならreference
  };

public:
  WholeBodyTorqueController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam& i_param);
  bool getParams(whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam& i_param);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  std::vector<cnoid::LinkPtr> useJoints_; // controlで上書きする関節(root含む)のリスト
  OutputInterpolators outputInterpolators_;

  cnoid::BodyPtr m_robot_ref_; // reference (q, basepos and baserpy only)
  cnoid::BodyPtr m_robot_act_; // actual
  cnoid::BodyPtr m_robot_com_; // command (q,tau only)

  // 0. robotの設定
  std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTablesMap_;

  // 1. portから受け取ったprimitive motion level 指令など
  std::map<std::string, std::shared_ptr<WholeBodyTorque::PrimitiveCommand> > primitiveCommandMap_;
  std::vector<std::shared_ptr<WholeBodyTorque::Collision> > collisions_;

  // 2. primitiveCommandMap_を受け取り、m_robot_comを計算する
  WholeBodyTorque::TorqueController torqueController_;

  // static functions
  static void readPorts(const std::string& instance_name, WholeBodyTorqueController::Ports& port);
  static void calcReferenceRobot(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, cnoid::BodyPtr& robot);
  static void calcActualRobot(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, cnoid::BodyPtr& robot);
  static void getPrimitiveCommand(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, double dt, std::map<std::string, std::shared_ptr<WholeBodyTorque::PrimitiveCommand> >& primitiveCommandMap);
  static void getCollision(const std::string& instance_name, const WholeBodyTorqueController::Ports& port, std::vector<std::shared_ptr<WholeBodyTorque::Collision> >& collisions);
  static void processModeTransition(const std::string& instance_name, WholeBodyTorqueController::ControlMode& mode, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com, WholeBodyTorqueController::OutputInterpolators& outputInterpolators, const std::vector<cnoid::LinkPtr>& useJoints);
  static void preProcessForControl(const std::string& instance_name, WholeBodyTorque::TorqueController& torqueController);
  static void calcq(const std::string& instance_name, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_act, cnoid::BodyPtr& robot_com, WholeBodyTorqueController::OutputInterpolators& outputInterpolators, double dt, const std::vector<cnoid::LinkPtr>& useJoints=std::vector<cnoid::LinkPtr>());
  static void calcOutputPorts(const std::string& instance_name, WholeBodyTorqueController::Ports& port, const cnoid::BodyPtr& robot_com, const cnoid::BodyPtr& robot_act, WholeBodyTorqueController::OutputInterpolators& outputInterpolators, double dt);
  static void enableJoint(const cnoid::LinkPtr& joint_com, WholeBodyTorqueController::OutputInterpolators& outputInterpolators);
  static void disableJoint(const cnoid::LinkPtr& joint_com, const cnoid::BodyPtr& robot_ref, WholeBodyTorqueController::OutputInterpolators& outputInterpolators);
};


extern "C"
{
  void WholeBodyTorqueControllerInit(RTC::Manager* manager);
};

#endif // WholeBodyTorqueController_H
