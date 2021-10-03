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

#include <whole_body_master_slave_choreonoid/idl/PrimitiveState.hh>
#include "PrimitiveMotionLevelControllerService_impl.h"
#include "PrimitiveCommand.h"
#include "PositionController.h"

class PrimitiveMotionLevelController : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_qRefIn_("qRefIn", m_qRef_),// from sh
      m_basePosRefIn_("basePosRefIn", m_basePosRef_),// from sh
      m_baseRpyRefIn_("baseRpyRefIn", m_baseRpyRef_),// from sh
      m_primitiveCommandRefIn_("primitiveCommandRefIn", m_primitiveCommandRef_),

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
    WholeBodyMasterSlaveChoreonoidIdl::TimedPrimitiveStateSeq m_primitiveCommandRef_;
    RTC::InPort <WholeBodyMasterSlaveChoreonoidIdl::TimedPrimitiveStateSeq> m_primitiveCommandRefIn_;

    RTC::TimedDoubleSeq m_qCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qComOut_;
    RTC::TimedPoint3D m_basePosCom_;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosComOut_;
    RTC::TimedOrientation3D m_baseRpyCom_;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyComOut_;
    RTC::TimedDoubleSeq m_baseTformCom_; // for HrpsysSeqStateROSBridge
    RTC::OutPort<RTC::TimedDoubleSeq> m_baseTformComOut_; // for HrpsysSeqStateROSBridge
    WholeBodyMasterSlaveChoreonoidIdl::TimedPrimitiveStateSeq m_primitiveCommandCom_;
    RTC::OutPort <WholeBodyMasterSlaveChoreonoidIdl::TimedPrimitiveStateSeq> m_primitiveCommandComOut_;

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
  bool setParams(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);
  bool getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);

protected:

  unsigned int m_debugLevel_;
  unsigned int loop_;
  double m_dt_;

  Ports ports_;
  ControlMode mode_;

  cnoid::BodyPtr m_robot_ref_; // reference (q, basepos and baserpy only)
  cnoid::BodyPtr m_robot_com_; // command

  std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > outputRatioInterpolator_;

  // 1. 受け取ったprimitive motion level 指令
  std::map<std::string, std::shared_ptr<PrimitiveMotionLevel::PrimitiveCommand> > primitiveCommandMap_;

  // 2. primitiveCommandMap_を受け取り、m_robot_comを計算する
  PrimitiveMotionLevel::PositionController positionController_;

};


extern "C"
{
  void PrimitiveMotionLevelControllerInit(RTC::Manager* manager);
};

#endif // PrimitiveMotionLevelController_H
