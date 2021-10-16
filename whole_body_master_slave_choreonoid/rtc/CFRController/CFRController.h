#ifndef CFRController_H
#define CFRController_H

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

#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include "CFRControllerService_impl.h"
#include "PrimitiveCommand.h"

class CFRController : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_primitiveCommandRefIn_("primitiveCommandRefIn", m_primitiveCommandRef_),

      m_primitiveCommandComOut_("primitiveCommandComOut", m_primitiveCommandCom_),

      m_CFRControllerServicePort_("CFRControllerService") {
    }

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandRefIn_;

    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandComOut_;

    CFRControllerService_impl m_service0_;
    RTC::CorbaPort m_CFRControllerServicePort_;
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
  CFRController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam& i_param);
  bool getParams(whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam& i_param);

protected:

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  // 1. portから受け取ったprimitive motion level 指令など
  std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> > primitiveCommandMap_;

  // static functions
  static void readPorts(const std::string& instance_name, CFRController::Ports& port);
  static void getPrimitiveCommand(const std::string& instance_name, const CFRController::Ports& port, double dt, std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> >& primitiveCommandMap);
  static void processModeTransition(const std::string& instance_name, CFRController::ControlMode& mode);
  static void preProcessForControl(const std::string& instance_name);
  static void calcOutputPorts(const std::string& instance_name, CFRController::Ports& port, std::map<std::string, std::shared_ptr<CFR::PrimitiveCommand> >& primitiveCommandMap);
};


extern "C"
{
  void CFRControllerInit(RTC::Manager* manager);
};

#endif // CFRController_H
