#ifndef WholeBodyMasterSlaveChoreonoid_H
#define WholeBodyMasterSlaveChoreonoid_H

#include <memory>
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

#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/IIRFilter.h>

#include <whole_body_master_slave_choreonoid/idl/EndEffectorState.hh>
#include "WholeBodyMasterSlaveChoreonoidService_impl.h"
#include "wbms_core.h"

//#define USE_DEBUG_PORT

class WholeBodyMasterSlaveChoreonoid : public RTC::DataFlowComponentBase{
public:
  WholeBodyMasterSlaveChoreonoid(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startWholeBodyMasterSlave();
  bool stopWholeBodyMasterSlave();
  bool pauseWholeBodyMasterSlave();
  bool resumeWholeBodyMasterSlave();
  bool setParams(const WholeBodyMasterSlaveChoreonoidIdl::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param);
  bool getParams(WholeBodyMasterSlaveChoreonoidIdl::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param);

  class Ports {
  public:
    Ports();

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedPoint3D m_basePosRef_;
    RTC::InPort<RTC::TimedPoint3D> m_basePosRefIn_;
    RTC::TimedOrientation3D m_baseRpyRef_;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyRefIn_;
    WholeBodyMasterSlaveChoreonoidIdl::TimedEndEffectorStateSeq m_eeStateRef_;
    RTC::InPort <WholeBodyMasterSlaveChoreonoidIdl::TimedEndEffectorStateSeq> m_eeStateRefIn_;

    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedOrientation3D m_imuAct_;
    RTC::InPort<RTC::TimedOrientation3D> m_imuActIn_;
    std::map<std::string, RTC::TimedDoubleSeq> m_fsensorAct_;
    std::map<std::string, std::shared_ptr<RTC::InPort <RTC::TimedDoubleSeq> > > m_fsensorActIn_;

    RTC::TimedDoubleSeq m_qCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qComOut_;
    RTC::TimedPoint3D m_basePosCom_;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosComOut_;
    RTC::TimedOrientation3D m_baseRpyCom_;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyComOut_;

    WholeBodyMasterSlaveChoreonoidIdl::TimedEndEffectorStateSeq m_eeStateAct_;
    RTC::OutPort<WholeBodyMasterSlaveChoreonoidIdl::TimedEndEffectorStateSeq> m_eeStateActOut_;

    RTC::Time m_delayCheckPacket_;
    RTC::InPort<RTC::Time> m_delayCheckPacketInboundIn_;
    RTC::OutPort<RTC::Time> m_delayCheckPacketOutboundOut_;

    WholeBodyMasterSlaveChoreonoidService_impl m_service0_;
    RTC::CorbaPort m_WholeBodyMasterSlaveChoreonoidServicePort_;
  };

  class ControlMode{
  public:
    enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_WBMS, MODE_WBMS, MODE_SYNC_TO_IDLE};
  private:
    mode_enum current, previous, next;
  public:
    ControlMode(){ current = previous = next = MODE_IDLE;}
    ~ControlMode(){}
    bool setNextMode(const mode_enum _request){
      switch(_request){
      case MODE_SYNC_TO_WBMS:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_WBMS; return true; }else{ return false; }
      case MODE_WBMS:
        if(current == MODE_SYNC_TO_WBMS){ next = MODE_WBMS; return true; }else{ return false; }
      case MODE_SYNC_TO_IDLE:
        if(current == MODE_WBMS){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      case MODE_IDLE:
        if(current == MODE_SYNC_TO_IDLE ){ next = MODE_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(){ previous = current; current = next; }
    mode_enum now(){ return current; }
    mode_enum pre(){ return previous; }
    bool isRunning(){ return (current==MODE_SYNC_TO_WBMS) || (current==MODE_WBMS) || (current==MODE_SYNC_TO_IDLE) ;}
    bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_WBMS) ;}
  };

protected:

  unsigned int m_debugLevel_;
  unsigned int loop;
  double m_dt_;

  Ports port_;
  ControlMode mode_;

  cnoid::BodyPtr m_robot_ref_; // reference (without ee. q, basepos and baserpy only)
  cnoid::BodyPtr m_robot_act_; // actual
  cnoid::BodyPtr m_robot_com_; // command

  std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > outputRatioInterpolator_;

  std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::VectorX> > outputSmoothingInterpolator_;
  cnoid::VectorX avg_q_vel_, avg_q_acc_;

  cpp_filters::IIRFilter<cnoid::Vector3> staticBalancingCOMOffsetFilter_;




  std::map<std::string, IKConstraint> ee_ikc_map; // e.g. feet hands head com
  std::map<std::string, size_t> contact_states_index_map;

  HumanPose raw_pose;

  boost::shared_ptr<WBMSCore> wbms;
  boost::shared_ptr<CapsuleCollisionChecker> sccp;

  hrp::Vector3 torso_rot_rmc;

  hrp::Vector3 rel_act_cp;
  hrp::Vector3 rel_act_zmp;
  struct timespec startT, endT;
  std::string time_report_str;

  hrp::Vector3 static_balancing_com_offset;

  std::vector<std::string> ee_names;
  std::vector<std::string> tgt_names;

  RTC::ReturnCode_t setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop);
  void solveFullbodyIK(HumanPose& ref);
  void preProcessForWholeBodyMasterSlave();
  void processWholeBodyMasterSlave(const HumanPose& ref);
  void smoothingJointAngles(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe);
  bool isOptionalDataContact (const std::string& ee_name) { return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false; }
  void addTimeReport(const std::string& prefix){
    clock_gettime(CLOCK_REALTIME, &endT);
    std::stringstream ss;
    ss << prefix << "= " << std::fixed <<std::setprecision(2) << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-6) << " [ms] / ";
    time_report_str += ss.str();
    clock_gettime(CLOCK_REALTIME, &startT);
  }
};


extern "C"
{
  void WholeBodyMasterSlaveChoreonoidInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlaveChoreonoid_H
