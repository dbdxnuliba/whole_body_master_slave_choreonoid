#ifndef PrimitiveStateROSBridge_H
#define PrimitiveStateROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <whole_body_master_slave_choreonoid/idl/PrimitiveStateIdl.hh>

#include <ros/ros.h>
#include <whole_body_master_slave_choreonoid/PrimitiveStateArray.h>

#include <urdf/model.h>
#include <cnoid/Body>

class PrimitiveStateROSBridge : public RTC::DataFlowComponentBase{
protected:
  std::shared_ptr<urdf::Model> robot_urdf_;
  cnoid::BodyPtr robot_vrml_;

  whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq m_primitiveCommandRTM_;
  RTC::InPort <whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq> m_primitiveCommandRTMIn_;
  ros::Publisher pub_;

  ros::Subscriber sub_;
  whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq m_primitiveCommandROS_;
  RTC::OutPort <whole_body_master_slave_choreonoid::TimedPrimitiveStateIdlSeq> m_primitiveCommandROSOut_;
public:
  PrimitiveStateROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void topicCallback(const whole_body_master_slave_choreonoid::PrimitiveStateArray::ConstPtr& msg);
};


extern "C"
{
  void PrimitiveStateROSBridgeInit(RTC::Manager* manager);
};

#endif // PrimitiveStateROSBridge_H
