#ifndef PrimitiveStateROSBridge_H
#define PrimitiveStateROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>

#include <ros/ros.h>
#include <primitive_motion_level_msgs/PrimitiveStateArray.h>

#include <urdf/model.h>
#include <cnoid/Body>

class PrimitiveStateROSBridge : public RTC::DataFlowComponentBase{
protected:
  std::shared_ptr<urdf::Model> robot_urdf_;
  cnoid::BodyPtr robot_vrml_;

  primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandRTM_;
  RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandRTMIn_;
  ros::Publisher pub_;

  ros::Subscriber sub_;
  primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandROS_;
  RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandROSOut_;
public:
  PrimitiveStateROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void topicCallback(const primitive_motion_level_msgs::PrimitiveStateArray::ConstPtr& msg);
};


extern "C"
{
  void PrimitiveStateROSBridgeInit(RTC::Manager* manager);
};

#endif // PrimitiveStateROSBridge_H
