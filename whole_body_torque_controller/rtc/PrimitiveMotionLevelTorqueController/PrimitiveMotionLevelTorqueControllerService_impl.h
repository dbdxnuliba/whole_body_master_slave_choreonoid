// -*-C++-*-
#ifndef PrimitiveMotionLevelTorqueControllerSERVICESVC_IMPL_H
#define PrimitiveMotionLevelTorqueControllerSERVICESVC_IMPL_H

#include "whole_body_torque_controller/idl/PrimitiveMotionLevelTorqueControllerService.hh"

class PrimitiveMotionLevelTorqueController;

class PrimitiveMotionLevelTorqueControllerService_impl
  : public virtual POA_whole_body_torque_controller::PrimitiveMotionLevelTorqueControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  PrimitiveMotionLevelTorqueControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~PrimitiveMotionLevelTorqueControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_torque_controller::PrimitiveMotionLevelTorqueControllerService::PrimitiveMotionLevelTorqueControllerParam& i_param);
  void getParams(whole_body_torque_controller::PrimitiveMotionLevelTorqueControllerService::PrimitiveMotionLevelTorqueControllerParam_out i_param);
  //
  void setComp(PrimitiveMotionLevelTorqueController *i_comp);
private:
  PrimitiveMotionLevelTorqueController *comp_;
};

#endif
