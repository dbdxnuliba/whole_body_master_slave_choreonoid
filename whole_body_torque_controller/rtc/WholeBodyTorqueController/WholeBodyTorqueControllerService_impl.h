// -*-C++-*-
#ifndef WholeBodyTorqueControllerSERVICESVC_IMPL_H
#define WholeBodyTorqueControllerSERVICESVC_IMPL_H

#include "whole_body_torque_controller/idl/WholeBodyTorqueControllerService.hh"

class WholeBodyTorqueController;

class WholeBodyTorqueControllerService_impl
  : public virtual POA_whole_body_torque_controller::WholeBodyTorqueControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  WholeBodyTorqueControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~WholeBodyTorqueControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam& i_param);
  void getParams(whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam_out i_param);
  //
  void setComp(WholeBodyTorqueController *i_comp);
private:
  WholeBodyTorqueController *comp_;
};

#endif
