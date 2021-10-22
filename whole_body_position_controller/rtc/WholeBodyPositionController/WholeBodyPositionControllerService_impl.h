// -*-C++-*-
#ifndef WholeBodyPositionControllerSERVICESVC_IMPL_H
#define WholeBodyPositionControllerSERVICESVC_IMPL_H

#include "whole_body_position_controller/idl/WholeBodyPositionControllerService.hh"

class WholeBodyPositionController;

class WholeBodyPositionControllerService_impl
  : public virtual POA_whole_body_position_controller::WholeBodyPositionControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  WholeBodyPositionControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~WholeBodyPositionControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam& i_param);
  void getParams(whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam_out i_param);
  //
  void setComp(WholeBodyPositionController *i_comp);
private:
  WholeBodyPositionController *comp_;
};

#endif
