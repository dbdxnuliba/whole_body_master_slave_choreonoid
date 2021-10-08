// -*-C++-*-
#ifndef PrimitiveMotionLevelControllerSERVICESVC_IMPL_H
#define PrimitiveMotionLevelControllerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/PrimitiveMotionLevelControllerService.hh"

class PrimitiveMotionLevelController;

class PrimitiveMotionLevelControllerService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::PrimitiveMotionLevelControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  PrimitiveMotionLevelControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~PrimitiveMotionLevelControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_master_slave_choreonoid::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam_out i_param);
  //
  void setComp(PrimitiveMotionLevelController *i_comp);
private:
  PrimitiveMotionLevelController *comp_;
};

#endif
