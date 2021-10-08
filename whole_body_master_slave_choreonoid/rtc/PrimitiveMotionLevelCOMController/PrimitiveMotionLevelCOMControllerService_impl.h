// -*-C++-*-
#ifndef PrimitiveMotionLevelCOMControllerSERVICESVC_IMPL_H
#define PrimitiveMotionLevelCOMControllerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/PrimitiveMotionLevelCOMControllerService.hh"

class PrimitiveMotionLevelCOMController;

class PrimitiveMotionLevelCOMControllerService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  PrimitiveMotionLevelCOMControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~PrimitiveMotionLevelCOMControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam_out i_param);
  //
  void setComp(PrimitiveMotionLevelCOMController *i_comp);
private:
  PrimitiveMotionLevelCOMController *comp_;
};

#endif
