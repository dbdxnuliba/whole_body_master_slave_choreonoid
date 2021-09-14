// -*-C++-*-
#ifndef PrimitiveMotionLevelControllerSERVICESVC_IMPL_H
#define PrimitiveMotionLevelControllerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/PrimitiveMotionLevelControllerService.hh"

class PrimitiveMotionLevelController;

class PrimitiveMotionLevelControllerService_impl
  : public virtual POA_WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  PrimitiveMotionLevelControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~PrimitiveMotionLevelControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);
  void getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam_out i_param);
  //
  void setComp(PrimitiveMotionLevelController *i_comp);
private:
  PrimitiveMotionLevelController *comp_;
};

#endif
