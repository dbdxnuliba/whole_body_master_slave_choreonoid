// -*-C++-*-
#ifndef PrimitiveMotionLevelControllerSERVICESVC_IMPL_H
#define PrimitiveMotionLevelControllerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/PrimitiveMotionLevelControllerService.hh"

class PrimitiveMotionLevelController;

class PrimitiveMotionLevelControllerService_impl
  : public virtual POA_WholeBodyMasterSlaveChorenoidIdl::PrimitiveMotionLevelControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const WholeBodyMasterSlaveChorenoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param);
  void getParams(WholeBodyMasterSlaveChorenoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam_out i_param);
  //
  void wholebodymasterslavechoreonoid(PrimitiveMotionLevelController *i_comp);
private:
  PrimitiveMotionLevelController *comp_;
};

#endif
