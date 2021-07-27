// -*-C++-*-
#ifndef WholeBodyMasterSlaveChoreonoidSERVICESVC_IMPL_H
#define WholeBodyMasterSlaveChoreonoidSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/WholeBodyMasterSlaveChoreonoidService.hh"

class WholeBodyMasterSlaveChoreonoid;

class WholeBodyMasterSlaveChoreonoidService_impl
  : public virtual POA_WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService,
    public virtual PortableServer::RefCountServantBase
{
public:
  CORBA::Boolean startWholeBodyMasterSlave();
  CORBA::Boolean stopWholeBodyMasterSlave();
  void setParams(const WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param);
  void getParams(WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam_out i_param);
  //
  void wholebodymasterslavechoreonoid(WholeBodyMasterSlaveChoreonoid *i_wholebodymasterslavechoreonoid);
private:
  WholeBodyMasterSlaveChoreonoid *m_wholebodymasterslavechoreonoid;
};

#endif
