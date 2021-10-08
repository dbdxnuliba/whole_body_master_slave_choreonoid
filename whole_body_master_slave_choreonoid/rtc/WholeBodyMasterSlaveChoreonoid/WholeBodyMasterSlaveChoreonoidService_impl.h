// -*-C++-*-
#ifndef WholeBodyMasterSlaveChoreonoidSERVICESVC_IMPL_H
#define WholeBodyMasterSlaveChoreonoidSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/WholeBodyMasterSlaveChoreonoidService.hh"

class WholeBodyMasterSlaveChoreonoid;

class WholeBodyMasterSlaveChoreonoidService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::WholeBodyMasterSlaveChoreonoidService,
    public virtual PortableServer::RefCountServantBase
{
public:
  CORBA::Boolean startWholeBodyMasterSlave();
  CORBA::Boolean stopWholeBodyMasterSlave();
  void setParams(const whole_body_master_slave_choreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam_out i_param);
  //
  void wholebodymasterslavechoreonoid(WholeBodyMasterSlaveChoreonoid *i_wholebodymasterslavechoreonoid);
private:
  WholeBodyMasterSlaveChoreonoid *m_wholebodymasterslavechoreonoid;
};

#endif
