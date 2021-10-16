// -*-C++-*-
#ifndef CFRControllerSERVICESVC_IMPL_H
#define CFRControllerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/CFRControllerService.hh"

class CFRController;

class CFRControllerService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::CFRControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  CFRControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~CFRControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam_out i_param);
  //
  void setComp(CFRController *i_comp);
private:
  CFRController *comp_;
};

#endif
