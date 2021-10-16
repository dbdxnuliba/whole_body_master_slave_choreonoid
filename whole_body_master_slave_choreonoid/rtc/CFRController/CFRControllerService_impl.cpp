#include "CFRControllerService_impl.h"
#include "CFRController.h"

CFRControllerService_impl::CFRControllerService_impl()
{
}

CFRControllerService_impl::~CFRControllerService_impl()
{
}

void CFRControllerService_impl::setComp(CFRController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean CFRControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean CFRControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void CFRControllerService_impl::setParams(const whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void CFRControllerService_impl::getParams(whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam_out i_param)
{
  i_param = whole_body_master_slave_choreonoid::CFRControllerService::CFRControllerParam();
  comp_->getParams(i_param);
};

