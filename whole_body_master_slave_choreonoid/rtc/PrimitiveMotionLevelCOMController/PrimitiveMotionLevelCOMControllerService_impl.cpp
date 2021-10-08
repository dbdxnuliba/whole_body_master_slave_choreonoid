#include "PrimitiveMotionLevelCOMControllerService_impl.h"
#include "PrimitiveMotionLevelCOMController.h"

PrimitiveMotionLevelCOMControllerService_impl::PrimitiveMotionLevelCOMControllerService_impl()
{
}

PrimitiveMotionLevelCOMControllerService_impl::~PrimitiveMotionLevelCOMControllerService_impl()
{
}

void PrimitiveMotionLevelCOMControllerService_impl::setComp(PrimitiveMotionLevelCOMController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean PrimitiveMotionLevelCOMControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean PrimitiveMotionLevelCOMControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void PrimitiveMotionLevelCOMControllerService_impl::setParams(const whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void PrimitiveMotionLevelCOMControllerService_impl::getParams(whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam_out i_param)
{
  i_param = new whole_body_master_slave_choreonoid::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam();
  comp_->getParams(*i_param);
};

