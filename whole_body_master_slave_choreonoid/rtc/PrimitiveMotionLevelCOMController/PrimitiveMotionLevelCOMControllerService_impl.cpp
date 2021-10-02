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

void PrimitiveMotionLevelCOMControllerService_impl::setParams(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void PrimitiveMotionLevelCOMControllerService_impl::getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam_out i_param)
{
  i_param = new WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelCOMControllerService::PrimitiveMotionLevelCOMControllerParam();
  comp_->getParams(*i_param);
};

