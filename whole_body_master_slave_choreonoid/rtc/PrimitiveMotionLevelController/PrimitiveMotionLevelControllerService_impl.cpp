#include "PrimitiveMotionLevelControllerService_impl.h"
#include "PrimitiveMotionLevelController.h"

PrimitiveMotionLevelControllerService_impl::PrimitiveMotionLevelControllerService_impl()
{
}

PrimitiveMotionLevelControllerService_impl::~PrimitiveMotionLevelControllerService_impl()
{
}

void PrimitiveMotionLevelControllerService_impl::setComp(PrimitiveMotionLevelController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean PrimitiveMotionLevelControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean PrimitiveMotionLevelControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void PrimitiveMotionLevelControllerService_impl::setParams(const WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void PrimitiveMotionLevelControllerService_impl::getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam_out i_param)
{
  i_param = new WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam();
  comp_->getParams(*i_param);
};

