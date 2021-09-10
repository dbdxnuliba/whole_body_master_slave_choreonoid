#include "PrimitiveMotionLevelControllerService_impl.h"
#include "PrimitiveMotionLevelController.h"

void PrimitiveMotionLevelControllerService_impl::wholebodymasterslave(PrimitiveMotionLevelController *i_wholebodymasterslave)
{
  comp_ = i_wholebodymasterslavechoreonoid;
}

CORBA::Boolean PrimitiveMotionLevelControllerService_impl::startWholeBodyMasterSlave()
{
  return comp_->startWholeBodyMasterSlave();
};

CORBA::Boolean PrimitiveMotionLevelControllerService_impl::stopWholeBodyMasterSlave()
{
  return comp_->stopWholeBodyMasterSlave();
};

void PrimitiveMotionLevelControllerService_impl::setParams(const WholeBodyMasterSlaveChorenoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam& i_param)
{
  comp_>setParams(i_param);
};

void PrimitiveMotionLevelControllerService_impl::getParams(WholeBodyMasterSlaveChoreonoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam_out i_param)
{
  i_param = new WholeBodyMasterSlaveChorenoidIdl::PrimitiveMotionLevelControllerService::PrimitiveMotionLevelControllerParam();
  comp_->getParam(*i_param);
};

