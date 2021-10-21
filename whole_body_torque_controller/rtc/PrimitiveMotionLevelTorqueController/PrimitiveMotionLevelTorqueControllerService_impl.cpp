#include "PrimitiveMotionLevelTorqueControllerService_impl.h"
#include "PrimitiveMotionLevelTorqueController.h"

PrimitiveMotionLevelTorqueControllerService_impl::PrimitiveMotionLevelTorqueControllerService_impl()
{
}

PrimitiveMotionLevelTorqueControllerService_impl::~PrimitiveMotionLevelTorqueControllerService_impl()
{
}

void PrimitiveMotionLevelTorqueControllerService_impl::setComp(PrimitiveMotionLevelTorqueController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean PrimitiveMotionLevelTorqueControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean PrimitiveMotionLevelTorqueControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void PrimitiveMotionLevelTorqueControllerService_impl::setParams(const whole_body_torque_controller::PrimitiveMotionLevelTorqueControllerService::PrimitiveMotionLevelTorqueControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void PrimitiveMotionLevelTorqueControllerService_impl::getParams(whole_body_torque_controller::PrimitiveMotionLevelTorqueControllerService::PrimitiveMotionLevelTorqueControllerParam_out i_param)
{
  i_param = new whole_body_torque_controller::PrimitiveMotionLevelTorqueControllerService::PrimitiveMotionLevelTorqueControllerParam();
  comp_->getParams(*i_param);
};

