#include "WholeBodyTorqueControllerService_impl.h"
#include "WholeBodyTorqueController.h"

WholeBodyTorqueControllerService_impl::WholeBodyTorqueControllerService_impl()
{
}

WholeBodyTorqueControllerService_impl::~WholeBodyTorqueControllerService_impl()
{
}

void WholeBodyTorqueControllerService_impl::setComp(WholeBodyTorqueController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean WholeBodyTorqueControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean WholeBodyTorqueControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void WholeBodyTorqueControllerService_impl::setParams(const whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void WholeBodyTorqueControllerService_impl::getParams(whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam_out i_param)
{
  i_param = new whole_body_torque_controller::WholeBodyTorqueControllerService::WholeBodyTorqueControllerParam();
  comp_->getParams(*i_param);
};

