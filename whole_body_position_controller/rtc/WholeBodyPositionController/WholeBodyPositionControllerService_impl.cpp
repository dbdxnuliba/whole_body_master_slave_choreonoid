#include "WholeBodyPositionControllerService_impl.h"
#include "WholeBodyPositionController.h"

WholeBodyPositionControllerService_impl::WholeBodyPositionControllerService_impl()
{
}

WholeBodyPositionControllerService_impl::~WholeBodyPositionControllerService_impl()
{
}

void WholeBodyPositionControllerService_impl::setComp(WholeBodyPositionController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean WholeBodyPositionControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean WholeBodyPositionControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void WholeBodyPositionControllerService_impl::setParams(const whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void WholeBodyPositionControllerService_impl::getParams(whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam_out i_param)
{
  i_param = new whole_body_position_controller::WholeBodyPositionControllerService::WholeBodyPositionControllerParam();
  comp_->getParams(*i_param);
};

