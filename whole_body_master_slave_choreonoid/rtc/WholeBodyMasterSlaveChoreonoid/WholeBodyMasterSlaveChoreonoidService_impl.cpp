#include "WholeBodyMasterSlaveChoreonoidService_impl.h"
#include "WholeBodyMasterSlaveChoreonoid.h"

void WholeBodyMasterSlaveChoreonoidService_impl::wholebodymasterslave(WholeBodyMasterSlaveChoreonoid *i_wholebodymasterslave)
{
  m_wholebodymasterslave = i_wholebodymasterslave;
}

CORBA::Boolean WholeBodyMasterSlaveChoreonoidService_impl::startWholeBodyMasterSlave()
{
    return m_wholebodymasterslave->startWholeBodyMasterSlave();
};

CORBA::Boolean WholeBodyMasterSlaveChoreonoidService_impl::stopWholeBodyMasterSlave()
{
    return m_wholebodymasterslave->stopWholeBodyMasterSlave();
};

void WholeBodyMasterSlaveChoreonoidService_impl::setParams(const WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param)
{
  m_wholebodymasterslave->setParams(i_param);
};

void WholeBodyMasterSlaveChoreonoidService_impl::getParams(WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam_out i_param)
{
  i_param = new WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam();
  m_wholebodymasterslavechoreonoid->getParam(*i_param);
};
