#include "WholeBodyMasterSlaveChoreonoidService_impl.h"
#include "WholeBodyMasterSlaveChoreonoid.h"

void WholeBodyMasterSlaveChoreonoidService_impl::wholebodymasterslave(WholeBodyMasterSlaveChoreonoid *i_wholebodymasterslave)
{
  m_wholebodymasterslavechoreonoid = i_wholebodymasterslavechoreonoid;
}

CORBA::Boolean WholeBodyMasterSlaveChoreonoidService_impl::startWholeBodyMasterSlave()
{
    return m_wholebodymasterslavechoreonoid->startWholeBodyMasterSlave();
};

CORBA::Boolean WholeBodyMasterSlaveChoreonoidService_impl::stopWholeBodyMasterSlave()
{
    return m_wholebodymasterslavechoreonoid->stopWholeBodyMasterSlave();
};

void WholeBodyMasterSlaveChoreonoidService_impl::setParams(const WholeBodyMasterSlaveChoreonoidIdl::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam& i_param)
{
  m_wholebodymasterslavechoreonoid->setParams(i_param);
};

void WholeBodyMasterSlaveChoreonoidService_impl::getParams(WholeBodyMasterSlaveChoreonoidIdl::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam_out i_param)
{
  i_param = new WholeBodyMasterSlaveChoreonoid::WholeBodyMasterSlaveChoreonoidService::WholeBodyMasterSlaveChoreonoidParam();
  m_wholebodymasterslavechoreonoid->getParam(*i_param);
};

