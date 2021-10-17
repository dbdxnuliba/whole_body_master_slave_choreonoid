#!/usr/bin/env python

import sys

# ros modules
import rospy
from geometry_msgs.msg import PolygonStamped, Point

# rtm modules
import OpenRTM_aist
import RTC

module_spec = ["implementation_id", "CFRVisualizer",
               "type_name",         "CFRVisualizer",
               "description",       "CFRVisualizer",
               "version",           "1.0",
               "vendor",            "Naoki-Hiraoka",
               "category",          "example",
               "activity_type",     "DataFlowComponent",
               "max_instance",      "10",
               "language",          "Python",
               "lang_type",         "script",
               ""]

#
# Component
#
class CFRVisualizer(OpenRTM_aist.DataFlowComponentBase):
    pub_ = None
    m_vertices_ = None
    m_verticesIn_ = None

    def __init__ (self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        self.m_vertices_ = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
        self.m_verticesIn_ = OpenRTM_aist.InPort("verticesIn", self.m_vertices_)
        self.registerInPort("verticesIn", self.m_verticesIn_)
        self.pub_ = rospy.Publisher("~cfr", PolygonStamped, queue_size=1)
        return RTC.RTC_OK

    # InPort -> Publisher
    def onExecute(self, ec_id):
        if self.m_verticesIn_.isNew():
            self.m_vertices_ = self.m_verticesIn_.read()
            msg = PolygonStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "/command/odom"
            for i in range(len(self.m_vertices_.data)/2):
                point = Point()
                point.x = self.m_vertices_.data[i*2+0]
                point.y = self.m_vertices_.data[i*2+1]
                point.z = 0.0
                msg.polygon.points.append(point)
            self.pub_.publish(msg)
        return RTC.RTC_OK

def CFRVisualizerInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=module_spec)
    manager.registerFactory(profile,
                            CFRVisualizer,
                            OpenRTM_aist.Delete)
    comp = manager.createComponent("CFRVisualizer?instance_name="+rospy.get_name()[1:])

if __name__ == '__main__':
    rospy.init_node('CFRVisualizer')
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(CFRVisualizerInit)
    mgr.activateManager()
    mgr.runManager()
