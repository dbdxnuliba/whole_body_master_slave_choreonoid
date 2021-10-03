#!/usr/bin/env python

# rosparams
#   end_effectors:
#     - name: <name>
#       parent_link_name: <linkname for URDF>
#       local_pose: [x, y, z, x, y, z, w]
#       M: [x ,y, z, rx, ry, rz] # default 0
#       D: [x ,y, z, rx, ry, rz] # default 0
#       K: [x ,y, z, rx, ry, rz] # default 0
#       wrench_gain: [x ,y, z, rx, ry, rz] # default 0
#       support_com: <bool> #default false
#       time: <double> # default 0.1

# publish
#   ~command: whole_body_master_slave_choreonoid/PrimitiveStateArray

# server
#   ~activate: std_srvs/SetBool

import copy

import rospy

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import tf
from tf import TransformListener, transformations
from whole_body_master_slave_choreonoid.msg import PrimitiveState, PrimitiveStateArray
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool, SetBoolResponse

class EndEffector:
    server = None
    tfl = None
    int_marker = None
    state = None
    is_active = False
    def __init__(self, interactiveMarkerServer, tfListener, param):
        self.server = interactiveMarkerServer
        self.tfl = tfListener

        self.state = PrimitiveState()
        self.state.name = param["name"]
        self.state.parent_link_name = param["parent_link_name"]
        self.state.local_pose.position.x = param["local_pose"][0]
        self.state.local_pose.position.y = param["local_pose"][1]
        self.state.local_pose.position.z = param["local_pose"][2]
        self.state.local_pose.orientation.x = param["local_pose"][3]
        self.state.local_pose.orientation.y = param["local_pose"][4]
        self.state.local_pose.orientation.z = param["local_pose"][5]
        self.state.local_pose.orientation.w = param["local_pose"][6]
        if "M" in param:
            self.state.M = param["M"]
        else:
            self.state.M = [0.0] * 6
        if "D" in param:
            self.state.D = param["D"]
        else:
            self.state.D = [0.0] * 6
        if "K" in param:
            self.state.K = param["K"]
        else:
            self.state.K = [0.0] * 6
        if "wrench_gain" in param:
            self.state.wrench_gain = param["wrench_gain"]
        else:
            self.state.wrench_gain = [0.0] * 6
        if "support_com" in param:
            self.state.support_com = param["support_com"]
        else:
            self.state.support_com = False
        if "time" in param:
            self.state.time = param["time"]
        else:
            self.state.time = 0.1

        self.int_marker = InteractiveMarker()
        self.int_marker.name = self.state.name
        self.int_marker.scale = 0.4

    def activate(self):
        if self.is_active:
            return True
        self.int_marker.description = self.int_marker.name;
        self.int_marker.header.frame_id = "odom"

        try:
            (trans,rot) = self.tfl.lookupTransform(self.int_marker.header.frame_id, self.state.parent_link_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("failed to lookup transform between "+self.int_marker.header.frame_id+" and "+self.state.parent_link_name)
        scale, shear, angles, translation, persp = tf.transformations.decompose_matrix(tf.transformations.compose_matrix(translate=trans,angles=tf.transformations.euler_from_quaternion(rot)).dot(tf.transformations.compose_matrix(translate=[self.state.local_pose.position.x,self.state.local_pose.position.y,self.state.local_pose.position.z],angles=tf.transformations.euler_from_quaternion([self.state.local_pose.orientation.x,self.state.local_pose.orientation.y,self.state.local_pose.orientation.z,self.state.local_pose.orientation.w]))))
        self.state.pose.position.x = translation[0]
        self.state.pose.position.y = translation[1]
        self.state.pose.position.z = translation[2]
        self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w = tf.transformations.quaternion_from_euler(angles[0],angles[1],angles[2])
        self.int_marker.pose = copy.deepcopy(self.state.pose)

        self.int_marker.controls = []
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(self.int_marker, self.processFeedback)

        self.server.applyChanges()
        self.is_active = True

    def deactivate(self):
        if not self.is_active:
            return True
        self.server.erase(self.int_marker.name)
        self.server.applyChanges()

        self.is_active = False

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.state.pose = copy.deepcopy(feedback.pose)

if __name__ == "__main__":
    rospy.init_node("pml_interactive_marker_server")

    server = InteractiveMarkerServer("pml_interactive_marker_server")
    tfl = TransformListener()

    end_effectors = []
    if rospy.has_param("~end_effectors"):
        end_effector_params = rospy.get_param("~end_effectors")
        for end_effector_param in end_effector_params:
            end_effectors.append(EndEffector(server,tfl,end_effector_param))

    pub = rospy.Publisher('~command', PrimitiveStateArray, queue_size=1)

    is_active = False
    def handle_activate(req):
        global is_active
        for end_effector in end_effectors:
            if req.data:
                end_effector.activate()
            else:
                end_effector.deactivate()
        is_active = req.data
        return SetBoolResponse(True, "")
    s = rospy.Service('~activate', SetBool, handle_activate)

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        if is_active:
            msg = PrimitiveStateArray()
            for end_effector in end_effectors:
                msg.primitive_state.append(end_effector.state)
            pub.publish(msg)
        r.sleep()






