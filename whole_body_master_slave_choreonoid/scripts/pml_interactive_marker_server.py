#!/usr/bin/env python

# rosparams
#   end_effectors:
#     - name: <name>
#       parent_link_name: <linkname for URDF>
#       local_pose: [x, y, z, x, y, z, w] # default 0,0,0,0,0,0,1
#       is_wrenchC_global: <bool> # default false
#       wrenchC: [<double>, <double>, <double>, <double>, <double>, <double>,
#                 <double>, <double>, <double>, <double>, <double>, <double> ...] # default []
#       wrenchld: [<double>,
#                  <double> ...] # default []
#       wrenchud: [<double>,
#                  <double> ...] # default []
#       pose_follow_gain: [<double>, <double>, <double>, <double>, <double>, <double>] # default 1.0
#       wrench_follow_gain: [<double>, <double>, <double>, <double>, <double>, <double>] # default 0.0
#       support_com: <bool> #default false
#       time: <double> # default 0.1
#       wrench_gain: [x ,y, z, rx, ry, rz] # default 0
#       M: [x ,y, z, rx, ry, rz] # default 10, 10, 10, 5, 5, 5
#       D: [x ,y, z, rx, ry, rz] # default 200, 200, 200, 100, 100, 100
#       K: [x ,y, z, rx, ry, rz] # default 400, 400, 400, 200, 200, 200

# publish
#   ~command: primitive_motion_level_msgs/PrimitiveStateArray

# server
#   ~activate: std_srvs/SetBool

import copy

import rospy

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import tf
from tf import TransformListener, transformations
from primitive_motion_level_msgs.msg import PrimitiveState, PrimitiveStateArray
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool, SetBoolResponse

tf_prefix = ""

class EndEffector:
    server = None
    menu_handler = None
    tfl = None
    int_marker = None
    state = None
    is_active = False
    def __init__(self, interactiveMarkerServer, tfListener, param):
        self.server = interactiveMarkerServer
        self.tfl = tfListener
        self.menu_handler = MenuHandler()

        self.state = PrimitiveState()
        self.state.name = param["name"]
        if self.state.name != "com":
            self.state.parent_link_name = param["parent_link_name"]
        else:
            self.state.parent_link_name = "com"
        if "local_pose" in param:
            self.state.local_pose.position.x = param["local_pose"][0]
            self.state.local_pose.position.y = param["local_pose"][1]
            self.state.local_pose.position.z = param["local_pose"][2]
            self.state.local_pose.orientation.x = param["local_pose"][3]
            self.state.local_pose.orientation.y = param["local_pose"][4]
            self.state.local_pose.orientation.z = param["local_pose"][5]
            self.state.local_pose.orientation.w = param["local_pose"][6]
        else:
            self.state.local_pose.position.x = 0.0
            self.state.local_pose.position.y = 0.0
            self.state.local_pose.position.z = 0.0
            self.state.local_pose.orientation.x = 0.0
            self.state.local_pose.orientation.y = 0.0
            self.state.local_pose.orientation.z = 0.0
            self.state.local_pose.orientation.w = 1.0
        if "support_com" in param:
            self.state.support_com = param["support_com"]
        else:
            self.state.support_com = False
        if "time" in param:
            self.state.time = param["time"]
        else:
            self.state.time = 0.1
        if "is_wrenchC_global" in param:
            self.state.is_wrenchC_global = param["is_wrenchC_global"]
        else:
            self.state.is_wrenchC_global = False
        if "wrenchC" in param:
            self.state.wrenchC = map(lambda x: float(x), param["wrenchC"])
        else:
            self.state.wrenchC = []
        if "wrenchld" in param:
            self.state.wrenchld = map(lambda x: float(x), param["wrenchld"])
        else:
            self.state.wrenchld = []
        if "wrenchud" in param:
            self.state.wrenchud = map(lambda x: float(x), param["wrenchud"])
        else:
            self.state.wrenchud = []
        if "pose_follow_gain" in param:
            self.state.pose_follow_gain = param["pose_follow_gain"]
        else:
            self.state.pose_follow_gain = [1.0]*6
        if "wrench_follow_gain" in param:
            self.state.wrench_follow_gain = param["wrench_follow_gain"]
        else:
            self.state.wrench_follow_gain = [0.0]*6
        if "M" in param:
            self.state.M = map(lambda x: float(x), param["M"])
        else:
            self.state.M = [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
        if "D" in param:
            self.state.D = map(lambda x: float(x), param["D"])
        else:
            self.state.D = [200.0, 200.0, 200.0, 100.0, 100.0, 100.0]
        if "K" in param:
            self.state.K = map(lambda x: float(x), param["K"])
        else:
            self.state.K = [400.0, 400.0, 400.0, 200.0, 200.0, 200.0]

        self.int_marker = InteractiveMarker()
        self.int_marker.name = self.state.name
        self.int_marker.scale = 0.4

    def activate(self):
        if self.is_active:
            return True
        self.int_marker.description = self.int_marker.name;
        self.int_marker.header.frame_id = tf_prefix + "odom"

        try:
            (trans,rot) = self.tfl.lookupTransform(self.int_marker.header.frame_id, tf_prefix + self.state.parent_link_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("failed to lookup transform between "+self.int_marker.header.frame_id+" and "+tf_prefix + self.state.parent_link_name)
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

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append( Marker() )
        control.markers[0].type = Marker.CUBE
        control.markers[0].scale.x = 0.08
        control.markers[0].scale.y = 0.08
        control.markers[0].scale.z = 0.08
        control.markers[0].color.r = 0.8
        control.markers[0].color.g = 0.8
        control.markers[0].color.b = 0.8
        control.markers[0].color.a = 0.5
        self.int_marker.controls.append(control)

        self.server.insert(self.int_marker, self.processFeedback)

        self.menu_handler = MenuHandler()
        entry = self.menu_handler.insert( "Support COM", callback=self.supportCOMCb )
        if self.state.support_com:
            self.menu_handler.setCheckState( entry, MenuHandler.CHECKED )
        else:
            self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED )
        self.menu_handler.apply(self.server, self.state.name)

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

    def supportCOMCb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self.state.support_com = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self.state.support_com = True
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("pml_interactive_marker_server")

    server = InteractiveMarkerServer("pml_interactive_marker_server")
    tfl = TransformListener()

    if rospy.has_param("~tf_prefix"):
        tf_prefix = rospy.get_param("~tf_prefix")
        if tf_prefix is not "":
            tf_prefix = "/" + tf_prefix + "/"
    else:
        tf_prefix = ""


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

    try:
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if is_active:
                msg = PrimitiveStateArray()
                for end_effector in end_effectors:
                    msg.primitive_state.append(end_effector.state)
                pub.publish(msg)
            r.sleep()
    except rospy.ROSInterruptException:
        pass






