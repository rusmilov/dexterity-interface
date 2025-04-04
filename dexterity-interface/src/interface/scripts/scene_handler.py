#!/usr/bin/env python

"""
Handles the objects in the scene.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin


class SceneHandler:
    def __init__(self):
        rospy.init_node('scene_handler')
    

        self.server = None
        self.menu_handler = MenuHandler()
        self.br = None
        self.counter = 0

    
        self.br = TransformBroadcaster()
        
        # create a timer to update the published transforms
        rospy.Timer(rospy.Duration(0.01), self.frameCallback)

        self.server = InteractiveMarkerServer("basic_controls")

        self.menu_handler.insert( "First Entry", callback=self.processFeedback )
        self.menu_handler.insert( "Second Entry", callback=self.processFeedback )
    
        
        position = Point(1, 0, 0)
        self.makeMenuMarker('base_link', 'apple', position )

        position = Point(2, 0, 0)
        self.makeMenuMarker('base_link','bannana', position )
        

        self.server.applyChanges()

        rospy.spin()


    def frameCallback(self, msg ):
        time = rospy.Time.now()
        self.br.sendTransform( (0, 0, 0), (0, 0, 0, 1.0), time, "scene", "base_link" )
        self.counter += 1

    def processFeedback(self, feedback ):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
        self.server.applyChanges()


    def makeBox(self, msg ):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def makeBoxControl(self, msg ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg) )
        msg.controls.append( control )
        return control

    def saveMarker(self, int_marker ):
        self.server.insert(int_marker, self.processFeedback)


    #####################################################################
    # Marker Creation

    def normalizeQuaternion(self, quaternion_msg ):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s


    def makeMenuMarker(self, frame_id, description, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id # "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = description #"context_menu" # REQUIRES UNIQUE NAME
        # int_marker.description = "Context Menu\n(Right Click)"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description = description #"Apple"
        # control.name = "menu_only_control"
        int_marker.controls.append(copy.deepcopy(control))

        # make one control showing a box
        marker = self.makeBox( int_marker )
        control.markers.append( marker )
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        self.menu_handler.apply( self.server, int_marker.name )


if __name__=="__main__":
    scene_handler = SceneHandler()
