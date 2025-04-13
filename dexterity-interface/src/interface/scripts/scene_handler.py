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
from std_msgs.msg import Header
from interface.msg import Object, ObjectArray



class SceneHandler:
    def __init__(self):
        rospy.init_node('scene_handler')
    
        self.server = InteractiveMarkerServer("scene/object_controls")
        self.br = TransformBroadcaster()
        self.object_pub = rospy.Publisher("/scene/objects", ObjectArray, queue_size=10)

        self.objects = {}  # Dictionary of current objects in scene
        self.object_idx = 0  # Name of marker needs to be unique or won't show

        # Timer to publish transforms and monitor objects in scene
        rospy.Timer(rospy.Duration(0.01), self.object_watcher_callback)
        rospy.Timer(rospy.Duration(0.01), self.frame_callback)
    

        rospy.spin()


    def frame_callback(self, msg ):
        time = rospy.Time.now()

        for _, obj in self.objects.items():
            self.br.sendTransform( (obj.x, obj.y, obj.z), (0, 0, 0, 1.0), time, obj.frame_id, "scene")


    def process_feedback(self, feedback ):
        s = "Feedback from object '" + feedback.marker_name
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



    def add_object(self, id, description, x, y, z, length, width, height) -> str:
        """
        id: MUST BE UNIQUE per object
        """

        frame_id = f"object/{id}"
        object = InteractiveMarker()
        object.header.frame_id = frame_id 
        object.pose.position = Point(0,0,0)
        object.scale = 1
        object.name = id 

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        object.controls.append(copy.deepcopy(control))

        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = length
        marker.scale.y = width
        marker.scale.z = height
        marker.color.r = 1
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        # Add text marker for description label
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = description
        text_marker.scale.x = width
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.pose.position.z = z + height/2 + 0.05 # A bit above the object

        control.markers.append( marker )
        control.markers.append(text_marker)

        control.always_visible = True
        object.controls.append(control)

        self.server.insert(object, self.process_feedback)
        self.server.applyChanges()

        self.object_idx += 1

        obj = Object()
        obj.header = Header()
        obj.header.stamp = rospy.Time.now()
        obj.id =id
        obj.frame_id = frame_id
        obj.description =description
        obj.x = x
        obj.y = y
        obj.width = width
        obj.length = length
        obj.height = height

        self.objects[id] = obj

    

    def object_watcher_callback(self, msg):
        # TODO Connect to VLM

        if not self.objects: # Only add if objects list is empty (first time called)
            self.add_object(id= 'apple_1', description='apple', 
                            x=0.5, y=0, z=0.025, length=0.05, width=0.05, height=0.05 )
            
            self.add_object(id= 'bread_1',  description='bread', 
                    x=0.5, y=0.2, z=0.025, length=0.05, width=0.05, height=0.05 )
            
            self.add_object(id= 'peanut_butter_1',  description='peanut butter jar', 
                    x=0.5, y=-0.2, z=0.025, length=0.05, width=0.05, height=0.05 )

        msg = ObjectArray()
        for id, obj in self.objects.items():
            msg.objects.append(obj)

        self.object_pub.publish(msg)

        


if __name__=="__main__":
    scene_handler = SceneHandler()
