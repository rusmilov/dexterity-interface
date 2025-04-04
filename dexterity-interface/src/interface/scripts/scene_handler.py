#!/usr/bin/env python

"""
Handles the objects in the scene.
"""


# import rospy
# from std_msgs.msg import String
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# class SceneHandler():
#     def __init__(self):
#         rospy.init_node('scene_handler')


#         marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
#         rate = rospy.Rate(10)

#         # List of positions for each cube (x, y, z)
#         cube_positions = [
#             (0.0, 0.0, 0.5),
#             (0.5, 0.0, 0.5),
#             (0.0, 0.5, 0.5),
#             (-0.5, 0.0, 0.5),
#             (0.0, -0.5, 0.5),
#         ]

#         while not rospy.is_shutdown():
#             for i, pos in enumerate(cube_positions):
#                 marker = Marker()
#                 marker.header.frame_id = "scene_root"
#                 marker.header.stamp = rospy.Time.now()
#                 marker.ns = "cube_markers"
#                 marker.id = i
#                 marker.type = Marker.CUBE
#                 marker.action = Marker.ADD

#                 marker.pose.position.x = pos[0]
#                 marker.pose.position.y = pos[1]
#                 marker.pose.position.z = pos[2]
#                 marker.pose.orientation.w = 1.0  # no rotation

#                 marker.scale.x = 1
#                 marker.scale.y = 0.1
#                 marker.scale.z = 0.1

#                 marker.color.r = 1.0
#                 marker.color.g = 0.5
#                 marker.color.b = 0.0
#                 marker.color.a = 1.0  # fully opaque

#                 marker.lifetime = rospy.Duration(0)  # lasts forever

#                 marker_pub.publish(marker)

#             rate.sleep()


     
        

# if __name__ == "__main__":
#     scene_handler = SceneHandler()
#     rospy.spin()  # Keep the node running


import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def processFeedback(feedback):
    p = feedback.pose.position
    print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

if __name__=="__main__":
    rospy.init_node("visualization_marker")
    
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("visualization_marker")
    
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "scene_root"
    int_marker.name = "scene_root"
    int_marker.description = "Simple 1-DOF Control"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 1
    box_marker.scale.y = 1
    box_marker.scale.z = 1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    # add the control to the interactive marker
    int_marker.controls.append(rotate_control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()