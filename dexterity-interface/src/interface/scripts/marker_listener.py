#!/usr/bin/env python

import rospy
from visualization_msgs.msg import InteractiveMarkerFeedback

def feedback_callback(msg):
    if msg.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo("Marker moved:")
        rospy.loginfo(f"  Name: {msg.marker_name}")
        rospy.loginfo(f"  Position: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}")
        # You can convert this into a joint command here.

def main():
    rospy.init_node('marker_feedback_listener')
    rospy.Subscriber('/basic_controls/feedback', InteractiveMarkerFeedback, feedback_callback)
    rospy.loginfo("Listening for marker feedback on /basic_controls/feedback")
    rospy.spin()

if __name__ == '__main__':
    main()
