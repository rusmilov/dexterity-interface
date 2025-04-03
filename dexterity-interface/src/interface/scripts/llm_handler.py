#!/usr/bin/env python
import rospy, os
from llm_interface import LLMInterface
from std_msgs.msg import String



class LLMHandler():
    def __init__(self):
        rospy.init_node('llm_handler')

        llm_role = ("You are an assistant for a 7-DOF Robot arm. Please rank ALL the following primitives and select the most useful primitives " +
                "to accomplish the given directive. Use 'STOP' to indicate the last relevant primitive. Additionally, " + 
                "for each primitive, output the corresponding coordinates in the Panda robot's link0 coordinate system. Each primitive " +
                "should be output in the format: 'PRIMITIVE x=... y=... z=...'." )

        self.llm = LLMInterface(llm_role)

        primitives = {
            "PICK":
                "Gripper moves to the specified position and closes the gripper.",
            "PLACE":
                "Gripper should move to the specified position and release. However, the gripper \
                won't directly move to it, but rather move to a pre-place position which is 8cm above the place position, \
                then move slowly with force control to the place position. Once it released, it will return to the pre-place \
                position. \
                Not like drop, the place action put things down slowly with care.",
            "RESET":
                "Releases the gripper and moves the arm back to the default position",
            "MOVE":
                "Moves the gripper to the specified position.",
            "GRASP":
                "Closes the gripper only.",
            # "RELEASE":
            #     "Releases the gripper only.",
            "STOP":
                "Immediately stops all movement in with the robot and move to the default position.",
            "UNSCREW":
                "Gripper goes to specified position, grasps the object, and untwists is 2 times, and then \
                pull the object up",
            "DROP":
                "Gripper should move to the specified position and release. However, the gripper\
                won't directly move to it, but rather move to a pre-drop position which is 8cm above the drop position,\
                then move IMMEDIATELY to the drop position. Once it released, it will return to the pre-drop \
                position.Not like place, the drop action put down things quickly."
        }


        prim_str  = "Primitives: " + " | ".join(f"{key}: {value}" for key, value in primitives.items())

        example_query = "Directive: Open a jar at position (0.8, 0.4, 0.2). Assume jar base is already stabilized." + prim_str
        example_response = "1. MOVE x=0.8 y=0.4 z= 0.5\n 2. UNSCREW x=0.8 y=0.4 z= 0.2\n 2. PLACE x=0.8 y=0.6 z= 0\n3. STOP\n"
        self.llm.init_history(example_query, example_response)


        self.llm_exec_pub = rospy.Publisher("/llm_commands", String, queue_size=10)
        self.llm_response_pub = rospy.Publisher("/llm_response", String, queue_size=10)
        rospy.Subscriber("/user_response", String, self.user_response_callback)

    def user_response_callback(self, msg):
        output = self.llm.query_openai("Directive: " + msg.data)
        commands = output.choices[0].message.content
        self.llm_response_pub.publish(commands)
        self.llm_exec_pub.publish(commands)
        

        rospy.loginfo("All commands have been published as a single message. Waiting for subscribers to process...")
        

if __name__ == "__main__":
    llm_handler = LLMHandler()
    rospy.spin()  # Keep the node running


