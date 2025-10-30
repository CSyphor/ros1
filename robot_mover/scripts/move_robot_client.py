#!/usr/bin/env python3
import rospy, actionlib
from robot_mover.msg import MoveRobotAction, MoveRobotGoal

def feedback_cb(fb):
    rospy.loginfo("Feedback: (%.2f, %.2f)"%(fb.current_x, fb.current_y))

rospy.init_node("move_robot_client")
client = actionlib.SimpleActionClient("move_robot", MoveRobotAction)
client.wait_for_server()
goal = MoveRobotGoal(x=2.0, y=3.0)
client.send_goal(goal, feedback_cb=feedback_cb)
client.wait_for_result()
print("Result:", client.get_result())

