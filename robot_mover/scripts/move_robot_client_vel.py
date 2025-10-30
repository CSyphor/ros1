#!/usr/bin/env python3
import rospy, actionlib
from robot_mover.msg import MoveRobotVelAction, MoveRobotVelGoal

rospy.init_node("move_robot_client_vel")
client = actionlib.SimpleActionClient("move_robot_vel", MoveRobotVelAction)
client.wait_for_server()
goal = MoveRobotVelGoal(x=3.0, y=4.0, velocity=0.5)
client.send_goal(goal, feedback_cb=lambda fb: rospy.loginfo((fb.current_x, fb.current_y)))
client.wait_for_result()
print("Result:", client.get_result())
