#!/usr/bin/env python3
import rospy, actionlib
from robot_pickplace.msg import PickPlaceGripAction, PickPlaceGripGoal

rospy.init_node("pick_place_client_grip")
c=actionlib.SimpleActionClient("pick_place_action_grip",PickPlaceGripAction);c.wait_for_server()
goal=PickPlaceGripGoal(xp=1,yp=2,zp=0,xq=3,yq=4,zq=0,grip=0.6)
c.send_goal(goal,feedback_cb=lambda f:rospy.loginfo((f.cx,f.cy,f.cz,f.status)))
c.wait_for_result();print("Result:",c.get_result())
