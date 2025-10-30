#!/usr/bin/env python3
import rospy, actionlib
from robot_pickplace.msg import PickPlaceAction, PickPlaceGoal

rospy.init_node("pick_place_client")
c=actionlib.SimpleActionClient("pick_place_action",PickPlaceAction);c.wait_for_server()
goal=PickPlaceGoal(xp=1,yp=2,zp=0,xq=3,yq=4,zq=0)
c.send_goal(goal,feedback_cb=lambda f:rospy.loginfo((f.cx,f.cy,f.cz,f.status)))
rospy.sleep(2);c.cancel_goal()
c.wait_for_result();print("Result:",c.get_result())
