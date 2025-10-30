#!/usr/bin/env python3
import rospy, actionlib, time
from robot_pickplace.msg import PickPlaceAction, PickPlaceFeedback, PickPlaceResult

def execute(goal):
    fb, res = PickPlaceFeedback(), PickPlaceResult()
    # Move to pick
    for i in range(3):
        if server.is_preempt_requested():
            res.result="Preempted"; server.set_preempted(res); return
        fb.cx, fb.cy, fb.cz, fb.status = goal.xp, goal.yp, i, "Moving to pick"
        server.publish_feedback(fb); time.sleep(1)
    time.sleep(1)  # grasp
    # Move to place
    for i in range(3):
        if server.is_preempt_requested():
            res.result="Preempted"; server.set_preempted(res); return
        fb.cx, fb.cy, fb.cz, fb.status = goal.xq, goal.yq, i, "Moving to place"
        server.publish_feedback(fb); time.sleep(1)
    res.result="Pick and Place done"; server.set_succeeded(res)

rospy.init_node("pick_place_server")
server = actionlib.SimpleActionServer("pick_place_action", PickPlaceAction, execute, False)
server.start(); rospy.spin()
