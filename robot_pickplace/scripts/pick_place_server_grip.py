#!/usr/bin/env python3
import rospy, actionlib, time
from robot_pickplace.msg import PickPlaceGripAction, PickPlaceGripFeedback, PickPlaceGripResult

def execute(g):
    fb,res=PickPlaceGripFeedback(),PickPlaceGripResult()
    for i in range(3):
        if server.is_preempt_requested(): res.result="Preempted"; server.set_preempted(res); return
        fb.cx,fb.cy,fb.cz,fb.status=g.xp,g.yp,i,"Moving to pick"; server.publish_feedback(fb); time.sleep(1)
    time.sleep(1)  # grasp
    if g.grip<0.5: res.result="Grip too weak";res.grip_ok=False;server.set_aborted(res);return
    # place
    for i in range(3):
        if server.is_preempt_requested(): res.result="Preempted"; server.set_preempted(res); return
        fb.cx,fb.cy,fb.cz,fb.status=g.xq,g.yq,i,"Moving to place"; server.publish_feedback(fb); time.sleep(1)
    res.result="Done";res.grip_ok=True;server.set_succeeded(res)

rospy.init_node("pick_place_server_grip")
server=actionlib.SimpleActionServer("pick_place_action_grip",PickPlaceGripAction,execute,False)
server.start();rospy.spin()
