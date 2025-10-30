#!/usr/bin/env python3
import rospy, actionlib
from robot_mover.msg import MoveRobotVelAction, MoveRobotVelFeedback, MoveRobotVelResult

def execute(goal):
    fb, res = MoveRobotVelFeedback(), MoveRobotVelResult()
    x, y = 0.0, 0.0
    step = max(0.05, goal.velocity*0.1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if server.is_preempt_requested():
            res.result = "Preempted"
            server.set_preempted(res)
            return
        if abs(goal.x-x)<0.1 and abs(goal.y-y)<0.1:
            res.result = "Reached with velocity %.1f"%goal.velocity
            server.set_succeeded(res)
            return
        x += step if goal.x>x else -step
        y += step if goal.y>y else -step
        fb.current_x, fb.current_y = x, y
        server.publish_feedback(fb)
        rate.sleep()

rospy.init_node("move_robot_server_vel")
server = actionlib.SimpleActionServer("move_robot_vel", MoveRobotVelAction, execute, False)
server.start()
rospy.spin()
