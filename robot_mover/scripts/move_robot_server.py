#!/usr/bin/env python3
import rospy, actionlib
from robot_mover.msg import MoveRobotAction, MoveRobotFeedback, MoveRobotResult

class MoveRobotServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "move_robot", MoveRobotAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        fb, res = MoveRobotFeedback(), MoveRobotResult()
        x, y = 0.0, 0.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                res.result = "Preempted"
                self.server.set_preempted(res)
                return
            if abs(goal.x-x)<0.1 and abs(goal.y-y)<0.1:
                res.result = "Reached (%.1f,%.1f)" % (goal.x, goal.y)
                self.server.set_succeeded(res)
                return
            x += 0.1 if goal.x>x else -0.1
            y += 0.1 if goal.y>y else -0.1
            fb.current_x, fb.current_y = x, y
            self.server.publish_feedback(fb)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("move_robot_server")
    MoveRobotServer()
    rospy.spin()
