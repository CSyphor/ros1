Awesome ✅ ROS Noetic (Python3).
I’ll give you **exact minimal templates** for each topic — if you memorize + understand these, you can solve ANY variation in your exam.

---

# ✅ ROS Lab Exam Code Pack (Python3 / Noetic)

📌 **Your Goal:** Learn these 6 scripts → modify as needed in exam
📌 I’ll keep them short, clean, and fast to write under pressure

---

## 1️⃣ Teleop Bot (Keyboard Control)

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    'x': (0, 0)
}

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    rospy.init_node('teleop_key')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    speed = 0.4
    turn = 1.0
    cmd = Twist()

    while not rospy.is_shutdown():
        key = getKey()
        if key in moveBindings.keys():
            x, z = moveBindings[key]
            cmd.linear.x = x * speed
            cmd.angular.z = z * turn
        pub.publish(cmd)
        rate.sleep()
```

---

## 2️⃣ Wander Bot (Random Movement + Simple Avoidance)

```python
#!/usr/bin/env python3
import rospy, random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

cmd = Twist()
pub = None
min_dist = 0.5

def callback(scan):
    global cmd
    front = min(min(scan.ranges[0:15] + scan.ranges[-15:]))
    if front < min_dist:
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0
    else:
        cmd.linear.x = 0.3
        cmd.angular.z = random.uniform(-0.5, 0.5)
    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('wander_bot')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
```

---

## 3️⃣ Velocity Ramp (Smooth Acceleration)

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('vel_ramp')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)
cmd = Twist()

target_speed = 0.5
step = 0.05

while not rospy.is_shutdown():
    if cmd.linear.x < target_speed:
        cmd.linear.x += step
    pub.publish(cmd)
    rate.sleep()
```

> ✅ If asked for **ramp-down**, just reverse condition.

---

## 4️⃣ TurtleBot / Turtlesim Follow-Up Bot

Follow another bot’s /turtle1/pose:

```python
#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

leader = Pose()

def callback(msg):
    global leader
    leader = msg

if __name__ == "__main__":
    rospy.init_node('follow_bot')
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    cmd = Twist()
    while not rospy.is_shutdown():
        dx = leader.x - 5.5
        dy = leader.y - 5.5
        dist = math.sqrt(dx*dx + dy*dy)

        cmd.linear.x = min(1.0, dist)
        cmd.angular.z = math.atan2(dy, dx)
        pub.publish(cmd)
        rate.sleep()
```

> If following another robot → `/odom` instead of Pose.

---

## 5️⃣ Action Server (Simple Fibonacci Example)

```python
#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciFeedback, FibonacciResult

def execute_cb(goal):
    seq = [0, 1]
    feedback = FibonacciFeedback()
    result = FibonacciResult()

    for i in range(1, goal.order):
        seq.append(seq[i] + seq[i-1])
        feedback.sequence = seq
        server.publish_feedback(feedback)
        rospy.sleep(1)

    result.sequence = seq
    server.set_succeeded(result)

rospy.init_node('fibonacci_server')
server = actionlib.SimpleActionServer('fibonacci', FibonacciAction, execute_cb, False)
server.start()
rospy.spin()
```

---

## 6️⃣ Action Client

```python
#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal

rospy.init_node('fibonacci_client')
client = actionlib.SimpleActionClient('fibonacci', FibonacciAction)
client.wait_for_server()

goal = FibonacciGoal(order=5)
client.send_goal(goal)
client.wait_for_result()

print(client.get_result())
```

---

# ✅ How to Study in 2 Hours (Exam Booster Mode)

| Task                                          | Time   |
| --------------------------------------------- | ------ |
| Write all scripts once by hand                | 45 min |
| Run on turtlesim / simulation                 | 30 min |
| Try quick mods (change topic name, threshold) | 20 min |
| Practice writing without reference            | 20 min |

---

## ✅ Quick Commands to Remember

| Purpose         | Command                                                      |
| --------------- | ------------------------------------------------------------ |
| Create a pkg    | `catkin_create_pkg pkg_name rospy geometry_msgs sensor_msgs` |
| Make executable | `chmod +x yourfile.py`                                       |
| Run node        | `rosrun pkg yourfile.py`                                     |
| Check topics    | `rostopic list`                                              |
| Echo            | `rostopic echo /cmd_vel`                                     |
| Publish         | `rostopic pub ...`                                           |

---

If you want, I can give you:

✅ A 1-page **cheat sheet** for final revision
✅ Mini-test questions similar to your lab
✅ Simulation commands for **Wander Bot** & **Follow Bot**

Would you like me to generate a **timed practice test** right now?
