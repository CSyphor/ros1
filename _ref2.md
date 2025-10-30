## 🧩 Q1. **Teleop Bot (Custom Keyboard Control)**

**Question:**
Write a ROS node that lets you control a mobile robot (or turtle) using `WASD` keys for direction and `Q` to quit.

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

rospy.init_node('teleop_bot')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)
msg = Twist()

settings = termios.tcgetattr(sys.stdin)
print("WASD to move, Q to quit")

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

while not rospy.is_shutdown():
    key = getKey()
    if key == 'w':
        msg.linear.x = 2.0; msg.angular.z = 0.0
    elif key == 's':
        msg.linear.x = -2.0; msg.angular.z = 0.0
    elif key == 'a':
        msg.linear.x = 0.0; msg.angular.z = 2.0
    elif key == 'd':
        msg.linear.x = 0.0; msg.angular.z = -2.0
    elif key == 'q':
        break
    else:
        msg.linear.x = 0.0; msg.angular.z = 0.0
    pub.publish(msg)
    rate.sleep()

msg.linear.x = 0.0; msg.angular.z = 0.0
pub.publish(msg)
```

---

## 🧩 Q2. **Velocity Ramp with Teleop Control**

**Question:**
Write a ROS node that takes keyboard input (`WASD`) but accelerates smoothly (ramps velocity) instead of instant jumps.

```python
#!/usr/bin/env python
import rospy, sys, select, termios, tty
from geometry_msgs.msg import Twist

rospy.init_node('teleop_velramp')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(20)
msg = Twist()

settings = termios.tcgetattr(sys.stdin)
v = 0.0
w = 0.0

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

while not rospy.is_shutdown():
    key = getKey()
    if key == 'w': v += 0.1
    elif key == 's': v -= 0.1
    elif key == 'a': w += 0.1
    elif key == 'd': w -= 0.1
    elif key == 'q': break

    v = max(min(v, 2.0), -2.0)
    w = max(min(w, 2.0), -2.0)
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)
    rate.sleep()

msg.linear.x = msg.angular.z = 0
pub.publish(msg)
```

---

## 🧩 Q3. **Action Server + Client: “Move Robot to Target”**

**Question:**
Implement an **Action Server** that moves a simulated robot from current to goal coordinates.
Use a **SimpleActionClient** to send the goal and display progress.

### ✅ Server (`move_bot_server.py`)

```python
#!/usr/bin/env python
import rospy, actionlib, math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from your_pkg.msg import MoveBotAction, MoveBotFeedback, MoveBotResult

x = 0; y = 0; yaw = 0
def pose_cb(p): 
    global x, y, yaw
    x, y, yaw = p.x, p.y, p.theta

def execute(goal):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    msg = Twist()
    fb = MoveBotFeedback()
    res = MoveBotResult()

    while not rospy.is_shutdown():
        dx, dy = goal.x - x, goal.y - y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.1:
            break
        msg.linear.x = 1.0 * dist
        msg.angular.z = 4.0 * (math.atan2(dy, dx) - yaw)
        pub.publish(msg)
        fb.distance = dist
        server.publish_feedback(fb)
        rate.sleep()

    msg.linear.x = msg.angular.z = 0
    pub.publish(msg)
    res.success = True
    server.set_succeeded(res)

rospy.init_node('move_bot_server')
rospy.Subscriber('/turtle1/pose', Pose, pose_cb)
server = actionlib.SimpleActionServer('move_bot', MoveBotAction, execute, False)
server.start()
rospy.spin()
```

### ✅ Client (`move_bot_client.py`)

```python
#!/usr/bin/env python
import rospy, actionlib
from your_pkg.msg import MoveBotAction, MoveBotGoal

rospy.init_node('move_bot_client')
client = actionlib.SimpleActionClient('move_bot', MoveBotAction)
client.wait_for_server()

goal = MoveBotGoal()
goal.x = 8.0; goal.y = 8.0
client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
```

---

## 🧩 Q4. **Follow Another Bot**

**Question:**
You have two turtles (`/turtle1` and `/turtle2`). Write a node that makes `turtle2` follow `turtle1` using pose feedback.

```python
#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

t1 = Pose()
t2 = Pose()

def t1_cb(p): global t1; t1 = p
def t2_cb(p): global t2; t2 = p

rospy.init_node('follow_bot')
rospy.Subscriber('/turtle1/pose', Pose, t1_cb)
rospy.Subscriber('/turtle2/pose', Pose, t2_cb)
pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)
msg = Twist()

while not rospy.is_shutdown():
    dx = t1.x - t2.x
    dy = t1.y - t2.y
    dist = math.sqrt(dx**2 + dy**2)
    angle = math.atan2(dy, dx)
    msg.linear.x = 0.8 * dist
    msg.angular.z = 4.0 * (angle - t2.theta)
    pub.publish(msg)
    rate.sleep()
```

---

## 🧩 Q5. **Wander + Collision Avoid (Laser / Simulated Sensor)**

**Question:**
Write a ROS node that simulates random wandering, but if it detects an obstacle (distance < 1.0 from fake input), it rotates randomly and continues.

```python
#!/usr/bin/env python
import rospy, random
from geometry_msgs.msg import Twist

rospy.init_node('wander_avoid')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(5)
msg = Twist()

while not rospy.is_shutdown():
    dist = random.uniform(0.5, 3.0)   # fake obstacle distance
    if dist < 1.0:
        msg.linear.x = 0.0
        msg.angular.z = random.uniform(-3.0, 3.0)
    else:
        msg.linear.x = random.uniform(0.5, 2.0)
        msg.angular.z = 0.0
    pub.publish(msg)
    rate.sleep()
```

---

### ✅ CMake + Package Setup Steps (common for all)

```bash
cd ~/catkin_ws/src
catkin_create_pkg your_pkg rospy geometry_msgs turtlesim actionlib actionlib_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Then put `.py` files in `your_pkg/scripts`, make them executable:

```bash
chmod +x *.py
rosrun your_pkg teleop_bot.py
```

