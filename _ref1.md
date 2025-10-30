```bash
cd ~/catkin_ws/src
catkin_create_pkg turtle_tasks rospy std_msgs geometry_msgs turtlesim
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Now go into the `src/turtle_tasks/scripts/` folder — create `.py` files here for each question.

---

## 🐢 1️⃣ Question: Move the Turtle in a Square Path

### 📜 Task:

Write a node that moves the turtlesim turtle in a **square** — each side = 2 seconds forward, 90° turn.

### Code: `square_move.py`

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_square():
    rospy.init_node('square_move')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    vel = Twist()

    for i in range(4):
        vel.linear.x = 1.0
        vel.angular.z = 0.0
        rospy.loginfo(f"Moving forward side {i+1}")
        pub.publish(vel)
        rospy.sleep(2)

        vel.linear.x = 0.0
        vel.angular.z = 1.57
        rospy.loginfo("Turning 90 degrees")
        pub.publish(vel)
        rospy.sleep(1)

    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    rospy.loginfo("Completed square")

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
```

### 🧱 Add to `CMakeLists.txt`

In the end of file:

```cmake
catkin_install_python(PROGRAMS
  scripts/square_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### ✅ Run

```bash
roscore &
rosrun turtlesim turtlesim_node
rosrun turtle_tasks square_move.py
```

---

## 🌀 2️⃣ Question: Velocity Ramp — Smooth Acceleration and Stop

### Code: `velocity_ramp.py`

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def velocity_ramp():
    rospy.init_node('velocity_ramp')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    rate = rospy.Rate(10)

    # Accelerate forward
    for spd in range(0, 11):
        vel.linear.x = spd * 0.1
        pub.publish(vel)
        rospy.loginfo(f"Speed: {vel.linear.x}")
        rate.sleep()

    # Decelerate
    for spd in reversed(range(0, 11)):
        vel.linear.x = spd * 0.1
        pub.publish(vel)
        rospy.loginfo(f"Speed: {vel.linear.x}")
        rate.sleep()

    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)

if __name__ == "__main__":
    velocity_ramp()
```

---

## 🧭 3️⃣ Question: Follow-Up Bot (turtle2 follows turtle1)

### Step 1 — Spawn 2 turtles

```bash
rosservice call /spawn 4 4 0 "turtle2"
```

### Code: `follow_bot.py`

```python
#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

leader = Pose()
follower = Pose()
pub = None

def leader_pose(msg):
    global leader
    leader = msg

def follower_pose(msg):
    global follower
    follower = msg

def follow_bot():
    global pub
    rospy.init_node('follow_bot')
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, leader_pose)
    rospy.Subscriber('/turtle2/pose', Pose, follower_pose)

    rate = rospy.Rate(10)
    vel = Twist()
    Kp = 1.0
    Ka = 4.0

    while not rospy.is_shutdown():
        dx = leader.x - follower.x
        dy = leader.y - follower.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        diff = angle_to_goal - follower.theta

        vel.linear.x = Kp * distance
        vel.angular.z = Ka * diff
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    follow_bot()
```

### Run:

```bash
rosrun turtlesim turtlesim_node
rosrun turtle_tasks follow_bot.py
```

Now move `turtle1` using `turtle_teleop_key` and watch `turtle2` follow it.

---

## 🎲 4️⃣ Question: Wander Bot (Random Path)

### Code: `wander_bot.py`

```python
#!/usr/bin/env python3
import rospy, random
from geometry_msgs.msg import Twist

def wander():
    rospy.init_node('wander_bot')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    vel = Twist()

    while not rospy.is_shutdown():
        vel.linear.x = random.uniform(0.5, 2.0)
        vel.angular.z = random.uniform(-2.0, 2.0)
        rospy.loginfo(f"Linear: {vel.linear.x:.2f}, Angular: {vel.angular.z:.2f}")
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    wander()
```

---

## 🧠 5️⃣ Question: Action Server for Turtle Motion

*(If your syllabus includes Actions, this one is strong.)*

Create a custom action file (if not given):

```bash
cd ~/catkin_ws/src/turtle_tasks/action
touch MoveTurtle.action
```

### In `MoveTurtle.action`

```
float32 distance
---
bool success
---
float32 progress
```

Update `CMakeLists.txt`:

```cmake
find_package(actionlib_msgs REQUIRED)
add_action_files(
  DIRECTORY action
  FILES MoveTurtle.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)
catkin_package(CATKIN_DEPENDS actionlib_msgs)
```

Rebuild:

```bash
cd ~/catkin_ws && catkin_make
```

### Action Server Node: `turtle_action_server.py`

```python
#!/usr/bin/env python3
import rospy, actionlib
from geometry_msgs.msg import Twist
from turtle_tasks.msg import MoveTurtleAction, MoveTurtleFeedback, MoveTurtleResult

def execute_cb(goal):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    feedback = MoveTurtleFeedback()
    result = MoveTurtleResult()
    vel = Twist()

    dist = 0.0
    t0 = rospy.Time.now().to_sec()
    while dist < goal.distance:
        vel.linear.x = 1.0
        pub.publish(vel)
        t1 = rospy.Time.now().to_sec()
        dist = t1 - t0
        feedback.progress = dist / goal.distance
        server.publish_feedback(feedback)
        rate.sleep()

    vel.linear.x = 0
    pub.publish(vel)
    result.success = True
    server.set_succeeded(result)

rospy.init_node('turtle_action_server')
server = actionlib.SimpleActionServer('move_turtle', MoveTurtleAction, execute_cb, False)
server.start()
rospy.spin()
```

### Client Node: `turtle_action_client.py`

```python
#!/usr/bin/env python3
import rospy, actionlib
from turtle_tasks.msg import MoveTurtleAction, MoveTurtleGoal

rospy.init_node('turtle_action_client')
client = actionlib.SimpleActionClient('move_turtle', MoveTurtleAction)
client.wait_for_server()

goal = MoveTurtleGoal(distance=5.0)
client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
```

---
