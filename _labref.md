### **1️⃣ Road Edge Detection & Center Line**

```python
#!/usr/bin/env python
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=50)
    if lines is not None:
        x1, y1, x2, y2 = np.mean(lines[:,0], axis=0).astype(int)
        cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 3)
    cv2.imshow("Road Center", frame)
    cv2.waitKey(1)

rospy.init_node('road_edge_detector')
rospy.Subscriber('/camera/image_raw', Image, callback)
rospy.spin()
```

---

### **2️⃣ Two-Node Velocity Ramp System**

#### **Node 1 – Random Velocity Generator**

```python
#!/usr/bin/env python
import rospy, random
from geometry_msgs.msg import Twist

rospy.init_node('random_vel_gen')
pub = rospy.Publisher('/random_vel', Twist, queue_size=10)
rate = rospy.Rate(5)
msg = Twist()

while not rospy.is_shutdown():
    msg.linear.x = random.uniform(-1.0, 1.0)
    msg.angular.z = random.uniform(-2.0, 2.0)
    pub.publish(msg)
    rate.sleep()
```

#### **Node 2 – Velocity Ramp Controller**

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('vel_ramp_controller')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(20)
ramp = 0.05
target = Twist()
current = Twist()

def callback(msg):
    global target
    target = msg

rospy.Subscriber('/random_vel', Twist, callback)

while not rospy.is_shutdown():
    current.linear.x += max(min(target.linear.x - current.linear.x, ramp), -ramp)
    current.angular.z += max(min(target.angular.z - current.angular.z, ramp), -ramp)
    pub.publish(current)
    rate.sleep()
```

---

### **3️⃣ Custom Teleop Node**

```python
#!/usr/bin/env python
import rospy, sys, select, termios, tty
from geometry_msgs.msg import Twist

rospy.init_node('custom_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(20)
settings = termios.tcgetattr(sys.stdin)
msg = Twist()

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

print("WASD to move, SPACE to stop, Q to quit")
try:
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w': msg.linear.x, msg.angular.z = 0.5, 0
        elif key == 's': msg.linear.x, msg.angular.z = -0.5, 0
        elif key == 'a': msg.linear.x, msg.angular.z = 0, 1.0
        elif key == 'd': msg.linear.x, msg.angular.z = 0, -1.0
        elif key == ' ': msg.linear.x, msg.angular.z = 0, 0
        elif key == 'q': break
        pub.publish(msg)
        rate.sleep()
finally:
    msg.linear.x = msg.angular.z = 0
    pub.publish(msg)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

---

### **4️⃣ Polygon Visualization (Publisher + Subscriber)**

#### **Publisher**

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

rospy.init_node('poly_pub')
pub = rospy.Publisher('/polygon_params', Int32MultiArray, queue_size=10)
rate = rospy.Rate(1)
msg = Int32MultiArray()

while not rospy.is_shutdown():
    msg.data = [5, 2]  # e.g., 5 edges, length 2
    pub.publish(msg)
    rate.sleep()
```

#### **Subscriber**

```python
#!/usr/bin/env python
import rospy, cv2, numpy as np
from std_msgs.msg import Int32MultiArray

def callback(data):
    n, l = data.data
    img = np.zeros((400,400,3), np.uint8)
    pts = []
    for i in range(n):
        angle = 2*np.pi*i/n
        x = int(200 + l*50*np.cos(angle))
        y = int(200 + l*50*np.sin(angle))
        pts.append([x,y])
    pts = np.array([pts], np.int32)
    cv2.polylines(img, pts, True, (0,255,0), 2)
    cv2.imshow("Polygon", img)
    cv2.waitKey(1)

rospy.init_node('poly_sub')
rospy.Subscriber('/polygon_params', Int32MultiArray, callback)
rospy.spin()
```

---

### **5️⃣ Velocity Publisher + Motion Subscriber**

#### **Publisher**

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('vel_pub')
pub = rospy.Publisher('/vel_data', Twist, queue_size=10)
rate = rospy.Rate(5)
msg = Twist()

while not rospy.is_shutdown():
    msg.linear.x = 1.0
    msg.angular.z = 0.5
    pub.publish(msg)
    rate.sleep()
```

#### **Subscriber**

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo("Robot moving: Linear=%.2f Angular=%.2f" % (data.linear.x, data.angular.z))

rospy.init_node('vel_sub')
rospy.Subscriber('/vel_data', Twist, callback)
rospy.spin()
```

---

### **6️⃣ Wander Bot with Safety (LaserScan)**

```python
#!/usr/bin/env python
import rospy, random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('wander_bot_safety')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(rospy.get_param('~rate', 10))

safe_dist = rospy.get_param('~safe_dist', 0.8)
fov = rospy.get_param('~fov', 60)
max_lin = rospy.get_param('~max_lin', 0.5)
max_ang = rospy.get_param('~max_ang', 1.5)

msg = Twist()
min_front = 10.0

def scan_cb(scan):
    global min_front
    ranges = list(scan.ranges)
    n = len(ranges)
    start = int(n/2 - fov/2)
    end = int(n/2 + fov/2)
    front = [r for r in ranges[start:end] if not rospy.is_shutdown()]
    min_front = min(front) if front else 10.0

rospy.Subscriber('/scan', LaserScan, scan_cb)

while not rospy.is_shutdown():
    if min_front < safe_dist:
        msg.linear.x = 0.0
        msg.angular.z = random.uniform(-max_ang, max_ang) * (min_front / safe_dist)
    else:
        msg.linear.x = max_lin * (min_front / 3.0)
        msg.angular.z = random.uniform(-max_ang, max_ang)
    pub.publish(msg)
    rate.sleep()
```

---

All scripts go under `scripts/` in your ROS package → make executable (`chmod +x filename.py`) → then:

```bash
rosrun your_pkg <filename>.py
```
