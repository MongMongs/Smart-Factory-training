# 키보드로 주행 조정

## 아래 코드 파일들의 위치

```
~/ros2_ws/
    └── src/
        └── omo_r1_bringup/
            ├── scripts/
            │   └── teleop_key.py
            ├── launch/
            │   └── omo_r1_motor.launch.py
            ├── nodes/
            │   ├── omo_r1_motor_node.py
            │   └── teleop_key.py
            ├── CMakeLists.txt  
            └── package.xml    
```

## omo_r1_motor_node.py

```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class OmoR1MotorNode(Node):
    def __init__(self):
        super().__init__('omo_r1_motor_node')

        port = '/dev/ttyUSB0'  # /dev/ttyMotor 대신 /dev/ttyUSB0 사용
        baud_rate = 115200
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2) 
            self.get_logger().info(f'Connected to motor driver on port: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to motor driver: {e}')
            self.ser = None 
            return

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Omo R1 motor node has been started.')

    def cmd_vel_callback(self, data):
        linear_speed = data.linear.x
        angular_speed = data.angular.z

        self.get_logger().info(f'Received cmd_vel - Linear: {linear_speed}, Angular: {angular_speed}')
        
        self.send_motor_command(linear_speed, angular_speed)

    def send_motor_command(self, linear_speed, angular_speed):
        if self.ser is None:
            self.get_logger().error("Serial port is not initialized.")
            return
        try:
            command = f'$CVW,{int(linear_speed * 1000)},{int(angular_speed * 1000)}\n'
            self.ser.write(command.encode())
            self.get_logger().info(f'Sent command: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send motor command: {e}')

    def __del__(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')

def main(args=None):
    rclpy.init(args=args)
    node = OmoR1MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## teleop_key.py
nodes 안에 들어가는 코드
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios

class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()

            if key == 'w':
                if self.speed < 0.5:
                    self.speed += 0.1
                twist.linear.x = self.speed
            elif key == 's':
                if self.speed > 0.0:
                    self.speed -= 0.1
                twist.linear.x = self.speed
            elif key == 'a':
                twist.angular.z = 0.5
            elif key == 'd':
                twist.angular.z = -0.5
            elif key == ' ' or key == '\x03':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                break

            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## teleop_key.py
scripts 안에 들어가는 코드
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopKey(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.0

    def get_key(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()
            if key == 'w':
                if self.speed < 0.5:
                    self.speed += 0.1
                twist.linear.x = self.speed
            elif key == 's':
                if self.speed > 0.0:
                    self.speed -= 0.1
                twist.linear.x = self.speed
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

## CMakeLists.txt

```
cmake_minimum_required(VERSION 3.5)
project(omo_r1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/omo_r1_motor_node.py
  scripts/teleop_key.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## package.xml
```
<?xml version="1.0"?>
<package format="2">
  <name>omo_r1_bringup</name>
  <version>0.1.0</version>
  <description>OMO R1 bringup package for ROS2</description>

  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache 2.0</license>

  <!-- Build tool dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- Export information -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
