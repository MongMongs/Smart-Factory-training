#

##

```~/ros2_ws/
    └── src/
        └── omo_r1_bringup/
            ├── scripts/
            │   ├── command_interface.py       # 새로운 스크립트
            │   └── teleop_key.py              # 기존 키보드 조종 스크립트 (필요 시 삭제)
            ├── launch/
            │   └── omo_r1_motor.launch.py
            ├── nodes/
            │   ├── omo_r1_motor_node.py
            │   └── cmd_vel_publisher.py        # 새로운 노드
            ├── CMakeLists.txt  
            └── package.xml    
```

## cmd_vel_publisher.py

```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            'control_commands',
            self.control_callback,
            10
        )
        self.speed = 0.3  # 직진 속도
        self.running = False  # 직진 상태 플래그
        self.lock = threading.Lock()  # 동기화용 락

        # 스레드 시작
        self.thread = threading.Thread(target=self.publish_cmd_vel)
        self.thread.start()

        self.get_logger().info('CmdVelPublisher node has been started.')

    def control_callback(self, msg):
        command = msg.data.lower()
        with self.lock:
            if command == 'run':
                self.running = True
                self.get_logger().info('Received run command. Starting to publish cmd_vel.')
            elif command == 'stop':
                self.running = False
                self.get_logger().info('Received stop command. Stopping cmd_vel publishing.')
            elif command == 'exit':
                self.running = False
                self.get_logger().info('Received exit command. Stopping node.')
                rclpy.shutdown()

    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        rate = 10  # 10Hz
        while rclpy.ok():
            with self.lock:
                if self.running:
                    self.publisher.publish(twist)
                    self.get_logger().info(f'Published cmd_vel: Linear.x = {twist.linear.x}')
            time.sleep(1.0 / rate)

    def __del__(self):
        self.get_logger().info('CmdVelPublisher node is shutting down.')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```
## command_interface.py 

```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')
        self.publisher = self.create_publisher(String, 'control_commands', 10)
        self.get_logger().info('CommandInterface node has been started.')
        self.run_command_interface()

    def run_command_interface(self):
        while rclpy.ok():
            try:
                command = input("Enter command (run/stop/exit): ").strip().lower()
                if command not in ['run', 'stop', 'exit']:
                    print("Invalid command. Please enter 'run', 'stop', or 'exit'.")
                    continue
                msg = String()
                msg.data = command
                self.publisher.publish(msg)
                self.get_logger().info(f'Published command: {command}')
                if command == 'exit':
                    break
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info('Exiting CommandInterface.')
                break

def main(args=None):
    rclpy.init(args=args)
    node = CommandInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```
## CMakeLists.txt

```
**cmake_minimum_required(VERSION 3.5)
project(omo_r1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # std_msgs 추가

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/omo_r1_motor_node.py
  nodes/cmd_vel_publisher.py       # 새로운 노드 추가
  scripts/command_interface.py     # 새로운 스크립트 추가
  scripts/teleop_key.py            # 기존 스크립트 (필요 시)
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
**
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
  <depend>std_msgs</depend> <!-- std_msgs 추가 -->

  <!-- Export information -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

## 빌드 및 설치

```
cd ~/ros2_ws/
colcon build
source install/setup.bash

```

## 터미널 1: omo_r1_motor_node.py 실행

```
ros2 run omo_r1_bringup omo_r1_motor_node.py
```

## 터미널 2: cmd_vel_publisher.py 실행

```
ros2 run omo_r1_bringup cmd_vel_publisher.py
```

## 터미널 3: command_interface.py 실행

```
ros2 run omo_r1_bringup command_interface.py
```


