
## 아래 파일 위치

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
    
    Arduino IDE 입력 - wd_bh_arduino_control.ino
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
        self.speed = 0.3  # 기본 직진 속도
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
            elif command == 'wd':
                self.get_logger().info('Received wd command. Performing custom actions.')
                self.perform_wd_action()  # wd 명령 처리 함수 호출
            elif command == 'bh':
                self.get_logger().info('Received bh command. Performing custom actions.')
                self.perform_bh_action()  # bh 명령 처리 함수 호출
            elif command == 'exit':
                self.running = False
                self.get_logger().info('Received exit command. Stopping node.')
                rclpy.shutdown()

    def perform_wd_action(self):
        """wd 명령을 수행하는 함수"""
        # 1. 즉시 정지
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

        # 2. 2초 동안 전진
        time.sleep(1)  # 잠시 멈춘 후
        twist.linear.x = 0.3  # 직진 속도로 전진
        self.publisher.publish(twist)
        self.get_logger().info('Moving forward for 2 seconds.')
        time.sleep(2)

        # 3. 다시 정지
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped after moving forward.')

        # 4. "제초제 분사" 메시지 출력 및 3초 카운트다운
        print("제초제 분사")
        for i in range(3, 0, -1):
            print(f"{i}...")
            time.sleep(1)

        # 5. 다시 직진
        self.running = True
        self.get_logger().info('Resuming forward movement.')

    def perform_bh_action(self):
        """bh 명령을 수행하는 함수"""
        # 1. 즉시 정지
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

        # 2. 2초 동안 뒤로 이동
        twist.linear.x = -0.3  # 뒤로 가는 속도
        self.publisher.publish(twist)
        self.get_logger().info('Moving backward for 2 seconds.')
        time.sleep(2)

        # 3. 다시 정지
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped after moving backward.')

        # 4. 6번 핀을 2초간 ON하고 카운트다운 시작 (아두이노 또는 외부 기기와의 통신)
        print("Turning on pin 6 for 2 seconds and starting countdown.")
        # 여기서 아두이노로 'bh' 명령 전송 코드 필요
        # ser.write(b'bh\n') 와 같은 방식으로 아두이노와 통신

        # 5. 카운트 다운 출력
        for i in range(3, 0, -1):
            print(f"{i}...")
            time.sleep(1)

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
                command = input("Enter command (run/stop/wd/exit): ").strip().lower()
                if command not in ['run', 'stop', 'wd', 'exit']:
                    print("Invalid command. Please enter 'run', 'stop', 'wd', or 'exit'.")
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

## teleop_key.py

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

## CMakeLists.txt

```
cmake_minimum_required(VERSION 3.5)
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

## wd_bh_arduino_control.ino


```
#include <Servo.h>

Servo servoMotor;
int currentAngle = 90;  // 서보 모터의 현재 각도 (90도: 중앙)
int angleStep = 60;     // 각도 변화 (60도)
bool directionRight = true;  // 오른쪽 방향이면 true, 왼쪽이면 false

void setup() {
  // 서보 모터 핀 설정 (예: D9 핀)
  servoMotor.attach(9);

  // 서보 모터를 중앙으로 설정
  servoMotor.write(currentAngle);

  // 시리얼 통신 시작
  Serial.begin(115200);
  Serial.println("Servo motor initialized to 90 degrees.");
}

void loop() {
  // 시리얼로부터 데이터가 들어왔는지 확인
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // '\n'까지 데이터를 읽음
    command.trim();

    if (command == "wd") {
      Serial.println("Received 'wd' command");
      controlServo();  // 서보 모터 제어
    } else if (command == "bh") {
      Serial.println("Received 'bh' command");
      executeBHCommand();  // bh 명령 처리
    }
  }
}

// 서보 모터 제어 함수
void controlServo() {
  if (directionRight) {
    currentAngle += angleStep;  // 오른쪽으로 회전
    if (currentAngle > 180) currentAngle = 180;  // 최대 각도 제한
    directionRight = false;  // 방향 전환
  } else {
    currentAngle -= angleStep;  // 왼쪽으로 회전
    if (currentAngle < 0) currentAngle = 0;  // 최소 각도 제한
    directionRight = true;  // 방향 전환
  }
  servoMotor.write(currentAngle);  // 서보 모터 각도 설정
  Serial.print("Servo motor moved to ");
  Serial.print(currentAngle);
  Serial.println(" degrees.");
}

// 'bh' 명령 처리 함수
void executeBHCommand() {
  // 6번 핀을 2초간 ON과 동시에 카운트다운 시작
  Serial.println("Turning on pin 6 for 2 seconds and starting countdown.");
  digitalWrite(6, HIGH);

  // 카운트 다운 출력 (동시에 진행)
  for (int i = 3; i > 0; i--) {
    Serial.println(i);
    delay(1000);  // 1초 대기
  }

  // 2초 뒤에 릴레이 OFF
  digitalWrite(6, LOW);
  Serial.println("Pin 6 turned off.");
}
```
