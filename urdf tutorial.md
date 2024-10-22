# - URDF

## -  URDF history


(참고 : https://with-rl.tistory.com/entry/URDF%EB%A5%BC-%EC%9D%B4%EC%9A%A9%ED%95%9C-%EA%B0%84%EB%8B%A8%ED%95%9C-%EB%A1%9C%EB%B4%87-%EB%A7%8C%EB%93%A4%EA%B8%B0-1)


### - Terminal 1


9  source /opt/ros/humble/setup.bash
53  sudo apt install ros-humble-xacro
35  rm -rf build install log
50  colcon build --packages-select urdf_tutorial
51  source install/setup.bash
54  ros2 launch urdf_tutorial robot_1.launch.py


### - Terminal 2


1  cd ros2_ws/
2  source install/setup.bash
3  rviz2
4  source /opt/ros/humble/setup.bash~
5  source /opt/ros/humble/setup.bash
6  rviz2 


### - 실행 결과


![image](https://github.com/user-attachments/assets/c64b2440-2d2b-446f-9b94-a8caa706629f)
