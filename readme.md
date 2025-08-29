# URDF Simulation Tutorial (ROS2 Humble)

본 패키지는 ROS2 Humble 기반의 URDF 및 Gazebo 시뮬레이션 실습 예제입니다.  
Diff drive 로봇을 URDF로 정의하고, Gazebo + RViz2 + ros2_control + teleop을 통합하여 제어할 수 있습니다.

---
제작자 : Changwoo-Han
\
제작일 : 2025/08/29
\
참고자료 : https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html

---
## ⚙️ 실행 방법
## 0. 준비
워크스페이스 빌드 및 환경 설정:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 1. Teleop_key 로 로봇 움직이기
#### 조건: use_stamped_vel: false 로 설정 후, 런치파일 재시작

첫 번째 터미널:
```bash
ros2 launch urdf_sim_tutorial 13-diffdrive.launch.py
```

두 번째 터미널:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. rqt_robot_steering 으로 로봇 움직이기
#### 조건: use_stamped_vel: true 로 설정(최초 다운시, true로 설정된 상태임) 후, 런치파일 재시작

첫 번째 터미널:
```bash
ros2 launch urdf_sim_tutorial 13-diffdrive.launch.py
```
RQT GUI 슬라이더를 조작하여 로봇 제어 가능

## 3. 패키지 구성
urdf/ : 로봇 URDF 및 Xacro 파일

launch/ : Gazebo, RViz2, ros2_control 관련 런치 파일

config/ : ros2_control 및 controller 설정 파일

twist_to_stamped_node.py : Twist → TwistStamped 변환 노드 (rqt 제어용)

videos/ : 실습 시연 영상 (직접 촬영본)
