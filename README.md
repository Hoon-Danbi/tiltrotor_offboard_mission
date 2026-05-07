# Tiltrotor Offboard Mission

PX4 드론 자율비행 ROS 패키지

## 환경

- Ubuntu 18.04 (Bionic)
- ROS Melodic
- PX4 v1.13.3
- Gazebo 9
- MAVROS 1.16.0

## 설치

\`\`\`bash
cd ~/catkin_ws/src
git clone https://github.com/Hoon-Danbi/tiltrotor_offboard_mission.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
\`\`\`

## 실행

### 1. PX4 시뮬레이션 실행
\`\`\`bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
\`\`\`

### 2. MAVROS 실행
\`\`\`bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
\`\`\`

### 3. 이륙 노드 실행
\`\`\`bash
rosrun tiltrotor_offboard_mission takeoff.py
\`\`\`

## 노드 설명

### takeoff.py
- AUTO.TAKEOFF 모드로 변경
- 시동 ON
- 자동 이륙 (기본 5m)
