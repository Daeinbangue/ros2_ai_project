# ROS2 MoveIt2 기반 로봇 제어 과제

1. 프로젝트 개요
본 프로젝트는 ROS2 Humble 환경에서 MoveIt2와 RViz를 이용하여  
로봇 암을 제어하는 GUI 기반 애플리케이션을 구현하는 과제입니다.

PyQt GUI를 통해 목표 자세를 입력하고,  
MoveIt2 기반 Motion Planning 및 Trajectory Execution을 수행합니다.

---



2. 개발 환경
- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- Python 3
- PyQt5



3. 패키지 구조
---
ros2_ws/
├── src/
│ └── my_ros2_assignment/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/
│ │ └── my_ros2_assignment
│ └── my_ros2_assignment/
│ ├── init.py
│ └── my_node.py
└── requirements.txt

---



4. 실행 방법

4.1 install
pip install -r requirements.txt

4.2 Build
cd ros2_ws
colcon build
source install/setup.bash

4.3 Launch
ros2 run my_ros2_assignment my_node



5. 구현 내용

- PyQt 기반 GUI 구현

- MoveIt2 (OMPL, RRTConnect) 기반 Motion Planning

- RViz Interactive Marker 기반 제어

- ros2_control Trajectory 실행



6.  현재 이슈 및 분석

6.1 Planning 실패 현상

- 일부 목표 자세에 대해 다음과 같은 MoveIt2 에러가 발생함:

Unable to sample any valid states for goal tree


6.2 원인 분석

- 목표 Pose가 로봇의 IK 해 공간 외부에 위치

- Orientation constraint로 인한 샘플링 공간 축소

- Joint limit 및 Self-collision 조건에 의한 유효 상태 부족

- Planner sampling timeout 제한


6.3 확인 사항

- RViz에서 Interactive Marker 기반 직접 조작 시 정상적인 Planning 및 Execution 가능

- Joint Space Trajectory 실행은 정상 동작

- 문제는 GUI → Cartesian Pose → IK 변환 구간에 집중됨



7. 향후 개선 방향

- 목표 Pose 유효성 사전 검사 (IK feasibility check)

- Orientation constraint 완화 옵션 추가

- Joint-space 기반 상대 이동 방식 병행

- Planning timeout 및 sampling parameter 조정



8. 결론

본 과제에서는 ROS2 기반 로봇 제어 파이프라인을 구성하고
MoveIt2의 Planning 구조 및 한계를 분석하는 데 중점을 두었습니다.
Planning 실패 이슈 또한 MoveIt2 내부 동작 원리 이해를 통해 원인을 명확히 파악하였습니다.
