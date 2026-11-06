# Piper 듀얼암 URDF

## 제공 파일

- `piper_dual_description/urdf/dual_piper.urdf`: 그리퍼를 포함한 통합 URDF.
- `piper_dual_description/meshes/`: URDF에서 참조하는 원본 STL 10개. 양팔이 공유.
- `piper_dual_description/package.xml`, `CMakeLists.txt`: ROS2 ament_cmake description 패키지.

## 원본과 변경 사항

원본: https://github.com/agilexrobotics/piper_ros/blob/017ffefa64511bc6325bd77ddc4e16065c152051/src/piper_description/urdf/piper_description.urdf

요청한 humble 브랜치에서 확인한 커밋 기준. `gripper_base`, `link7`, `link8`과 prismatic `joint7`, `joint8`이 있어 그리퍼 포함 버전이다. 원본 질량, 관성, 관절 축, 제한, visual/collision 형상을 유지했다. 이름에 left_/right_를 붙이고 양팔 베이스를 공통 world에 연결했다. 충돌 형상 확대나 SRDF 충돌 제외 설정은 아직 하지 않았다.

## 배치

로봇 앞쪽을 +X, 로봇 기준 왼쪽을 +Y, 위쪽을 +Z로 둔다. world는 양팔 베이스 원점의 중간이다.

| 베이스 | xyz (m) | rpy (rad) |
|---|---|---|
| left_base_link | 0, +0.0875, 0 | 0, 0, 0 |
| right_base_link | 0, -0.0875, 0 | 0, 0, 0 |

실물에서 서로 마주 보는 베이스 껍데기 표면 사이 간격을 약 **9.5cm**로 측정한 값을 사용한다. 원본 `base_link.STL`의 Y 폭 약 8cm를 더해 베이스 원점 간격을 **17.5cm**로 배치했다. 두 팔은 같은 방향과 높이로 설치하고 순수 좌우로 떨어져 있다고 가정한다.

이는 STL 외형을 기준으로 측정값에 맞춘 배치다. 실제 외함과 STL의 폭이나 돌출부가 다르면 실제 원점 간격도 달라지므로, 충돌 제한에 사용하기 전에 실물과 RViz/FK 위치를 대조한다. 원본 메시 자체를 확대하지 않았으며 추가 충돌 여유는 포함하지 않는다.

간격 변경: URDF 맨 끝이 아니라 각 팔 뒤에 있는 `world_to_left_base`, `world_to_right_base`의 origin y를 수정한다. 전체 원점 간격 D에 대해 +D/2, -D/2를 사용한다.

## 관절 이름과 그리퍼

- 왼팔: left_joint1 ... left_joint6, left_joint7, left_joint8.
- 오른팔: right_joint1 ... right_joint6, right_joint7, right_joint8.
- fixed joint: 각 팔의 joint6_to_gripper_base 및 world 연결.
- 총 21 links, 20 joints (revolute 12, prismatic 4, fixed 4).

원본의 두 손가락 조인트를 그대로 보존했으며 mimic은 추가하지 않았다. 원본에서 joint7은 [0, 0.035]m, joint8은 [-0.035, 0]m다. 대칭 개폐를 모델에 표현하려면 각 팔에서 joint8 = -joint7로 두 값을 함께 채운다. 실제 SDK가 주는 전체 개구 폭과 모델의 손가락 이동량 변환은 장치에서 확인해야 한다. 실제 드라이버의 joint1...joint7 이름을 통합 모델의 좌우 이름으로 매핑하고 두 번째 손가락 상태도 채워야 한다.

## ROS2 Humble에서 사용

`piper_dual_description` 폴더를 ROS2 작업공간의 src 아래에 복사한 후 작업공간 루트에서 실행:

```bash
colcon build --packages-select piper_dual_description
source install/setup.bash
```

robot_state_publisher, Pinocchio, MoveIt에 동일한 `urdf/dual_piper.urdf`를 로드한다. `package://piper_dual_description/meshes/...`를 해석하려면 이 패키지를 빌드하고 setup.bash를 source해야 한다. Pinocchio를 직접 쓰는 경우 해당 작업공간의 install/share를 패키지 검색 경로에 전달할 수 있다.

이 패키지는 모델만 제공하며 로봇을 움직이는 코드, 드라이버 launch, MoveIt SRDF 또는 충돌 제한 노드는 포함하지 않는다.

## 검증

XML 구문, 이름 중복, 링크 연결의 단일 트리, 참조 링크/메시 존재, 원본 대비 각 팔 내용 보존, 고정 변환과 베이스 메시 간격을 검사했다. ROS2/Pinocchio 로딩과 RViz 렌더는 실행 환경에서 별도 확인이 필요하다.

## 라이선스

메시와 로봇 정의는 원본 저장소의 MIT 라이선스를 따른다. 원본 LICENSE를 패키지에 포함했다.
