# Dual PiPER · 신형 그리퍼 + D405

[URDF](urdf/dual_piper.urdf) · ROS 패키지: `piper_dual_d405_description`

기존 `updated_gripper`를 기준으로 뒤쪽 3030×500 mm 카메라 기둥을 제거하고,
양쪽 v6 센서 마운트에 RealSense D405 본체를 추가한 별도 모델입니다.
팔 배치(베이스 간격 175 mm), 관절 원점·축·한계, 100 mm 그리퍼와 마운트 위치는 같습니다.

## 사용

이 폴더를 ROS workspace의 `src/`에 복사하고 workspace 루트에서 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_dual_d405_description
source install/setup.bash
```

메시가 포함된 패키지 전체를 사용하세요. URDF 안의 `package://` 경로는 이 패키지를 참조합니다.

## 카메라 위치와 프레임

기존 마운트 CAD의 `tcp.json`에 기록된 명목 RGB/depth 위치를 사용합니다.
마운트 H 좌표에서 카메라 링크는 `(0, 52.243, 72.337)` mm,
회전 RPY는 `(0, 20.6, 90)` 도입니다. TCP 기준 뒤 102.657 mm, 위 74.737 mm입니다.
H→그리퍼 변환은 기존 마운트 고정 조인트를 그대로 따릅니다.

각 팔에 `left_d405_link` / `right_d405_link`와 depth, color, infra1, infra2 프레임 및
각 optical 프레임을 추가했습니다. 카메라 링크는 X 전방·Y 좌측·Z 위쪽,
optical 프레임은 Z 전방·X 우측·Y 아래쪽입니다.
프레임 관계는 RealSense 공식 D405 xacro의 명목값을 사용합니다.
실물 캘리브레이션 값은 아닙니다. 드라이버와 함께 TF를 발행할 때는 이름을 맞추고
같은 자식 프레임을 robot_state_publisher와 카메라 드라이버가 중복 발행하지 않도록 설정하세요.

## 형상과 범위

- 공식 D405 STL을 포함합니다. 공식 URDF 기준 충돌 상자는 23×42×42 mm, 질량은 58 g입니다.
- USB 케이블·커넥터와 체결 볼트는 포함하지 않습니다.
- 기둥의 링크·고정 조인트·충돌 형상은 제거했습니다.
- 기존 SW 안전장치와 관제센터가 자동으로 이 모델을 사용하는 것은 아닙니다.
  이 패키지는 독립 URDF이며, 실물 충돌 방지에 사용하려면 해당 SW 모델·가드도 별도로 갱신해야 합니다.

## 출처

- 팔·그리퍼·마운트: `../updated_gripper/urdf/dual_piper.urdf`와 해당 메시.
- 마운트 좌표: `reference/tcp.json`, `reference/native.json`.
- 카메라: [공식 RealSense D405 모델](https://github.com/IntelRealSense/realsense-ros/blob/489265d328e9011efc83c6ee405abfadccc24e36/realsense2_description/urdf/_d405.urdf.xacro).
- 카메라 메시·공식 xacro는 Apache-2.0입니다. `LICENSE.realsense`와 원본 저작권 고지를 포함합니다.
  원본 xacro를 정적 URDF로 펼치고 마운트 연결 좌표와 프레임 접두어를 적용했습니다.
- 파일 해시와 고정한 공식 소스 커밋은 `reference/sources.json`에 기록했습니다.
