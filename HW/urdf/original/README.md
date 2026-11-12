# Dual PiPER URDF — 교체 전

- ROS 패키지 이름: `piper_dual_original_gripper_description`
- [URDF](urdf/dual_piper.urdf) · [STL 메시](meshes/)
- 기준 안전장치: [SW/original_gripper](../../../SW/original_gripper/README.md)

ROS 작업 공간의 `src/` 아래에 이 폴더를 복사하고 작업 공간 루트에서 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_dual_original_gripper_description
source install/setup.bash
```

이 패키지는 모델만 제공합니다. 안전장치와 시뮬레이션은 SW 아래의 해당 작업 공간에서 실행합니다.

교체 전 모델의 총 그리퍼 개방량은 70 mm입니다.
