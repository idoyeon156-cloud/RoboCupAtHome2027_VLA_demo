# Dual PiPER URDF — 교체 후

- ROS 패키지 이름: `piper_dual_updated_gripper_description`
- [URDF](urdf/dual_piper.urdf) · [STL 메시](meshes/)
- 기준 안전장치: [SW/updated_gripper](../../../SW/updated_gripper/README.md)

ROS 작업 공간의 `src/` 아래에 이 폴더를 복사하고 작업 공간 루트에서 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_dual_updated_gripper_description
source install/setup.bash
```

이 패키지는 모델만 제공합니다. 안전장치와 시뮬레이션은 SW 아래의 해당 작업 공간에서 실행합니다.

교체 후 모델은 새 그리퍼 베이스·장착 플랜지·양쪽 D405 마운트 v6와 총 100 mm 개방량을 적용합니다. J7은 0~50 mm, J8은 −50~0 mm입니다.

마운트는 고정 조립체의 충돌 외곽에 포함됩니다. 장착 좌표와 범위는 [SENSOR_MOUNT.md](../../../SW/updated_gripper/SENSOR_MOUNT.md)를 참고하세요.

두 베이스 뒷면과 정렬한 3030×500mm 수직 프로파일도 포함합니다. [프로파일 배치](../../../SW/updated_gripper/REAR_PROFILE.md)를 참고하세요.

알파 J5=-70°를 정확히 표현하도록 기존 ±1.22rad 반올림 한계를 ±70°(±1.2217304764rad)로 맞췄습니다.
