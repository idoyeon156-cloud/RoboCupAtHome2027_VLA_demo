# Dual PiPER — 수정된 그리퍼 모델

시리얼 붓기 데모에서 사용한 양팔 URDF입니다. 양쪽 그리퍼에 플랜지와 새 베이스 메시를 적용했습니다. J7과 J8의 이동 한계는 각각 0~50 mm와 −50~0 mm입니다.

- URDF: [urdf/dual_piper.urdf](urdf/dual_piper.urdf)
- 메시: [meshes/](meshes/)
- 기존 모델: [collision](../../collision/piper_dual_description/) · [collision_configurable](../../collision_configurable/piper_dual_description/)

ROS 2 작업 공간의 `src/` 아래에 이 패키지 폴더를 복사한 후 작업 공간 루트에서 `colcon build --packages-select piper_dual_updated_gripper_description`을 실행하세요.
