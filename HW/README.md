# HW — 하드웨어 모델

URDF와 STL은 [URDF/](URDF/) 안에서 교체 전·후로 구분합니다.

```text
HW/
└── URDF/
    ├── original/           # 그리퍼 교체 전
    └── updated_gripper/    # 그리퍼 교체 후
```

| 모델 | 독립 ROS 패키지 |
| --- | --- |
| [교체 전](URDF/original/README.md) | `piper_dual_original_gripper_description` |
| [교체 후](URDF/updated_gripper/README.md) | `piper_dual_updated_gripper_description` |

안전장치와 씨리얼 시뮬레이션은 [SW](../SW/)에서 실행합니다.
