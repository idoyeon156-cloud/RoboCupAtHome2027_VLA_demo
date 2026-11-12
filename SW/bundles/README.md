# 안전장치 소스 ZIP

각 ZIP은 설정·URDF·STL·충돌 메시·ROS 소스·실행 스크립트를 포함한 독립 작업 공간입니다.
압축을 풀고 ZIP 안의 `README_START.md`를 따라 해당 기기에서 설치·빌드하세요.

| 폴더 | 기준 모델 |
| --- | --- |
| [original_gripper/](original_gripper/) | 교체 전 그리퍼, 총 개방량 70 mm |
| [updated_gripper/](updated_gripper/) | 교체 후 베이스·플랜지·양쪽 D405 마운트 v6·3030 기둥, 총 개방량 100 mm |

| 파일명에 들어가는 구분 | 실행 |
| --- | --- |
| `distance_rviz` | `bash run_distance_rviz.sh` |
| `meshes_rviz` | `bash run_meshes_rviz.sh` |
| `distance_real` | `bash run_distance_real.sh` |
| `meshes_real` | `bash run_meshes_real.sh` (거리 → 메시 두 단계) |

RViz 시뮬은 하나씩 실행합니다. 실물용 기본값은 감시 모드입니다.
소스 변경 후에는 각 작업 공간에서 `python tools/package_configurable.py`로 ZIP 4개를 다시 만들 수 있습니다.
씨리얼 연속 재생은 [../cereal_pour/](../cereal_pour/README.md) 소스를 사용하세요.

## 알파 기반 시리얼 파지·붓기

두 배포본은 같은 설정과 경로 생성기를 사용합니다. 알파는 양팔 `[0,90,0,0,-70,0]°`, 초기 물체 바닥은 베이스 장착면 Z=0, 그릇 파지 높이는 바닥에서 3.5cm입니다.

| 배포본 | 실행 |
| --- | --- |
| [시뮬레이션 ZIP](cereal_pour/dual_piper_cereal_pour_rviz.zip) | `bash run_cereal_rviz.sh` |
| [실물 배포 ZIP](cereal_pour/dual_piper_cereal_pour_real.zip) | `bash run_cereal_real.sh` |

실물용은 조인트 거리 → MoveIt 메시 가드를 사용하며 기본 monitor입니다. 물체 위치·크기는 `config/cereal_task.yaml`에서 수정합니다. 시작 서비스와 검증 범위는 [CEREAL_TASK.md](../cereal_pour/CEREAL_TASK.md)를 참고하세요.

재생성: `SW/cereal_pour`에서 `python tools/package_configurable.py --cereal`.
