# 새 그리퍼·D405 마운트·3030 기둥 — YAML 기반 시리얼 작업

물체 위치·크기를 [config/cereal_task.yaml](config/cereal_task.yaml)에서 입력합니다. 알파 `[0,90,0,0,-70,0]°` 이후 파지·붓기·복귀 관절 경로를 IK와 충돌 검사로 생성하며 RViz와 실물 배포용이 같은 설정을 사용합니다.

```bash
cd SW/cereal_pour
bash run_meshes_rviz.sh cereal:=true   # RViz 연속 재생
bash run_cereal_real.sh               # 실물용, 기본 monitor
```

YAML 수정 후 다시 실행하면 경로를 재생성합니다. 설치는 [SETUP.md](SETUP.md), 좌표계·설정·서비스·검증 범위는 [CEREAL_TASK.md](CEREAL_TASK.md)를 참고하세요. 실물 드라이버는 별도이며 prepare/start 서비스 요청으로 동작합니다. 그릇 바닥 중심 예시는 world `[0.49,0.115,0.0]m`입니다.

표시 모델과 MoveIt 모델은 교체 그리퍼·양쪽 마운트·고정 기둥을 포함합니다. 설치 모델 변경은 `config/robot.yaml`과 `model/dual_piper.source.urdf`에서 수행하고 `tools/prepare_model.py`로 생성합니다.

- [D405 장착 좌표·외곽 모델](SENSOR_MOUNT.md)
- [3030 배치·검사 범위](REAR_PROFILE.md)
- [기존 고정 관절각 시퀀스 기록](CEREAL_POUR.md)
- [독립 URDF 패키지](../../HW/URDF/updated_gripper)

검증: 예시 전체 경로 49개 경유점 생성, 0.5° 이하 보간의 1,800개 상태에서 거리·볼록 메시·물체 충돌 검사 통과. 한 주기 약 73.5초. 마찰·파지력·유체를 계산하는 물리 시뮬레이션은 아닙니다.
