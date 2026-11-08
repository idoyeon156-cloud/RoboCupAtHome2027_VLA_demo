# RoboCup@Home 2027 VLA Tutorial

기능별로 폴더를 나누어 관리합니다.

| 폴더 | 내용 |
|---|---|
| [collision](collision/README.md) | Piper 듀얼암 충돌 방지: Pinocchio 관절 거리 제한, MoveIt 다면체 충돌 메시, 시뮬 시각화 및 실물 연결 (기존 박스 모드 포함) |

충돌 방지 관련 기존 파일은 모두 `collision/` 아래에 있습니다. 설치·빌드·실행은 이 폴더에서 진행합니다.

안전장치는 **두 단계**로 구성됩니다.

| 단계 | 방식 | 기본 기준 |
|---|---|---|
| 1차 | Pinocchio 조인트 거리 기반 충돌 방지 | 양팔 관절·그리퍼 중심 사이 거리 9cm 이하에서 제한 |
| 2차 | MoveIt 다면체 메시 충돌 검사 | 링크·그리퍼의 입체 형상 간 충돌 검사 |

실물용 통합 실행에서는 **목표 명령 → 1차 거리 검사 → 2차 메시 검사 → Piper 드라이버** 순서로 연결됩니다. 기본은 감시 모드이며, 명령 전달·차단 활성화 방법은 [충돌 방지 안내](collision/README.md)를 참고하세요.

```bash
cd collision
# 1차 조인트 거리 기반 시뮬
bash run_collision_visual_demo.sh

# 2차 MoveIt 다면체 메시 시뮬 (기본)
bash run_moveit_visual_demo.sh

# 실물용 — 1차 + 2차 통합 실행, 기본 감시 모드
bash run_moveit_real_guard.sh
```

다면체 메시를 양팔에 동일하게 적용하며, **팔 하나당 31개 조각, 양팔 총 62개**입니다. 기존 박스 수동 편집은 시뮬 실행 명령에 `geometry:=boxes editor:=true`를 추가하면 사용할 수 있습니다.

최초 설치와 나머지 실행 방법은 [충돌 방지 안내](collision/README.md)를 참고하세요.
