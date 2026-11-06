# RoboCup@Home 2027 VLA Tutorial

기능별로 폴더를 나누어 관리합니다.

| 폴더 | 내용 |
|---|---|
| [collision](collision/README.md) | Piper 듀얼암 충돌 방지: Pinocchio 관절 거리 제한, MoveIt 충돌 박스, 시뮬 시각화 및 실물 연결용 프로토타입 |

충돌 방지 관련 기존 파일은 모두 `collision/` 아래에 있습니다. 설치·빌드·실행은 이 폴더에서 진행합니다.

```bash
cd collision
# MoveIt 충돌 박스 시뮬
bash run_moveit_visual_demo.sh
```

최초 설치와 나머지 실행 방법은 [충돌 방지 안내](collision/README.md)를 참고하세요.
