# Piper 듀얼암 충돌 방지

이 폴더에는 1차 Pinocchio 관절 거리 제한과 2차 MoveIt 다면체 충돌 메시의 시뮬용·실물 연결용 코드가 들어 있습니다.
저장소를 받은 뒤 `cd collision`로 들어와 설치·빌드·실행하세요.

- [1차 거리 제한 시뮬 설치·실행](piper_collision_guard/VISUAL_DEMO.md)
- [1차 거리 제한 실물 연결](piper_collision_guard/README.md)
- [2차 MoveIt 시뮬·실물 설치·실행](piper_moveit_guard/README.md)

기존 위치에서 만든 `build/`, `install/`은 재사용하지 말고 이 폴더에서 다시 빌드하세요.

## MoveIt 실행 방식

기본은 **구간별 볼록 다면체 메시**입니다. 원본 STL의 사선·곡면을 따라 베이스, 팔, 손목, 그리퍼 몸체와 손가락에 적용합니다. **팔 하나당 31개 조각, 양팔 총 62개**이며 시뮬용과 실물용이 같은 형상을 사용합니다. 지지 평면을 바깥으로 5mm(0.5cm) 이동한 여유가 메시 자체에 포함됩니다. 모서리 방향의 여유는 5mm보다 클 수 있습니다.

```bash
# 기본 다면체 시뮬 — 관절 조절기 + RViz
bash run_moveit_visual_demo.sh

# 기본 다면체 실물용 — 감시 모드
bash run_moveit_real_guard.sh

# 기존 박스 모드 — 수동 편집 창 포함
bash run_moveit_visual_demo.sh geometry:=boxes editor:=true

# 기존 박스 모드 — 실물용
bash run_moveit_real_guard.sh geometry:=boxes
```

다면체 기본 모드에는 박스 편집 창이 열리지 않습니다. 형상을 바꾸려면 생성 도구의 분할 기준을 수정하거나 여유를 지정해 메시를 재생성한 뒤 다시 빌드·실행합니다.

```bash
python3 tools/generate_collision_meshes.py --margin 0.005
```

기존 **박스 모드의 수동 편집 기능도 유지**합니다. 박스 편집기로 저장한 `collision_tuning.json`은 **`geometry:=boxes`에서만** 자동으로 읽으며, 기본 다면체 모드에는 적용되지 않습니다. 다른 폴더의 박스 설정은 다음처럼 지정합니다.

```bash
bash run_moveit_real_guard.sh geometry:=boxes collision_config:=/절대/경로/collision_tuning.json
```

실물용은 실행할 때 형상을 읽습니다. 기본 감시 모드는 상태만 알리며, 명령 전달·차단을 켜려면 `enforce:=true geometry_confirmed:=true`를 추가합니다. 드라이버는 별도로 실행합니다.

## 실물과 시뮬의 관절 기준

시뮬의 0°는 URDF에 정의된 관절 기준입니다. 실물 피드백도 같은 관절 기준으로 해석해야 하며, 현재 코드는 팔별 영점 차이를 자동으로 보정하거나 SDK의 영점을 다시 저장하지 않습니다. 관절별 오프셋 보정은 아직 구현하지 않았습니다.

## Piper 듀얼암 모델

### 충돌 제한 시각적으로 보는 데모 (RViz2)

**실제 로봇을 연결하지 않고**, 마우스로 양팔 관절 슬라이더를 움직여 거리 제한이 걸리는 자세를 확인하는 데모입니다. 허용 자세와 반투명 요청 자세, 보호 구, 최소 간격, 제한 상태를 함께 표시합니다.

**[설치·실행 방법 → 충돌 제한 시각화 데모](piper_collision_guard/VISUAL_DEMO.md)**

설치·빌드 후 `collision/` 폴더에서 `bash run_collision_visual_demo.sh`로 시작합니다.

1차 거리 제한 데모의 실행 파일은 `collision_limit_visual_demo.launch.py`입니다. MoveIt 메시 충돌 검사 데모는 위의 `run_moveit_visual_demo.sh`로 별도 실행합니다.

같은 방향을 바라보는 그리퍼 포함 Piper 두 대를 좌우로 배치한 모델입니다.
마주 보는 베이스 껍데기 표면 사이 간격 약 95mm와 원본 STL의 베이스 폭 약 80mm를 기준으로 배치했습니다.
베이스 원점 간 거리는 175mm이며, world 기준 왼팔은 Y=+87.5mm, 오른팔은 Y=-87.5mm입니다.

| 파일 | 용도 |
|---|---|
| [통합 URDF](piper_dual_description/urdf/dual_piper.urdf) | ROS2 / Pinocchio / MoveIt 로봇 모델 |
| [원본 메시](piper_dual_description/meshes) | URDF가 참조하는 STL 10개 |
| [Fusion 360 보기용 STL](preview/dual_piper_zero_pose_mm.stl) | 모든 관절이 0인 자세로 배치한 양팔 정적 메시, 단위 mm |
| [상세 설명](piper_dual_description/README.md) | 좌표, 관절 매핑, 설치 |
| [보기용 STL 생성 코드](tools/export_preview.py) | URDF visual 메시를 원점 자세로 변환·결합 |

### Windows / Fusion 360에서 보기

1. 이 저장소의 **Code → Download ZIP**으로 전체 파일을 다운로드하고 압축을 풉니다.
2. Fusion 360에서 새 디자인을 엽니다.
3. **Insert → Insert Mesh (삽입 → 메시 삽입)**를 선택합니다.
4. `preview/dual_piper_zero_pose_mm.stl`을 선택하고 **단위를 Millimeter (mm)**로 지정합니다.
5. 배율 1로 삽입하고 전체 모델이 보이도록 화면을 맞춥니다.

Fusion은 URDF를 기본 가져오기 형식으로 지원하지 않습니다. 보기용 STL에는 양팔 배치와 외형만 들어 있고 관절, 질량, ROS 정보 또는 충돌 방지 기능은 없습니다. 관절을 조절하며 모델을 보려면 URDF 뷰어/RViz를 사용하거나 별도의 URDF 가져오기 애드인이 필요합니다. 모든 관절 0은 모델의 기준 자세이며 실제 운용 권장 자세가 아닙니다.

공식 참고: [Fusion 지원 형식](https://help.autodesk.com/view/fusion360/ENU/?guid=TPD-SUPPORTED-FILE-FORMATS), [파일 가져오기](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-import-or-open-a-file-in-Autodesk-Fusion-360.html).

1차 거리 장치의 기본값은 관절 중심 거리 9cm입니다. 반지름 4cm 보호 구 표면 사이 1cm 이하에서 제한하며 시뮬·실기 노드가 같은 기준을 사용합니다. 링크 부피 검사는 2차 MoveIt에서 수행합니다.

### 현재 구현 상태

- 양팔 URDF와 그리퍼, 원본 visual/collision 메시: 포함.
- [Pinocchio 기반 1차 거리 제한 노드](piper_collision_guard/README.md): 관절 중심 거리 감시 및 명령 차단.
- [2차 MoveIt 다면체 충돌 검사](piper_moveit_guard/README.md): 좌우 링크 부피 검사, 양팔 이동 구간 표본 검사, 1차 장치와 직렬 명령 차단.
- MoveIt 기본 모델: 양팔 전체에 구간별 볼록 다면체 62개(한 팔 31개). 사선·곡면을 따라 생성한 메시와 5mm 예시 여유를 시뮬·실물용에서 공유합니다. 원본 visual/STL은 유지합니다. 다면체 검사는 MoveIt/Bullet, 기존 박스 모드는 MoveIt/FCL을 사용하며 `geometry:=boxes`로 선택합니다.
- 원본 베이스 메시 Y 폭 약 80mm이므로 이 배치에서 메시 외곽 간격은 약 95mm. 실제 외함 폭과 돌출부는 실물 대조가 필요함.

### 원본과 라이선스

로봇 정의와 메시 출처: [agilexrobotics/piper_ros, humble, 017ffefa](https://github.com/agilexrobotics/piper_ros/tree/017ffefa64511bc6325bd77ddc4e16065c152051).
원본 [MIT LICENSE](piper_dual_description/LICENSE)를 포함했습니다. Fusion용 파생 메시에도 동일한 원본 라이선스를 적용합니다.
