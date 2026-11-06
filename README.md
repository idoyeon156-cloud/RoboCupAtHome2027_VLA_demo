# RoboCup@Home 2027 VLA Tutorial

## Piper 듀얼암 모델

### 충돌 제한 시각적으로 보는 데모 (RViz2)

**실제 로봇을 연결하지 않고**, 마우스로 양팔 관절 슬라이더를 움직여 거리 제한이 걸리는 자세를 확인하는 데모입니다. 허용 자세와 반투명 요청 자세, 보호 구, 최소 간격, 제한 상태를 함께 표시합니다.

**[설치·실행 방법 → 충돌 제한 시각화 데모](piper_collision_guard/VISUAL_DEMO.md)**

설치·빌드 후 저장소 폴더에서 `bash run_collision_visual_demo.sh`로 시작합니다.

실행 파일: `collision_limit_visual_demo.launch.py`. 물리·제동 시뮬레이션이나 MoveIt 메시 충돌 검사는 아직 포함하지 않습니다.

같은 방향을 바라보는 그리퍼 포함 Piper 두 대를 좌우로 배치한 모델입니다.
마주 보는 베이스 껍데기 표면 사이 간격 약 95mm와 원본 STL의 베이스 폭 약 80mm를 기준으로 배치했습니다.
베이스 원점 간 거리는 175mm이며, world 기준 왼팔은 Y=+87.5mm, 오른팔은 Y=-87.5mm입니다.

| 파일 | 용도 |
|---|---|
| [통합 URDF](piper_dual_description/urdf/dual_piper.urdf) | ROS2 / Pinocchio / MoveIt 로봇 모델 |
| [원본 메시](piper_dual_description/meshes) | URDF가 참조하는 STL 10개 |
| [Fusion 360 보기용 STL](preview/dual_piper_zero_pose_mm.stl) | 모든 관절이 0인 자세로 배치한 양팔 정적 메시, 단위 mm |
| [상세 설명](piper_dual_description/README.md) | 좌표, 관절 매핑, 설치, 검증 범위 |
| [보기용 STL 생성 코드](tools/export_preview.py) | URDF visual 메시를 원점 자세로 변환·결합 |

### Windows / Fusion 360에서 보기

1. 이 저장소의 **Code → Download ZIP**으로 전체 파일을 다운로드하고 압축을 풉니다.
2. Fusion 360에서 새 디자인을 엽니다.
3. **Insert → Insert Mesh (삽입 → 메시 삽입)**를 선택합니다.
4. `preview/dual_piper_zero_pose_mm.stl`을 선택하고 **단위를 Millimeter (mm)**로 지정합니다.
5. 배율 1로 삽입하고 전체 모델이 보이도록 화면을 맞춥니다.

Fusion은 URDF를 기본 가져오기 형식으로 지원하지 않습니다. 보기용 STL에는 양팔 배치와 외형만 들어 있고 관절, 질량, ROS 정보 또는 충돌 방지 기능은 없습니다. 관절을 조절하며 모델을 보려면 URDF 뷰어/RViz를 사용하거나 별도의 URDF 가져오기 애드인이 필요합니다. 모든 관절 0은 모델의 기준 자세이며 실제 운용 권장 자세가 아닙니다.

공식 참고: [Fusion 지원 형식](https://help.autodesk.com/view/fusion360/ENU/?guid=TPD-SUPPORTED-FILE-FORMATS), [파일 가져오기](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-import-or-open-a-file-in-Autodesk-Fusion-360.html).

거리 보호 구는 원본 STL 경계 상자와 손가락 전체 개폐 범위를 덮도록 수정했습니다. 왼팔 J1 +15°/나머지 0° 자세를 1차 장치에서 차단하며, 시뮬과 실기 노드가 같은 설정을 사용합니다.

### 현재 구현 상태

- 양팔 URDF와 그리퍼, 원본 visual/collision 메시: 포함.
- [Pinocchio 기반 1차 거리 제한 노드](piper_collision_guard/README.md): 감시/명령 게이트 첫 구현, 실제 팔 정지 미검증.
- [2차 MoveIt 직육면체 충돌 검사](piper_moveit_guard/README.md): 좌우 링크 부피 검사, 양팔 이동 구간 표본 검사, 1차 장치와 직렬 명령 차단.
- MoveIt 전용 모델: 링크별 직육면체 20개, 면당 5mm 예시 여유. 원본 visual/STL은 유지.
- 원본 베이스 메시 Y 폭 약 80mm이므로 이 배치에서 메시 외곽 간격은 약 95mm. 실제 외함 폭과 돌출부는 실물 대조가 필요함.

### 원본과 라이선스

로봇 정의와 메시 출처: [agilexrobotics/piper_ros, humble, 017ffefa](https://github.com/agilexrobotics/piper_ros/tree/017ffefa64511bc6325bd77ddc4e16065c152051).
원본 [MIT LICENSE](piper_dual_description/LICENSE)를 포함했습니다. Fusion용 파생 메시에도 동일한 원본 라이선스를 적용합니다.
