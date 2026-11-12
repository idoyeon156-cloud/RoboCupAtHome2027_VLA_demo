# 새 그리퍼 · 시리얼 붓기 RViz 연속 재생

2026-11-12 작업 백업의 수정 모델과 시리얼 시퀀스를 별도 작업 공간으로 추가했습니다. 교체 후 그리퍼 모델을 사용하는 독립 작업 공간입니다.

## 실행

Ubuntu 22.04 / ROS 2 Humble 환경에서 이 폴더로 이동한 후 [SETUP.md](SETUP.md)의 처음 설치 절차에 따라 의존성을 설치하세요. 모델과 충돌 메시 생성물은 포함되어 있습니다.

```bash
cd SW/cereal_pour
bash run_meshes_rviz.sh cereal:=true
```

영점 2초 대기 → 왼팔 그릇 유지 자세 → 오른팔 대기 → 정렬 → 25° → 45° → 65° → 1초 유지 → 역순 복귀 → 양팔 영점을 반복합니다. 종료는 Ctrl+C입니다. 실제 시리얼 입자나 유체를 계산하지 않는 관절 동작 시각화이며 기본 시퀀스는 빈 그리퍼 기준입니다.

자동 재생은 로컬 ROS 도메인 178에서 실행하며 실물 드라이버 명령을 발행하지 않습니다. 같은 ROS 패키지 이름을 사용하는 기존 작업 공간과 별도 셸에서 실행하세요.

## 주요 파일

- [cereal_preview.py](piper_collision_guard/piper_collision_guard/cereal_preview.py): 연속 RViz 재생
- [cereal_pour.yaml](piper_collision_guard/config/cereal_pour.yaml): 자세·속도·유지 시간
- [dual_piper.urdf](piper_dual_description/urdf/dual_piper.urdf): 새 그리퍼 표시 모델
- [dual_piper_meshes.urdf](piper_moveit_guard/config/dual_piper_meshes.urdf): MoveIt 다면체 충돌 모델
- [독립 URDF 패키지](../../HW/URDF/updated_gripper): 새 그리퍼 베이스·플랜지 STL 포함
- [CEREAL_POUR.md](CEREAL_POUR.md): 원본 작업의 상세 시퀀스·물체 설정·실물 경로 참고

이 폴더는 시리얼 작업 백업의 소스를 보존합니다. 미리보기용 단일 합성 STL과 기존 배포 ZIP, 빌드·가상환경 파일은 제외했습니다. 실행에 필요한 URDF, STL, YAML, ROS 패키지, 도구는 포함합니다.

## 이번 업로드의 확인 범위

2026-11-12: 모델 manifest 해시 50개 일치, URDF 메시 참조 240개 해소, Python·Bash 구문 검사, 기본 시퀀스의 관절 한계 및 영점 반복 연결을 확인했습니다. 기본 한 주기는 약 54.56초입니다. 이 환경에는 Pinocchio가 없어 ROS 빌드 및 RViz 재생은 재실행하지 않았습니다. VALIDATION.md는 원본 백업의 과거 검증 기록입니다.
