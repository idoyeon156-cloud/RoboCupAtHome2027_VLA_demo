#!/usr/bin/env python3
"""Four self-contained editable source bundles, excluding local environments/builds."""
import argparse
import hashlib
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
BUNDLES=[
 ('dual_piper_distance_rviz_configurable','run_distance_rviz.sh','조인트 거리 RViz 시뮬레이션'),
 ('dual_piper_distance_real_configurable','run_distance_real.sh','조인트 거리 실물용'),
 ('dual_piper_meshes_rviz_configurable','run_meshes_rviz.sh','MoveIt 다면체 RViz 시뮬레이션'),
 ('dual_piper_meshes_real_configurable','run_meshes_real.sh','조인트 거리 → MoveIt 다면체 통합 실물용')]


CEREAL_BUNDLES = [
 ('dual_piper_cereal_pour_rviz', 'run_cereal_rviz.sh', '양팔 알파·파지·붓기 RViz 시뮬레이션'),
 ('dual_piper_cereal_pour_real', 'run_cereal_real.sh', '양팔 알파·파지·붓기 실물용')]


def package(out, cereal=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    files=[]
    for directory in ('model','config','piper_dual_description','piper_collision_guard','piper_moveit_guard','test'):
        files += [p for p in (ROOT/directory).rglob('*') if p.is_file()
                  and not any(x in p.parts for x in ('__pycache__','.pytest_cache','simulation_sessions'))
                  and not any(x.endswith('.egg-info') for x in p.parts)
                  and p.name != 'last_completed_simulation.json'
                  and p.suffix not in ('.pyc','.pyo')]
    files += [p for p in ROOT.glob('run*.sh')]
    files += [ROOT/x for x in ('README.md','SENSOR_MOUNT.md','REAR_PROFILE.md','VALIDATION.md','requirements.txt',
               'tools/prepare_model.py','tools/prepare_driver_overlay.py','tools/generate_collision_boxes.py',
               'tools/generate_collision_meshes.py','tools/package_configurable.py','tools/real_control.py','tools/plan_cereal_task.py','CEREAL_TASK.md','EASY_START_KO.md','REAL_START_KO.md',
               'SIM_START_KO.md','ARMS_ONLY_KO.md','REAL_RUN_OBSERVATIONS_2026-11-13_KO.md','COLLISION_ANALYSIS_AND_COMMANDS_KO.md','INCIDENT_2026-11-13_KO.md',
               'POSE_MISMATCH_2026-11-13_KO.md','CAN_BINDING_KO.md','CHAT_HANDOFF_2026-11-13_KO.md','tools/read_session_logs.py')]
    if (ROOT/'REAL_MOTION_HOLD_KO.md').is_file():
        files.append(ROOT/'REAL_MOTION_HOLD_KO.md')
    if cereal:
        files += [ROOT/'SETUP.md', ROOT/'CEREAL_POUR.md', ROOT/'CEREAL_TASK.md', ROOT/'tools/plan_cereal_pour.py', ROOT/'tools/plan_cereal_task.py']
    for name,entry,label in (CEREAL_BUNDLES if cereal else BUNDLES):
        prefix=name+'/'
        start=f'''# {label} — 수정 가능한 버전

이 ZIP에는 신형 그리퍼와 양쪽 D405 마운트 v6를 적용한 설정·URDF/STL·다면체 메시·보호 구·재생성 도구와 실행 경로가 들어 있습니다. 마운트와 고정 3030 기둥의 형상·검증 범위는 SENSOR_MOUNT.md와 REAR_PROFILE.md를 참고하세요.
압축을 푼 뒤 이 안내 파일이 있는 폴더에서 아래 순서로 실행합니다.

## 최초 환경 준비 (Ubuntu 22.04 / ROS 2 Humble)

```bash
sudo apt install ros-humble-moveit ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-tf2-ros ros-humble-python-qt-binding python3-venv python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## 실행

```bash
bash {entry}
```

실행기가 이 폴더 안에 자동으로 빌드합니다. 실제 Piper 드라이버는 실행하지 않습니다.
실물용의 기본값은 감시 모드입니다. 확인 후 명령 전달 활성화는 README.md를 참고하세요.

## 변경 반영

1. config/robot.yaml에서 설치 위치·각도·최대 개방 폭·마운트 형상을 수정합니다.
2. 부품 외형이 바뀌면 model/dual_piper.source.urdf와 해당 원본 STL을 수정합니다.
3. 아래 명령으로 새 다면체와 관련 모델을 함께 생성한 뒤 재실행합니다.

```bash
.venv/bin/python tools/prepare_model.py
bash {entry}
```

시뮬 입력은 °/mm, 설정의 rpy는 rad, 길이는 m입니다. 상세 수정 항목은 README.md에 있습니다.
각 ZIP은 독립된 작업 공간입니다. 여러 ZIP을 따로 쓰면 수정한 설정·원본 모델을 각 작업 공간에도 동일하게 적용하고 재생성해야 합니다.
한 작업 공간 안에서는 시뮬과 실물용이 같은 생성 모델을 공유합니다. 시뮬은 하나씩 실행하세요.
'''
        if cereal:
            hold_notice = ('별도 REAL_MOTION_HOLD_KO.md가 포함돼 있어 임시 실물 차단도 적용됩니다.\n'
                           if (ROOT/'REAL_MOTION_HOLD_KO.md').exists() else '')
            start = f'''# {label} — 팔 전용 구동 확인 배포

2026-11-13: 그리퍼 제외 상태의 전체 시퀀스와 제로 복귀를 확인한 현재 소스입니다.
속도·끊김·지터링을 일부 개선했으나 잔여 떨림과 그리퍼 활성화 문제는 조사 중입니다.
실제 물체 파지·붓기나 충돌·제동 성능 검증 완료를 뜻하지 않습니다.
{hold_notice}
**안전상 먼저 같은 작업 폴더에서 시뮬을 끝까지 완료해야 실물을 실행할 수 있습니다.
안정화 후 시뮬 없이 실행할 수 있도록 변경할 예정이며, 현재는 선행 조건을 유지합니다.**
소스·모델·설정·속도가 같으면 완료 기록을 재사용합니다. 새 폴더/기기 또는 변경 후에는 다시 완료합니다.

## 최초 준비

Ubuntu 22.04 / ROS 2 Humble 의존성은 SETUP.md를 참고하세요.

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## 먼저 시뮬 완료

이 README_START.md가 있는 폴더에서, 실물용 ZIP도 동일하게 실행합니다.

```bash
bash run_cereal_rviz.sh arms_only:=true
```

`SIMULATION COMPLETE` 후 종료합니다. 중간 종료는 완료가 아닙니다.
시뮬 완료 기록은 배포하지 않으며 다른 폴더의 기록을 복사해서 사용하지 않습니다.

## 실물 — 각기 별도 터미널

물체 없이 양팔 관절 영점·그리퍼 닫힘 상태에서 시작합니다. 외부 PiPER ROS 드라이버 설치가 필요합니다.

```bash
# 터미널 1: 관절 활성화, 그리퍼 CAN 명령 생략
bash run_piper_driver.sh can0 can1 --arms-only
```

```bash
# 터미널 2: 두 enforce 가드와 완료 경로
bash run_cereal_real.sh enforce:=true geometry_confirmed:=true arms_only:=true
```

```bash
# 터미널 3: PASS 후 전체 실행
bash run_real_control.sh check
bash run_real_control.sh run
```

제로 → 알파 → 가상 파지·붓기 경로 → 알파 → 제로를 연속 실행합니다.
`Finished; holding return_home`가 완료입니다. CAN은 로봇 뒤에서 본 왼팔 can0 / 오른팔 can1입니다.
현재 실행 기준은 README.md와 ARMS_ONLY_KO.md입니다. 과거 사고 문서는 조사 당시의 기록입니다.
'''
        with zipfile.ZipFile(out/(name+'.zip'),'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
            for p in sorted(set(files)):
                z.write(p,prefix+p.relative_to(ROOT).as_posix())
            z.writestr(prefix+'README_START.md',start)
            z.writestr(prefix+'SOURCE_SHA256.txt','\n'.join(
                hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(ROOT).as_posix()
                for p in sorted(set(files)))+'\n')
        with zipfile.ZipFile(out/(name+'.zip')) as z:
            if z.testzip() is not None: raise RuntimeError('ZIP integrity failure')
        print(name+'.zip', (out/(name+'.zip')).stat().st_size, 'bytes')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=ROOT.parent/'bundles/cereal_pour')
    parser.add_argument('--cereal', action='store_true', help='Build the alpha-based cereal simulation and real deployment ZIPs')
    args = parser.parse_args()
    package(args.output, args.cereal)
