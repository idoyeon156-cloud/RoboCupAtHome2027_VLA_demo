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


CEREAL_BUNDLE = ('dual_piper_cereal_pour_real', 'run_cereal_real.sh', '양팔 시리얼 붓기 실물용')


def package(out, cereal=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    files=[]
    for directory in ('model','config','piper_dual_description','piper_collision_guard','piper_moveit_guard','test'):
        files += [p for p in (ROOT/directory).rglob('*') if p.is_file()
                  and not any(x in p.parts for x in ('__pycache__','.pytest_cache'))
                  and not any(x.endswith('.egg-info') for x in p.parts)
                  and p.suffix not in ('.pyc','.pyo')]
    files += [p for p in ROOT.glob('run*.sh')]
    files += [ROOT/x for x in ('README.md','SENSOR_MOUNT.md','VALIDATION.md','requirements.txt',
               'tools/prepare_model.py','tools/generate_collision_boxes.py',
               'tools/generate_collision_meshes.py','tools/package_configurable.py')]
    if cereal:
        files += [ROOT/'CEREAL_POUR.md', ROOT/'tools/plan_cereal_pour.py']
    for name,entry,label in ([CEREAL_BUNDLE] if cereal else BUNDLES):
        prefix=name+'/'
        start=f'''# {label} — 수정 가능한 버전

이 ZIP에는 신형 그리퍼와 양쪽 D405 마운트 v6를 적용한 설정·URDF/STL·다면체 메시·보호 구·재생성 도구와 실행 경로가 들어 있습니다. 장착 형상과 검증 범위는 SENSOR_MOUNT.md를 참고하세요.
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
            start += '\n시리얼 붓기 시작·정지 및 물체 설정은 CEREAL_POUR.md를 참고하세요.\n'
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
    parser.add_argument('--cereal', action='store_true', help='Build only the cereal pouring deployment ZIP')
    args = parser.parse_args()
    package(args.output, args.cereal)
