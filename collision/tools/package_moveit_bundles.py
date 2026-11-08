#!/usr/bin/env python3
"""Package committed MoveIt sources, meshes and launchers, without build files."""
import argparse
from pathlib import Path
import subprocess
import zipfile

ROOT=Path(__file__).resolve().parents[1]
COMMON='''# 시작 안내

ROS2 Humble / Ubuntu 22.04용 소스 ZIP입니다. 설치된 ROS 의존성으로 빌드한 뒤 실행합니다.

## 최초 설치

```bash
sudo apt install ros-humble-moveit ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-tf2-ros python3-venv python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install pin==2.7.0 scipy==1.10.1
```

압축을 푼 뒤 이 파일이 있는 폴더에서 실행합니다. 스크립트가 소스를 빌드하므로 최초 실행은 시간이 걸립니다. 종료는 Ctrl+C입니다.

## 기본 충돌 형상: 전체 다면체

양팔의 베이스·팔·손목·그리퍼·양쪽 집게에 구간별 볼록 다면체를 적용했습니다.
한 팔 31개, 양팔 62개이며 시뮬과 실물용은 동일한 메시를 사용합니다.
원본 STL의 큰 사선과 곡면을 따라 맞췄고, 지지 평면을 바깥으로 5mm 이동한 여유가 화면의 메시 자체에 포함됩니다.
모서리 방향의 여유는 5mm보다 클 수 있으며 작은 오목한 홈은 여전히 메워질 수 있습니다.
실물 외함/정지 여유 검증은 별도입니다. 기본 실행은 생성된 STL을 그대로 읽으므로 재생성할 필요가 없습니다.

기본 시뮬은 관절 조절기와 RViz가 열리고 박스 편집 창은 열리지 않습니다.
기존 collision_tuning.json은 기본 다면체 모드에 자동 적용되지 않습니다.
박스 수동 편집이 필요하면 geometry:=boxes editor:=true를 시뮬 명령 뒤에 추가하세요.
실물용에서 이전 박스 설정을 쓰려면 geometry:=boxes collision_config:=/절대/경로/설정.json을 추가하세요.
실물용은 시작할 때만 설정을 읽습니다. 1차 Pinocchio 거리 기준은 유지합니다.

상세 설명: piper_moveit_guard/README.md

'''
DEMO='''## 시뮬/RViz 확인용

```bash
bash run_moveit_visual_demo.sh
```

좌우 병렬 관절 조절기, 다면체 형상, 접촉점과 겹침 깊이를 볼 수 있습니다.
실제 로봇 명령은 없으며 물리/제동 시뮬레이션은 아닙니다. 침범 자세를 볼 수 있도록 관절 슬라이더는 계속 움직입니다.
기본 데모 ROS 도메인은 179입니다. 편집 창만 숨기려면 editor:=false를 추가합니다.
'''
REAL='''## 실물 로봇 연결용 프로토타입

```bash
# 기본: 실제 피드백과 목표를 읽는 감시 모드
bash run_moveit_real_guard.sh
```

Pinocchio 검사 후 MoveIt이 현재 자세와 목표까지 이동 구간의 양팔 다면체 겹침을 표본 검사합니다.
충돌 위험이면 명령 전달을 차단합니다. 연속 비충돌 보장이나 자동 우회 경로 생성은 포함하지 않습니다.
실물 형상/정지 동작을 검증한 뒤 전달을 켜는 설정:

```bash
bash run_moveit_real_guard.sh enforce:=true geometry_confirmed:=true
```

Piper 드라이버/텔레옵은 별도 실행입니다. 목표는 /teleop/left_target, /teleop/right_target,
피드백은 /joint_states_left, /joint_states_right입니다. 최종 /joint_ctrl_cmd_*로 직접 보내는 우회 경로는 없어야 합니다.
기존 guard.launch.py는 중복 실행하지 마세요. 상세 단위/수동 reset은 README를 참고하세요.
실물 제동 검증 전 프로토타입이며 위치 유지 요청은 비상정지가 아닙니다. 독립 watchdog과 검증된 정지 동작이 필요합니다.
'''


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/tmp/piper_moveit_bundles')
    out=Path(parser.parse_args().output);out.mkdir(parents=True,exist_ok=True)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    dirty=subprocess.check_output(['git','status','--porcelain','--untracked-files=no','.'],cwd=ROOT,text=True).strip()
    # Include generated meshes and new loader/tests before they are committed.
    # Restrict collection to the three source packages and explicit tools.
    paths=[]
    for package in ('piper_dual_description','piper_collision_guard','piper_moveit_guard'):
        paths.extend(str(p.relative_to(ROOT)) for p in sorted((ROOT/package).rglob('*'))
                     if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache'))
                     and p.suffix not in ('.pyc','.pyo'))
    paths += ['tools/generate_collision_boxes.py','tools/generate_collision_meshes.py',
              'tools/generate_protection_spheres.py','tools/moveit_runtime.sh']
    for filename,script,readme in [
        ('dual_piper_moveit_visual_demo.zip','run_moveit_visual_demo.sh',DEMO),
        ('dual_piper_moveit_real_robot_guard_prototype.zip','run_moveit_real_guard.sh',REAL)]:
        prefix=filename[:-4]+'/'
        with zipfile.ZipFile(out/filename,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
            for name in paths+[script]:archive.write(ROOT/name,prefix+name)
            archive.writestr(prefix+'README_START.md',COMMON+readme)
            archive.writestr(prefix+'SOURCE_VERSION.txt',f'Base Git commit: {commit}\nLocal tracked changes included: {bool(dirty)}\nhttps://github.com/idoyeon156-cloud/Robocup-Home-2027_VLA_Tutorial\n')
        with zipfile.ZipFile(out/filename) as archive:
            assert archive.testzip() is None
            assert prefix+'piper_collision_guard/piper_collision_guard/box_editor.py' in archive.namelist()
            assert prefix+'piper_collision_guard/piper_collision_guard/box_tuning.py' in archive.namelist()
        print(filename,(out/filename).stat().st_size,'bytes')


if __name__=='__main__':main()
