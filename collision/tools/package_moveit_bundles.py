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

## 박스 수동 편집

시뮬용 실행 시 관절 조절기·RViz·박스 편집 창이 열립니다.
링크와 박스 번호를 선택하고 크기/중심 위치(mm), 회전(도), 면당 여유(mm)를 바꾸세요.
선택 링크는 노란색, 충돌한 링크는 빨간색으로 표시됩니다.
박스 본체가 폭 80mm이고 면당 여유가 5mm라면 충돌 검사 폭은 90mm입니다.
실제 MoveIt 반영 완료 표시를 확인하세요. 박스 추가/삭제(최소 1개), 반대 팔 복사, 기본값 복원도 가능합니다.

저장하면 실행 폴더에 collision_tuning.json이 생깁니다. 같은 폴더에서 재실행하면 자동으로 읽습니다.
시뮬 ZIP에서 맞춘 설정을 실물 ZIP 폴더에서 쓰려면 collision_tuning.json도 복사하세요.
다른 경로는 실행 명령 뒤에 collision_config:=/절대/경로/설정.json을 추가합니다.
실물용은 시작할 때만 설정을 읽으며 실시간 편집은 데모에만 적용됩니다.
이 편집은 MoveIt 박스만 바꾸며 1차 Pinocchio 보호 구/임계값은 바꾸지 않습니다.

상세 설명: piper_moveit_guard/README.md

'''
DEMO='''## 시뮬/RViz 확인용

```bash
bash run_moveit_visual_demo.sh
```

좌우 병렬 관절 조절기, 실시간 박스 편집, 접촉점과 겹침 깊이를 볼 수 있습니다.
실제 로봇 명령은 없으며 물리/제동 시뮬레이션은 아닙니다. 침범 자세를 볼 수 있도록 관절 슬라이더는 계속 움직입니다.
기본 데모 ROS 도메인은 179입니다. 편집 창만 숨기려면 editor:=false를 추가합니다.
'''
REAL='''## 실물 로봇 연결용 프로토타입

```bash
# 기본: 실제 피드백과 목표를 읽는 감시 모드
bash run_moveit_real_guard.sh
```

Pinocchio 검사 후 MoveIt이 현재 자세와 목표까지 이동 구간의 양팔 박스 겹침을 표본 검사합니다.
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
    paths=subprocess.check_output(['git','ls-files','piper_dual_description','piper_collision_guard','piper_moveit_guard',
        'tools/generate_collision_boxes.py','tools/generate_protection_spheres.py','tools/moveit_runtime.sh'],cwd=ROOT,text=True).splitlines()
    for filename,script,readme in [
        ('dual_piper_moveit_visual_demo.zip','run_moveit_visual_demo.sh',DEMO),
        ('dual_piper_moveit_real_robot_guard_prototype.zip','run_moveit_real_guard.sh',REAL)]:
        prefix=filename[:-4]+'/'
        with zipfile.ZipFile(out/filename,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
            for name in paths+[script]:archive.write(ROOT/name,prefix+name)
            archive.writestr(prefix+'README_START.md',COMMON+readme)
            archive.writestr(prefix+'SOURCE_VERSION.txt',f'Git commit: {commit}\nhttps://github.com/idoyeon156-cloud/Robocup-Home-2027_VLA_Tutorial\n')
        with zipfile.ZipFile(out/filename) as archive:
            assert archive.testzip() is None
            assert prefix+'piper_collision_guard/piper_collision_guard/box_editor.py' in archive.namelist()
            assert prefix+'piper_collision_guard/piper_collision_guard/box_tuning.py' in archive.namelist()
        print(filename,(out/filename).stat().st_size,'bytes')


if __name__=='__main__':main()
