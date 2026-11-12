#!/usr/bin/env python3
"""Export the committed distance demo and robot guard with matching defaults."""
import argparse
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/piper_nine_cm_bundles')
    out = Path(parser.parse_args().output)
    out.mkdir(parents=True, exist_ok=True)
    version = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    repository = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], cwd=ROOT, text=True).strip())
    source_prefix = ROOT.relative_to(repository).as_posix() + '/'
    paths = subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', version, '--',
        *[source_prefix + path for path in (
            'piper_dual_description', 'piper_collision_guard', 'run_collision_visual_demo.sh',
            'tools/generate_collision_boxes.py', 'tools/generate_protection_spheres.py')]],
        cwd=repository, text=True).splitlines()
    paths = [path.removeprefix(source_prefix) for path in paths]
    for filename, demo in [('dual_piper_collision_visual_demo.zip', True),
                           ('dual_piper_real_robot_guard_prototype.zip', False)]:
        prefix = 'Robocup-Home-2027_VLA_Tutorial/'
        with zipfile.ZipFile(out/filename, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for path in paths:
                if not demo and (path == 'run_collision_visual_demo.sh' or path.endswith(('degree_gui.py', 'box_editor.py', 'box_tuning.py', 'VISUAL_DEMO.md', 'collision_limit_visual_demo.launch.py', 'collision_limit_visual_demo.rviz'))):
                    continue
                content = subprocess.check_output(['git', 'show', f'{version}:{source_prefix}{path}'], cwd=ROOT)
                if not demo and path.endswith('/setup.py'):
                    text = content.decode().replace(', "VISUAL_DEMO.md"', '')
                    content = ('\n'.join(line for line in text.splitlines() if not any(entry in line for entry in ['collision_degree_sliders =', 'collision_limit_visual_demo =', 'moveit_box_editor =']))+'\n').encode()
                if path == 'piper_collision_guard/README.md':
                    text = content.decode().replace('../piper_moveit_guard/README.md', 'https://github.com/idoyeon156-cloud/Robocup-Home-2027_VLA_Tutorial/blob/main/collision/piper_moveit_guard/README.md')
                    if not demo:
                        text = '\n'.join(line for line in text.splitlines() if 'VISUAL_DEMO.md' not in line)
                    content = text.encode()
                archive.writestr(prefix+path, content)
            readme = '# Dual Piper — '+('거리 시각화 데모' if demo else '실기 연결용 1차 거리 guard 프로토타입')+'\n\n'
            readme += '기본 제한: 관절 중심 거리 9cm. 반지름 4cm 구 표면 간격 1cm 이하에서 제한합니다.\n'
            readme += '설치·빌드: piper_collision_guard/'+('VISUAL_DEMO.md' if demo else 'README.md')+'\n'
            readme += ('실행: bash run_collision_visual_demo.sh\n' if demo else '실행: ros2 launch piper_collision_guard guard.launch.py\n기본은 감시 모드이며 실제 제동은 미검증입니다. MoveIt 2차 검사는 별도 패키지입니다.\n')
            readme += '중심 거리 검사는 실제 외형의 부피 충돌을 모두 검출하지 않습니다.\n'
            archive.writestr(prefix+'README_START.md', readme)
            archive.writestr(prefix+'SOURCE_VERSION.txt', version+'\n')
        with zipfile.ZipFile(out/filename) as archive:
            assert archive.testzip() is None
            assert b'stop_clearance_m: 0.01' in archive.read(prefix+'piper_collision_guard/config/guard.yaml')
            for path in archive.namelist():
                if path.endswith('.py'):
                    compile(archive.read(path), path, 'exec')
        print(filename, (out/filename).stat().st_size, 'bytes')


if __name__ == '__main__':
    main()
