#!/usr/bin/env python3
"""Read local JSONL diagnostics without ROS, CAN, or any robot command."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', nargs='?', type=Path,
                        help='diagnostics/<run-id> directory; default: newest recorded run')
    parser.add_argument('--last', type=int, default=40)
    parser.add_argument('--samples', action='store_true', help='also show 20 Hz samples')
    parser.add_argument('--json', action='store_true', help='print complete JSON records')
    args = parser.parse_args()
    directory = args.directory
    if directory is None:
        runs = [p for p in (ROOT/'log').glob('*/diagnostics/*') if p.is_dir() and any(p.glob('*.jsonl'))]
        if not runs:
            parser.exit(1, '새 형식 진단 로그가 없습니다. 다음 시뮬 실행부터 기록됩니다.\n')
        directory = max(runs, key=lambda p: max(f.stat().st_mtime_ns for f in p.glob('*.jsonl')))
    rows = []
    for path in directory.glob('*.jsonl'):
        for number, line in enumerate(path.open(), 1):
            try:
                row = json.loads(line)
                if args.samples or row.get('event') != 'sample':
                    rows.append(row)
            except (ValueError, TypeError):
                print(f'불완전한 기록 건너뜀: {path.name}:{number}')
    rows.sort(key=lambda r: r.get('monotonic_ns', 0))
    print('실행 로그:', directory)
    for row in rows[-max(1, args.last):]:
        if args.json:
            print(json.dumps(row, ensure_ascii=False))
            continue
        data = row.get('data', {})
        print(f"{row.get('ros_time_ns', 0)/1e9:.3f} {row.get('node')} {row.get('event')}: "
              +json.dumps({k: data[k] for k in ('state', 'detail', 'step', 'blocked', 'latched',
                  'first_stop_reason', 'reason', 'pair', 'nearest_pair', 'minimum_clearance_m',
                  'checked_pose_role', 'success') if k in data}, ensure_ascii=False))
        for side in ('left', 'right'):
            if side in data:
                print('  '+side+': '+json.dumps(data[side], ensure_ascii=False))
        if 'checked_pose_role' in row:
            print('  검사 자세:', row['checked_pose_role'], '관절(rad/m):', row.get('checked_joint_rad_m'))
    if not rows:
        print('표시할 기록이 없습니다. --samples 또는 --json 옵션을 사용할 수 있습니다.')


if __name__ == '__main__':
    main()
