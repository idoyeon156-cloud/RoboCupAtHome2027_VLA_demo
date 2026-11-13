"""Freeze the path actually displayed by guarded simulation; never unlock hardware."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path

import yaml


def package_root():
    root = Path(os.environ.get('CEREAL_PACKAGE_ROOT', Path.cwd())).resolve()
    if not (root/'config/cereal_task.yaml').is_file():
        raise ValueError('CEREAL_PACKAGE_ROOT must point to the cereal_pour source directory')
    return root


def source_fingerprint(root):
    files = {root/'model/manifest.json', root/'tools/prepare_driver_overlay.py',
             root/'run_piper_driver.sh', root/'run.sh', root/'piper_moveit_guard/CMakeLists.txt'}
    for folder, patterns in (
        ('piper_collision_guard', ('*.py', '*.yaml')),
        ('piper_moveit_guard', ('*.cpp', '*.h', '*.urdf', '*.srdf', '*.stl', '*.STL')),
        ('config', ('*.yaml',)),
        ('piper_dual_description', ('*.urdf', '*.stl', '*.STL')),
    ):
        for pattern in patterns:
            files.update((root/folder).rglob(pattern))
    digest = hashlib.sha256()
    for path in sorted(files):
        digest.update(str(path.relative_to(root)).encode())
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def write_receipt(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def freeze_sequence(sequence_path, model_fingerprint, driver_speed, speed_scale):
    root = package_root()
    sequence = Path(sequence_path).resolve()
    data = sequence.read_bytes()
    config = yaml.safe_load(data)
    if config.get('planner', {}).get('backend') != 'moveit_ompl':
        raise ValueError('Guarded simulation requires a MoveIt-generated sequence')
    directory = root/'model/simulation_sessions'/os.environ['CEREAL_RUN_ID']
    directory.mkdir(parents=True, exist_ok=False)
    frozen = directory/'sequence.yaml'
    frozen.write_bytes(data)
    receipt = directory/'receipt.json'
    write_receipt(receipt, dict(
        state='pending', root=str(root), sequence_path=str(frozen),
        sequence_sha256=hashlib.sha256(data).hexdigest(),
        source_fingerprint=source_fingerprint(root), model_fingerprint=model_fingerprint,
        driver_speed_ratio=int(driver_speed), joint_speed_scale=float(speed_scale)))
    return frozen, receipt


def validate_receipt(path, *, complete=False):
    receipt = Path(path).resolve()
    value = json.loads(receipt.read_text())
    root = package_root()
    if value['root'] != str(root):
        raise ValueError('Simulation receipt belongs to another source directory')
    if hashlib.sha256(Path(value['sequence_path']).read_bytes()).hexdigest() != value['sequence_sha256']:
        raise ValueError('Frozen simulation path was changed')
    if source_fingerprint(root) != value['source_fingerprint']:
        raise ValueError('Source/model/settings changed after simulation; run simulation again')
    if complete and value['state'] != 'complete':
        raise ValueError('The guarded simulation has not completed this path')
    return receipt, value


def complete_snapshot(path):
    receipt, value = validate_receipt(path)
    value['state'] = 'complete'
    value['completed_utc'] = datetime.now(timezone.utc).isoformat()
    write_receipt(receipt, value)
    write_receipt(package_root()/'model/last_completed_simulation.json', value)


def completed_sequence(model_fingerprint=None, driver_speed=None, speed_scale=None):
    _, value = validate_receipt(package_root()/'model/last_completed_simulation.json', complete=True)
    if model_fingerprint is not None and value['model_fingerprint'] != model_fingerprint:
        raise ValueError('MoveIt collision model differs from the completed simulation')
    if driver_speed is not None and value['driver_speed_ratio'] != int(driver_speed):
        raise ValueError('Driver speed differs from the completed simulation')
    if speed_scale is not None and value['joint_speed_scale'] != float(speed_scale):
        raise ValueError('Sequence speed differs from the completed simulation')
    return Path(value['sequence_path'])
