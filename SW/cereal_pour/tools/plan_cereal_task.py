#!/usr/bin/env python3
"""Generate a verified, atomic v2 cereal sequence from config/cereal_task.yaml."""
import argparse
import hashlib
from pathlib import Path
import sys
import tempfile
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'piper_collision_guard'))
from piper_collision_guard.cereal_planning import generate

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/cereal_task.yaml')
    p.add_argument('--output',type=Path,default=ROOT/'model/cereal_task_sequence.yaml')
    p.add_argument('--check',action='store_true')
    args=p.parse_args()
    try:
        if args.check:
            cached=yaml.safe_load(args.output.read_text())
            for path,digest in cached['input_sha256'].items():
                if hashlib.sha256((ROOT/Path(path)).read_bytes()).hexdigest()!=digest:raise ValueError('Sequence is stale')
            config_paths=[ROOT/Path(path) for path in cached['input_sha256']]
            if args.config.resolve() not in [p.resolve() for p in config_paths]:raise ValueError('Sequence uses a different config')
            print('Planned sequence inputs verified');return
        result=generate(ROOT,args.config)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with tempfile.NamedTemporaryFile('w',dir=args.output.parent,delete=False) as f:
            tmp=Path(f.name);yaml.safe_dump(result,f,sort_keys=False)
        tmp.replace(args.output)
    except (ValueError,KeyError,TypeError,OSError) as e:p.exit(1,f'Planning failed; sequence not replaced: {e}\n')
    print(f'Validated {len(result["waypoints"])} waypoints -> {args.output}')
if __name__=='__main__':main()
