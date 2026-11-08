#!/usr/bin/env python3
"""Append the positioned pillar to an existing binary zero-pose STL; no network.

Run from anywhere in a checkout: python collision/tools/export_with_pillar.py
Or: python export_with_pillar.py --source dual_piper_zero_pose_mm.stl
    --pillar profile_3030_500_placed_mm.stl --output dual_piper_with_pillar_mm.stl
Only the new output is written. Source triangle records are preserved byte-for-byte.
"""
from pathlib import Path
import argparse, struct

def read_binary(path):
    raw=path.read_bytes()
    if len(raw)<84: raise ValueError(f'Not a binary STL: {path}')
    count=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+50*count: raise ValueError(f'Invalid binary STL: {path}')
    return count,raw[84:]

def main():
    here=Path(__file__).resolve()
    in_repo=here.parent.name=='tools' and here.parent.parent.name=='collision'
    root=here.parent.parent if in_repo else here.parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=root/'preview/dual_piper_zero_pose_mm.stl' if in_repo else root/'dual_piper_zero_pose_mm.stl')
    parser.add_argument('--pillar',type=Path,default=root/'pillar_3030/profile_3030_500_placed_mm.stl' if in_repo else root/'profile_3030_500_placed_mm.stl')
    parser.add_argument('--output',type=Path,default=root/'preview/dual_piper_with_3030_pillar_mm.stl' if in_repo else root/'dual_piper_with_3030_pillar_mm.stl')
    args=parser.parse_args()
    if args.output.resolve() in (args.source.resolve(),args.pillar.resolve()):
        parser.error('Output must be a different path from both inputs')
    if args.output.exists():parser.error(f'Refusing to overwrite {args.output}')
    a,ar=read_binary(args.source);b,br=read_binary(args.pillar)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_bytes(b'Dual Piper + generic 3030 500mm pillar; units mm'.ljust(80,b' ')+struct.pack('<I',a+b)+ar+br)
    assert args.output.read_bytes()[84:84+len(ar)]==ar
    print(f'Created {args.output} ({a+b} triangles); originals unchanged')

if __name__=='__main__':main()
