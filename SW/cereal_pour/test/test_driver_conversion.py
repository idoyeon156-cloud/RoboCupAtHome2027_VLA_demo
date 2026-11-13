import ast
import importlib.util
import math
from pathlib import Path

import pytest


def test_overlay_fixes_both_directions_without_changing_other_driver_code():
    path=Path(__file__).resolve().parents[1]/'tools/prepare_driver_overlay.py'
    spec=importlib.util.spec_from_file_location('driver_overlay',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source='import math\n'+''.join(f'f{i} = 0.017444\n' for i in range(12))+'cmd = 57324.840764\n'
    result=module.corrected_source(source)
    values={};exec(compile(ast.parse(result),'<test>','exec'),values)
    for deg in (-70.,0.,90.,150.):
        raw=round(math.radians(deg)*values['cmd'])
        assert raw==round(deg*1000)
        assert math.degrees(raw/1000*values['f0'])==pytest.approx(deg,abs=1e-10)
    with pytest.raises(ValueError,match='manual review'):
        module.corrected_source('unknown driver')
