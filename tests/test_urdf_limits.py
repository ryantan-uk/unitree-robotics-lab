"""对照官方 URDF 核对 common/robots.py 里的关节名和限位 (没装 URDF 时跳过)。"""
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from common import robots as R

DEPS = Path(os.environ.get("UNITREE_LAB_DEPS", Path.home() / "unitree_lab_deps")) / "unitree_ros" / "robots"
CASES = [("go2", DEPS / "go2_description/urdf/go2_description.urdf", R.GO2_URDF_JOINTS),
         ("g1", DEPS / "g1_description/g1_29dof_rev_1_0.urdf", R.G1_URDF_JOINTS)]


@pytest.mark.parametrize("robot,path,names", CASES)
def test_limits_match_urdf(robot, path, names):
    if not path.exists():
        pytest.skip(f"URDF 未安装: {path}")
    urdf = {j.get("name"): j.find("limit") for j in ET.parse(path).getroot().findall("joint")
            if j.get("type") == "revolute"}
    lo, hi = R.limits_of(robot)
    for i, name in enumerate(names):
        assert name in urdf, f"URDF 里没有关节 {name}"
        assert float(urdf[name].get("lower")) == pytest.approx(lo[i], abs=1e-3)
        assert float(urdf[name].get("upper")) == pytest.approx(hi[i], abs=1e-3)
