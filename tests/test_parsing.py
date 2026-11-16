import struct

import numpy as np
import pytest

from common import parsing as P


def test_remote_layout_is_40_bytes():
    assert P.REMOTE_SIZE == 40


def test_remote_roundtrip():
    raw = P.pack_remote(lx=0.25, ly=-0.5, rx=1.0, ry=-1.0, keys=["A", "L2", "start"])
    d = P.parse_remote(raw)
    assert d["lx"] == pytest.approx(0.25) and d["ly"] == pytest.approx(-0.5)
    assert d["rx"] == pytest.approx(1.0) and d["ry"] == pytest.approx(-1.0)
    assert sorted(d["keys"]) == sorted(["A", "L2", "start"])


def test_remote_bit_positions_match_gamepad_hpp():
    # gamepad.hpp: R1 是最低位, A 是第 8 位, left 是最高位
    for key, bit in (("R1", 0), ("L1", 1), ("A", 8), ("left", 15)):
        raw = struct.pack("<2B H 5f 16x", 0, 0, 1 << bit, 0, 0, 0, 0, 0)
        assert P.parse_remote(raw)["keys"] == [key]


def test_remote_accepts_signed_bytes_and_rejects_wrong_length():
    raw = P.pack_remote(keys=["left"])
    signed = [b - 256 if b > 127 else b for b in raw]      # ROS 2 的 int8[] 会给出负数
    assert P.parse_remote(signed)["keys"] == ["left"]
    with pytest.raises(ValueError):
        P.parse_remote(b"\x00" * 39)


@pytest.mark.parametrize("rpy", [(0, 0, 0), (0.3, -0.2, 1.0), (-1.0, 0.5, -2.5), (0.1, 1.2, 3.0)])
def test_quat_rpy_roundtrip(rpy):
    q = P.rpy_to_quat(rpy)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    assert P.quat_to_rpy(q) == pytest.approx(rpy, abs=1e-9)


def test_quat_batch_shape():
    rpy = np.random.default_rng(0).uniform(-1, 1, (50, 3))
    assert P.quat_to_rpy(P.rpy_to_quat(rpy)) == pytest.approx(rpy, abs=1e-9)


def test_rotation_matrix_is_orthonormal_and_yaw_rotates_x_to_y():
    R = P.rot_from_rpy(0.3, -0.4, 1.1)
    assert R @ R.T == pytest.approx(np.eye(3), abs=1e-12)
    assert P.rot_from_rpy(0, 0, np.pi / 2) @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-12)


def test_transform_2d_composition():
    T = P.transform_2d(1.0, 2.0, np.pi / 2)
    assert T @ [1, 0, 1] == pytest.approx([1.0, 3.0, 1.0])


def test_tilt():
    assert P.tilt_deg(0, 0) == pytest.approx(0)
    assert P.tilt_deg(np.radians(30), 0) == pytest.approx(30)


def test_rate_stats():
    s = P.rate_stats(np.arange(0, 1, 0.002))
    assert s["hz"] == pytest.approx(500, rel=1e-6) and s["dt_std_ms"] == pytest.approx(0, abs=1e-6)
    assert np.isnan(P.rate_stats([0.0])["hz"])
