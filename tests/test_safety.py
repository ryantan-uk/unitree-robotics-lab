from common.safety import CONFIRM_PHRASE, MotionGate


def test_live_gate_blocks_until_confirmed(capsys):
    sent = []
    gate = MotionGate(live=True)
    assert gate.call(sent.append, "StandUp") is None and sent == []
    assert not gate.allow_write()
    assert gate.confirm("yes") is False and sent == []
    assert gate.confirm(CONFIRM_PHRASE) is True
    gate.call(sent.append, "StandUp")
    assert sent == ["StandUp"] and gate.allow_write()
    gate.disarm()
    gate.call(sent.append, "Move")
    assert sent == ["StandUp"]
    assert [s for _, s in gate.log] == [False, True, False]
    assert "未下发" in capsys.readouterr().out


def test_offline_gate_never_sends_even_if_confirmed():
    sent = []
    gate = MotionGate(live=False, verbose=False)
    assert gate.confirm(CONFIRM_PHRASE) is False
    assert gate.call(sent.append, "StandUp") is None
    assert sent == [] and not gate.will_send and not gate.allow_write()
