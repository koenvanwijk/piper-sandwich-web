from lerobot_robot_piper_sim.assembler import TickAssembler


def st(seq):
    return {"seq": seq, "t_sim": seq / 30, "state": [0.0] * 14, "action": [0.0] * 14}


def test_latest_frame_le_seq_and_delay(jpg):
    a = TickAssembler(n_cams=2, delay=3)
    out = []
    for s in range(10):
        if s % 2 == 0:
            a.add_frame(s, 0, jpg())
        if s == 4:
            a.add_frame(4, 1, jpg())
        out += a.add_state(st(s))
    out += a.flush()
    assert [t.seq for t in out] == list(range(10))                       # volgorde, geen dubbelen
    assert 1 not in out[0].frames and 0 in out[0].frames                 # cam 1 nog niet beschikbaar
    assert out[5].frames[0][0] == 4 and out[5].frames[1][0] == 4         # nieuwste <= seq
    assert all(fs <= t.seq for t in out for fs, _ in t.frames.values())  # nooit een beeld uit de toekomst
