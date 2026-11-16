from gnss_hmi.parsers.rtkstat import (
    SatStat, SlipWindow, StatEpochAccumulator, parse_sat_line)

LINE = "$SAT,2372,113755.4,G12,1,231.5,18.2,2.31,0.012,1,34.5,1,0,120,0,3,1"


def _sat(tow, sat, freq=1, az=100.0, el=20.0, snr=45.0, vsat=1, slipc=0):
    # $SAT,week,tow,sat,freq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
    return (f"$SAT,2372,{tow},{sat},{freq},{az},{el},0.5,0.0,{vsat},{snr},"
            f"1,0,1,0,{slipc},0\n")


def test_parse_sat_line():
    s = parse_sat_line(LINE)
    assert isinstance(s, SatStat)
    assert s.sat == "G12" and abs(s.tow - 113755.4) < 1e-6
    assert abs(s.az - 231.5) < 1e-6 and abs(s.el - 18.2) < 1e-6
    assert abs(s.resp - 2.31) < 1e-6 and abs(s.snr - 34.5) < 1e-6
    assert s.valid is True and s.slipc == 3 and s.rejc == 1


def test_non_sat_lines_return_none():
    assert parse_sat_line("$POS,2372,113755.4,...") is None
    assert parse_sat_line("$SAT,bad") is None


def test_slip_window_counts_increments_only():
    w = SlipWindow(window_s=30.0)
    w.feed(100.0, "G12", 3)      # first sighting: baseline, no increment
    w.feed(101.0, "G12", 5)      # +2
    w.feed(102.0, "C08", 1)      # baseline
    w.feed(103.0, "C08", 2)      # +1
    assert w.count(now=110.0) == 3
    assert w.count(now=140.0) == 0     # both increments aged out (>30 s)


def test_accumulator_dedupes_second_frequency():
    acc = StatEpochAccumulator()
    for line in (_sat(100.0, "G05", freq=1, snr=45),
                 _sat(100.0, "G05", freq=2, snr=38),   # 2nd freq: dropped
                 _sat(100.0, "G07", freq=1, snr=47)):
        acc.feed(line)
    assert [s["sat"] for s in acc.sats] == ["G05", "G07"]
    assert acc.sats[0]["snr"] == 45.0                  # first frequency kept
    assert [s.sat for s in acc.satstats] == ["G05", "G07"]


def test_accumulator_promotes_only_completed_epoch():
    acc = StatEpochAccumulator()
    acc.feed(_sat(100.0, "G05"))
    acc.feed(_sat(100.0, "G07"))
    assert {s["sat"] for s in acc.sats} == {"G05", "G07"}
    # a new epoch (tow advances) starts fresh; public list flips to the new
    # epoch once its first sat lands, never showing an empty set in between
    acc.feed(_sat(101.0, "G11"))
    assert [s["sat"] for s in acc.sats] == ["G11"]
    acc.feed(_sat(101.0, "G05"))
    assert [s["sat"] for s in acc.sats] == ["G11", "G05"]


def test_accumulator_used_flag_from_vsat():
    acc = StatEpochAccumulator()
    acc.feed(_sat(100.0, "G05", vsat=1))
    acc.feed(_sat(100.0, "G07", vsat=0))
    used = {s["sat"]: s["used"] for s in acc.sats}
    assert used == {"G05": True, "G07": False}


def test_accumulator_feeds_slip_counts():
    acc = StatEpochAccumulator()
    slips = []
    acc.feed(_sat(100.0, "G05", slipc=2), on_slip=lambda sat, c: slips.append((sat, c)))
    acc.feed(_sat(100.0, "G05", freq=2, slipc=2),                 # 2nd freq: no slip cb
             on_slip=lambda sat, c: slips.append((sat, c)))
    assert slips == [("G05", 2)]                                  # once per sat


def test_accumulator_reset_clears_state():
    acc = StatEpochAccumulator()
    acc.feed(_sat(100.0, "G05"))
    acc.reset()
    assert acc.sats == [] and acc.satstats == []
    acc.feed(_sat(200.0, "G09"))
    assert [s["sat"] for s in acc.sats] == ["G09"]
