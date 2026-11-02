from gnss_hmi.parsers.rtksol import RtkSolution, parse_llh_solution

LINE = ("2026/08/27 04:15:55.400   44.501234567   90.287654321   617.1234"
        "   1  38   0.0110   0.0123   0.0322  -0.0001   0.0002   0.0003"
        "   0.80    2.5")


def test_parse_solution_line():
    s = parse_llh_solution(LINE)
    assert isinstance(s, RtkSolution)
    assert abs(s.lat - 44.501234567) < 1e-9 and abs(s.lon - 90.287654321) < 1e-9
    assert abs(s.alt - 617.1234) < 1e-6
    assert s.q == 1 and s.ns == 38
    assert abs(s.sdn - 0.0110) < 1e-6 and abs(s.sde - 0.0123) < 1e-6
    assert abs(s.age - 0.80) < 1e-6 and abs(s.ratio - 2.5) < 1e-6
    import datetime
    expect = datetime.datetime(2026, 8, 27, 4, 15, 55, 400000,
                               tzinfo=datetime.timezone.utc).timestamp()
    assert abs(s.t - expect) < 1e-3


def test_comment_and_garbage_return_none():
    assert parse_llh_solution("% GPST latitude ...") is None
    assert parse_llh_solution("") is None
    assert parse_llh_solution("2026/08/27 04:15:55.400 not a number") is None
