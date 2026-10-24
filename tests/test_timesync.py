from vionav.timesync import pair_nearest

def test_exact_pair():
    assert pair_nearest([10], [10], 0) == [(10, 10, 0)]

def test_nearest_pair():
    assert pair_nearest([100], [90, 104, 130], 10) == [(100, 104, 4)]

def test_outside_window():
    assert pair_nearest([100], [130], 20) == [(100, None, None)]

def test_tie_is_deterministic():
    assert pair_nearest([100], [95, 105], 10) == [(100, 95, -5)]
