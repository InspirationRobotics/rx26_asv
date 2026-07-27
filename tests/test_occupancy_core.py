from rx26_asv.api.navigation.occupancy_core import (
    OccupancyCore, SOURCE_COMMS, SOURCE_PERCEPTION)


def grid(**kw):
    base = dict(cell_size=0.5, decay_tau=10.0, occupied_threshold=50.0)
    base.update(kw)
    return OccupancyCore(**base)


def test_ingest_and_query():
    g = grid()
    g.ingest_detection(5.0, 5.0, radius=0.5, t=0.0)
    pts = g.occupied_points(t=0.0)
    assert pts
    assert all(abs(x - 5.0) < 1.5 and abs(y - 5.0) < 1.5 for x, y, _, _ in pts)
    assert all(src == SOURCE_PERCEPTION for _, _, _, src in pts)


def test_perception_decay_and_prune():
    g = grid(decay_tau=10.0)
    g.ingest_detection(5.0, 5.0, radius=0.5, t=0.0)
    assert g.occupied_points(t=0.0)
    # after one tau, value = 100*e^-1 ~ 36.8 < threshold 50 -> not occupied
    assert not g.occupied_points(t=10.0)
    n = len(g.cells)
    removed = g.prune(t=60.0)          # far past tau -> below prune floor
    assert removed == n and not g.cells


def test_comms_cells_never_decay():
    g = grid(decay_tau=10.0)
    g.ingest_detection(5.0, 5.0, radius=1.0, t=0.0,
                       source=SOURCE_COMMS, zone_id="K1")
    pts = g.occupied_points(t=1000.0)  # 100 tau later, still there
    assert pts and all(src == SOURCE_COMMS for _, _, _, src in pts)
    assert g.prune(t=1000.0) == 0


def test_clear_zone_removes_only_that_zone():
    g = grid()
    g.ingest_detection(5.0, 5.0, radius=1.0, t=0.0, source=SOURCE_COMMS, zone_id="K1")
    g.ingest_detection(20.0, 20.0, radius=1.0, t=0.0, source=SOURCE_COMMS, zone_id="K2")
    g.ingest_detection(40.0, 40.0, radius=0.5, t=0.0)      # perception
    removed = g.clear_zone("K1")
    assert removed > 0
    pts = g.occupied_points(t=0.0)
    assert not any(abs(x - 5.0) < 2 for x, _, _, _ in pts)     # K1 gone
    assert any(abs(x - 20.0) < 2 for x, _, _, _ in pts)        # K2 stays
    assert any(abs(x - 40.0) < 2 for x, _, _, _ in pts)        # perception stays


def test_perception_cannot_overwrite_keepout_cell():
    g = grid()
    g.ingest_detection(5.0, 5.0, radius=1.0, t=0.0, source=SOURCE_COMMS, zone_id="K1")
    g.ingest_detection(5.0, 5.0, radius=1.0, t=1.0, confidence=0.1)  # weak perception
    pts = g.occupied_points(t=1000.0)   # would have decayed if overwritten
    assert pts and all(src == SOURCE_COMMS for _, _, _, src in pts)


def test_refresh_uses_max_not_overwrite():
    g = grid(decay_tau=10.0)
    g.ingest_detection(5.0, 5.0, radius=0.5, t=0.0, confidence=1.0)
    # weak re-hit at t=1 must not clobber the strong (barely decayed) value
    g.ingest_detection(5.0, 5.0, radius=0.5, t=1.0, confidence=0.2)
    assert g.occupied_points(t=1.0)


def test_msg_dict_shape():
    g = grid()
    g.ingest_detection(5.0, 5.0, radius=0.5, t=0.0)
    d = g.to_msg_dict(0.0, origin=(32.7, -117.25), position_xyh=(1.0, 2.0, 0.5))
    assert d["cell_size"] == 0.5 and d["decay_tau"] == 10.0
    assert d["cells"] and {"x_coord", "y_coord", "value", "source"} <= set(d["cells"][0])
