from __future__ import annotations

import json
from pathlib import Path

import pytest

from vionav.euroc_acceptance import main
from vionav.euroc_replay import EurocAuditThresholds, audit_euroc_sequence, iter_sensor_events


def fixture(root: Path, *, missing_right: bool = False, imu_gap_ns: int = 5_000_000) -> Path:
    for sensor in ("cam0", "cam1"):
        (root / "mav0" / sensor / "data").mkdir(parents=True)
    (root / "mav0" / "imu0").mkdir(parents=True)
    times = [10_000_000, 20_000_000, 30_000_000]
    for sensor in ("cam0", "cam1"):
        lines = ["#timestamp,filename"]
        for index, timestamp in enumerate(times):
            if sensor == "cam1" and missing_right and index == 1:
                continue
            name = f"{timestamp}.png"
            (root / "mav0" / sensor / "data" / name).write_bytes(b"fixture-image")
            lines.append(f"{timestamp},{name}")
        (root / "mav0" / sensor / "data.csv").write_text("\n".join(lines) + "\n")
    imu_times = list(range(5_000_000, 35_000_001, imu_gap_ns))
    imu_lines = ["#timestamp,wx,wy,wz,ax,ay,az"]
    imu_lines.extend(f"{t},0,0,0,0,0,9.80665" for t in imu_times)
    (root / "mav0" / "imu0" / "data.csv").write_text("\n".join(imu_lines) + "\n")
    # Deliberately invalid: the sensor audit/replay must never read evaluator truth.
    gt = root / "mav0" / "state_groundtruth_estimate0"
    gt.mkdir(); (gt / "data.csv").write_text("THIS MUST NOT BE READ\n")
    return root


def thresholds(**overrides) -> EurocAuditThresholds:
    values = dict(minimum_stereo_pairs=3, minimum_stereo_coverage=1.0,
                  maximum_stereo_delta_ns=0, maximum_imu_gap_ns=5_000_000,
                  minimum_camera_times_bracketed_by_imu=1.0)
    values.update(overrides)
    return EurocAuditThresholds(**values)


def test_sensor_only_audit_and_timestamp_ordered_replay(tmp_path: Path):
    root = fixture(tmp_path / "MH_01_easy")
    report = audit_euroc_sequence(root, sequence="MH_01_easy",
                                  archive_checksum="known-md5", thresholds=thresholds())
    assert report.status == "PASS"
    assert report.sensor_only and not report.ground_truth_consumed
    events = list(iter_sensor_events(root, maximum_stereo_delta_ns=0))
    assert [event.timestamp_ns for event in events] == sorted(event.timestamp_ns for event in events)
    equal_timestamp = [event.kind for event in events if event.timestamp_ns == 10_000_000]
    assert equal_timestamp == ["imu", "stereo"]


def test_audit_fails_closed_on_incomplete_stereo_and_imu_gap(tmp_path: Path):
    root = fixture(tmp_path / "MH_01_easy", missing_right=True, imu_gap_ns=15_000_000)
    report = audit_euroc_sequence(
        root, sequence="MH_01_easy", archive_checksum="known-md5",
        thresholds=thresholds(maximum_imu_gap_ns=10_000_000),
    )
    assert report.status == "FAIL"
    assert any("stereo_pairs" in failure for failure in report.failures)
    assert any("stereo_coverage" in failure for failure in report.failures)
    assert any("maximum_imu_gap_ns" in failure for failure in report.failures)


def test_rejects_camera_path_traversal(tmp_path: Path):
    root = fixture(tmp_path / "MH_01_easy")
    cam0 = root / "mav0" / "cam0" / "data.csv"
    cam0.write_text(cam0.read_text().replace("10000000.png", "../10000000.png"))
    with pytest.raises(ValueError, match="unsafe camera filename"):
        audit_euroc_sequence(root, sequence="MH_01_easy",
                             archive_checksum="known-md5", thresholds=thresholds())


def test_cli_retains_provenance_and_failure_exit(tmp_path: Path):
    root = fixture(tmp_path / "MH_01_easy", missing_right=True)
    output = tmp_path / "audit.json"
    result = main([str(root), "--sequence", "MH_01_easy", "--archive-checksum", "363f",
                   "--minimum-stereo-pairs", "3", "--output", str(output)])
    assert result == 1
    report = json.loads(output.read_text())
    assert report["archive_checksum"] == "363f"
    assert report["official_dataset_doi"] == "10.3929/ethz-b-000690084"
    assert report["ground_truth_consumed"] is False

