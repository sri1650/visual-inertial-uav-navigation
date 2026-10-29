"""Fail-closed EuRoC sensor acquisition audit and replay.

Ground truth is deliberately absent from this module. It is loaded only by the
separate evaluator after an estimator has produced a trajectory.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import heapq
from pathlib import Path
from typing import Iterator, Literal

from .euroc import EurocCameraFrame, EurocImu, load_camera_csv, load_imu_csv


@dataclass(frozen=True)
class EurocStereoFrame:
    timestamp_ns: int
    cam0_path: Path
    cam1_path: Path
    timestamp_delta_ns: int


@dataclass(frozen=True)
class EurocSensorEvent:
    timestamp_ns: int
    kind: Literal["imu", "stereo"]
    sample: EurocImu | EurocStereoFrame


@dataclass(frozen=True)
class EurocAuditThresholds:
    minimum_stereo_pairs: int = 3000
    minimum_stereo_coverage: float = 0.995
    maximum_stereo_delta_ns: int = 1_000_000
    maximum_imu_gap_ns: int = 10_000_000
    minimum_camera_times_bracketed_by_imu: float = 0.995

    def validate(self) -> None:
        if self.minimum_stereo_pairs <= 0:
            raise ValueError("minimum_stereo_pairs must be positive")
        if not (0.0 <= self.minimum_stereo_coverage <= 1.0):
            raise ValueError("minimum_stereo_coverage must be in [0,1]")
        if self.maximum_stereo_delta_ns < 0 or self.maximum_imu_gap_ns <= 0:
            raise ValueError("timestamp limits must be positive")
        if not (0.0 <= self.minimum_camera_times_bracketed_by_imu <= 1.0):
            raise ValueError("minimum_camera_times_bracketed_by_imu must be in [0,1]")


@dataclass(frozen=True)
class EurocSequenceAudit:
    sequence: str
    official_dataset_doi: str
    archive_checksum_algorithm: str
    archive_checksum: str
    cam0_frames: int
    cam1_frames: int
    stereo_pairs: int
    stereo_coverage: float
    maximum_stereo_delta_ns: int
    imu_samples: int
    maximum_imu_gap_ns: int
    camera_times_bracketed_by_imu: float
    first_timestamp_ns: int
    last_timestamp_ns: int
    sensor_only: bool
    ground_truth_consumed: bool
    status: Literal["PASS", "FAIL"]
    failures: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _safe_frame_path(data_dir: Path, filename: str) -> Path:
    relative = Path(filename)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"unsafe camera filename: {filename!r}")
    candidate = data_dir / relative
    if not candidate.is_file():
        raise FileNotFoundError(candidate)
    return candidate


def _pair_stereo(
    cam0: list[EurocCameraFrame],
    cam1: list[EurocCameraFrame],
    cam0_data: Path,
    cam1_data: Path,
    maximum_delta_ns: int,
) -> list[EurocStereoFrame]:
    # Validate the complete acquisition manifests before pairing. Otherwise an
    # unmatched CSV row could silently escape frame-presence/path checks.
    cam0_paths = [_safe_frame_path(cam0_data, frame.filename) for frame in cam0]
    cam1_paths = [_safe_frame_path(cam1_data, frame.filename) for frame in cam1]
    pairs: list[EurocStereoFrame] = []
    left = right = 0
    while left < len(cam0) and right < len(cam1):
        delta = cam1[right].timestamp_ns - cam0[left].timestamp_ns
        if abs(delta) <= maximum_delta_ns:
            pairs.append(EurocStereoFrame(
                timestamp_ns=max(cam0[left].timestamp_ns, cam1[right].timestamp_ns),
                cam0_path=cam0_paths[left],
                cam1_path=cam1_paths[right],
                timestamp_delta_ns=abs(delta),
            ))
            left += 1
            right += 1
        elif delta < 0:
            right += 1
        else:
            left += 1
    return pairs


def _imu_bracketing_fraction(camera_times: list[int], imu_times: list[int]) -> float:
    if not camera_times or len(imu_times) < 2:
        return 0.0
    index = 0
    bracketed = 0
    for timestamp in camera_times:
        while index + 1 < len(imu_times) and imu_times[index + 1] < timestamp:
            index += 1
        if index + 1 < len(imu_times) and imu_times[index] <= timestamp <= imu_times[index + 1]:
            bracketed += 1
    return bracketed / len(camera_times)


def audit_euroc_sequence(
    root: str | Path,
    *,
    sequence: str,
    archive_checksum: str,
    archive_checksum_algorithm: str = "MD5",
    official_dataset_doi: str = "10.3929/ethz-b-000690084",
    thresholds: EurocAuditThresholds = EurocAuditThresholds(),
) -> EurocSequenceAudit:
    thresholds.validate()
    if not sequence.strip() or not archive_checksum.strip():
        raise ValueError("sequence and archive checksum are required provenance")
    root = Path(root)
    cam0 = load_camera_csv(root / "mav0" / "cam0" / "data.csv")
    cam1 = load_camera_csv(root / "mav0" / "cam1" / "data.csv")
    imu = load_imu_csv(root / "mav0" / "imu0" / "data.csv")
    if not cam0 or not cam1 or not imu:
        raise ValueError("camera and IMU streams must be non-empty")
    pairs = _pair_stereo(cam0, cam1, root / "mav0" / "cam0" / "data",
                         root / "mav0" / "cam1" / "data",
                         thresholds.maximum_stereo_delta_ns)
    coverage = len(pairs) / max(len(cam0), len(cam1))
    max_stereo_delta = max((pair.timestamp_delta_ns for pair in pairs), default=0)
    imu_times = [sample.timestamp_ns for sample in imu]
    max_imu_gap = max((b - a for a, b in zip(imu_times, imu_times[1:])), default=0)
    bracketed = _imu_bracketing_fraction([pair.timestamp_ns for pair in pairs], imu_times)
    failures: list[str] = []
    if len(pairs) < thresholds.minimum_stereo_pairs:
        failures.append(f"stereo_pairs {len(pairs)} < {thresholds.minimum_stereo_pairs}")
    if coverage < thresholds.minimum_stereo_coverage:
        failures.append(f"stereo_coverage {coverage:.6f} < {thresholds.minimum_stereo_coverage:.6f}")
    if max_stereo_delta > thresholds.maximum_stereo_delta_ns:
        failures.append("stereo timestamp delta exceeds threshold")
    if max_imu_gap > thresholds.maximum_imu_gap_ns:
        failures.append(f"maximum_imu_gap_ns {max_imu_gap} > {thresholds.maximum_imu_gap_ns}")
    if bracketed < thresholds.minimum_camera_times_bracketed_by_imu:
        failures.append(
            f"camera_times_bracketed_by_imu {bracketed:.6f} < "
            f"{thresholds.minimum_camera_times_bracketed_by_imu:.6f}"
        )
    timestamps = [imu[0].timestamp_ns, imu[-1].timestamp_ns,
                  pairs[0].timestamp_ns if pairs else cam0[0].timestamp_ns,
                  pairs[-1].timestamp_ns if pairs else cam0[-1].timestamp_ns]
    return EurocSequenceAudit(
        sequence=sequence,
        official_dataset_doi=official_dataset_doi,
        archive_checksum_algorithm=archive_checksum_algorithm,
        archive_checksum=archive_checksum,
        cam0_frames=len(cam0),
        cam1_frames=len(cam1),
        stereo_pairs=len(pairs),
        stereo_coverage=coverage,
        maximum_stereo_delta_ns=max_stereo_delta,
        imu_samples=len(imu),
        maximum_imu_gap_ns=max_imu_gap,
        camera_times_bracketed_by_imu=bracketed,
        first_timestamp_ns=min(timestamps),
        last_timestamp_ns=max(timestamps),
        sensor_only=True,
        ground_truth_consumed=False,
        status="FAIL" if failures else "PASS",
        failures=tuple(failures),
    )


def iter_sensor_events(
    root: str | Path,
    *,
    maximum_stereo_delta_ns: int = 1_000_000,
) -> Iterator[EurocSensorEvent]:
    """Yield timestamp-ordered sensor input; ground truth is never opened."""
    root = Path(root)
    imu = load_imu_csv(root / "mav0" / "imu0" / "data.csv")
    cam0 = load_camera_csv(root / "mav0" / "cam0" / "data.csv")
    cam1 = load_camera_csv(root / "mav0" / "cam1" / "data.csv")
    stereo = _pair_stereo(cam0, cam1, root / "mav0" / "cam0" / "data",
                          root / "mav0" / "cam1" / "data", maximum_stereo_delta_ns)
    imu_events = (EurocSensorEvent(x.timestamp_ns, "imu", x) for x in imu)
    stereo_events = (EurocSensorEvent(x.timestamp_ns, "stereo", x) for x in stereo)
    # IMU precedes a camera event at an equal timestamp, ensuring propagation
    # reaches the measurement time before the front end consumes the frame.
    yield from heapq.merge(imu_events, stereo_events,
                           key=lambda event: (event.timestamp_ns, 0 if event.kind == "imu" else 1))

