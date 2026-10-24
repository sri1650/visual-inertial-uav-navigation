from __future__ import annotations
import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EurocImu:
    timestamp_ns: int
    gyro_rad_s: tuple[float, float, float]
    accel_m_s2: tuple[float, float, float]


@dataclass(frozen=True)
class EurocCameraFrame:
    timestamp_ns: int
    filename: str


def _data_rows(path: Path):
    with path.open("r", encoding="utf-8", newline="") as f:
        for raw in f:
            if not raw.strip() or raw.lstrip().startswith("#"):
                continue
            yield next(csv.reader([raw]))


def load_imu_csv(path: str | Path):
    path = Path(path)
    out = []
    for row in _data_rows(path):
        if len(row) != 7:
            raise ValueError(f"expected 7 IMU columns, got {len(row)}")
        out.append(EurocImu(
            timestamp_ns=int(row[0]),
            gyro_rad_s=tuple(float(x) for x in row[1:4]),
            accel_m_s2=tuple(float(x) for x in row[4:7]),
        ))
    if any(a.timestamp_ns >= b.timestamp_ns for a, b in zip(out, out[1:])):
        raise ValueError("IMU timestamps must be strictly increasing")
    return out


def load_camera_csv(path: str | Path):
    path = Path(path)
    out = []
    for row in _data_rows(path):
        if len(row) != 2:
            raise ValueError(f"expected 2 camera columns, got {len(row)}")
        out.append(EurocCameraFrame(timestamp_ns=int(row[0]), filename=row[1].strip()))
    if any(a.timestamp_ns >= b.timestamp_ns for a, b in zip(out, out[1:])):
        raise ValueError("camera timestamps must be strictly increasing")
    return out


@dataclass(frozen=True)
class EurocGroundTruth:
    timestamp_ns: int
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    velocity_m_s: tuple[float, float, float]
    gyro_bias_rad_s: tuple[float, float, float]
    accel_bias_m_s2: tuple[float, float, float]


def load_groundtruth_csv(path: str | Path):
    path = Path(path)
    out = []
    for row in _data_rows(path):
        if len(row) < 17:
            raise ValueError(f"expected at least 17 ground-truth columns, got {len(row)}")
        out.append(EurocGroundTruth(
            timestamp_ns=int(row[0]),
            position_m=tuple(float(x) for x in row[1:4]),
            quaternion_wxyz=(float(row[4]), float(row[5]), float(row[6]), float(row[7])),
            velocity_m_s=tuple(float(x) for x in row[8:11]),
            gyro_bias_rad_s=tuple(float(x) for x in row[11:14]),
            accel_bias_m_s2=tuple(float(x) for x in row[14:17]),
        ))
    if any(a.timestamp_ns >= b.timestamp_ns for a, b in zip(out, out[1:])):
        raise ValueError("ground-truth timestamps must be strictly increasing")
    return out
