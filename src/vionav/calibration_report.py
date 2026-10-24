from __future__ import annotations
import json
from pathlib import Path
from .calibration import SensorCalibration


def build_calibration_report(calibration: SensorCalibration, *, time_sync_mse: float | None = None):
    calibration.validate()
    return {
        "camera": {
            "resolution": [calibration.camera.width, calibration.camera.height],
            "fx": calibration.camera.fx,
            "fy": calibration.camera.fy,
            "cx": calibration.camera.cx,
            "cy": calibration.camera.cy,
            "distortion_model": calibration.camera.distortion_model,
        },
        "camera_imu_time_offset_ns": calibration.camera_imu_time_offset_ns,
        "time_sync_mse": time_sync_mse,
        "quality_flags": {
            "intrinsics_valid": True,
            "extrinsics_valid": True,
            "time_offset_measured": calibration.camera_imu_time_offset_ns != 0 or time_sync_mse is not None,
        },
    }


def write_calibration_report(path: str | Path, report: dict) -> None:
    Path(path).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
