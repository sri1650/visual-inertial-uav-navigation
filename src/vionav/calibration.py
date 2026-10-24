from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Iterable


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion_model: str = "radtan"
    distortion: tuple[float, ...] = ()

    def validate(self) -> None:
        vals = (self.fx, self.fy, self.cx, self.cy, *self.distortion)
        if not all(isfinite(float(v)) for v in vals):
            raise ValueError("camera intrinsics contain non-finite values")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("focal lengths must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image dimensions must be positive")
        if not (0 <= self.cx < self.width and 0 <= self.cy < self.height):
            raise ValueError("principal point must lie inside the image")


@dataclass(frozen=True)
class RigidTransform:
    rotation: tuple[tuple[float, float, float], ...]
    translation_m: tuple[float, float, float]

    def validate(self, tol: float = 1e-3) -> None:
        if len(self.rotation) != 3 or any(len(r) != 3 for r in self.rotation):
            raise ValueError("rotation must be 3x3")
        if len(self.translation_m) != 3:
            raise ValueError("translation must have 3 elements")
        vals = [*self.translation_m, *(v for row in self.rotation for v in row)]
        if not all(isfinite(float(v)) for v in vals):
            raise ValueError("transform contains non-finite values")

        # Check orthonormal rows.
        rows = self.rotation
        for r in rows:
            n = sqrt(sum(float(x) ** 2 for x in r))
            if abs(n - 1.0) > tol:
                raise ValueError("rotation rows must be unit length")
        for i in range(3):
            for j in range(i + 1, 3):
                dot = sum(float(rows[i][k]) * float(rows[j][k]) for k in range(3))
                if abs(dot) > tol:
                    raise ValueError("rotation rows must be orthogonal")

        # Right-handed determinant.
        a, b, c = rows
        det = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
        if abs(det - 1.0) > 5 * tol:
            raise ValueError("rotation determinant must be +1")


@dataclass(frozen=True)
class SensorCalibration:
    camera: CameraIntrinsics
    T_body_camera: RigidTransform
    T_body_imu: RigidTransform
    camera_imu_time_offset_ns: int = 0

    def validate(self) -> None:
        self.camera.validate()
        self.T_body_camera.validate()
        self.T_body_imu.validate()

    @staticmethod
    def from_dict(data: dict) -> "SensorCalibration":
        cam = data["camera"]
        camera = CameraIntrinsics(
            fx=float(cam["fx"]),
            fy=float(cam["fy"]),
            cx=float(cam["cx"]),
            cy=float(cam["cy"]),
            width=int(cam["width"]),
            height=int(cam["height"]),
            distortion_model=str(cam.get("distortion_model", "radtan")),
            distortion=tuple(float(v) for v in cam.get("distortion", [])),
        )

        def parse_tf(key: str) -> RigidTransform:
            raw = data[key]
            return RigidTransform(
                rotation=tuple(tuple(float(v) for v in row) for row in raw["rotation"]),
                translation_m=tuple(float(v) for v in raw["translation_m"]),
            )

        out = SensorCalibration(
            camera=camera,
            T_body_camera=parse_tf("T_body_camera"),
            T_body_imu=parse_tf("T_body_imu"),
            camera_imu_time_offset_ns=int(data.get("camera_imu_time_offset_ns", 0)),
        )
        out.validate()
        return out
