from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class PreprocessConfig:
    equalize_hist: bool = False
    clahe_clip_limit: float | None = 2.0
    clahe_grid: tuple[int, int] = (8, 8)
    gaussian_blur_sigma: float = 0.0


@dataclass(frozen=True)
class FeatureTrackResult:
    previous_xy: np.ndarray
    current_xy: np.ndarray
    status: np.ndarray
    error: np.ndarray

    @property
    def valid_previous_xy(self) -> np.ndarray:
        return self.previous_xy[self.status]

    @property
    def valid_current_xy(self) -> np.ndarray:
        return self.current_xy[self.status]


@dataclass(frozen=True)
class GeometricFilterResult:
    inlier_mask: np.ndarray
    essential_matrix: np.ndarray | None

    @property
    def inlier_count(self) -> int:
        return int(np.count_nonzero(self.inlier_mask))


@dataclass(frozen=True)
class KeyframeDecision:
    create_keyframe: bool
    reason: str
    tracked_features: int
    median_parallax_px: float
    frames_since_last_keyframe: int


def preprocess_grayscale(image: np.ndarray, config: PreprocessConfig = PreprocessConfig()) -> np.ndarray:
    if image is None or image.size == 0:
        raise ValueError("image is empty")
    if image.ndim == 3:
        if image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.shape[2] == 4:
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        else:
            raise ValueError("unsupported channel count")
    elif image.ndim == 2:
        gray = image.copy()
    else:
        raise ValueError("image must be HxW or HxWxC")

    if gray.dtype != np.uint8:
        arr = np.asarray(gray, dtype=np.float32)
        finite = np.isfinite(arr)
        if not np.any(finite):
            raise ValueError("image contains no finite pixels")
        lo = float(np.min(arr[finite]))
        hi = float(np.max(arr[finite]))
        if hi <= lo:
            gray = np.zeros(arr.shape, dtype=np.uint8)
        else:
            gray = np.clip((arr - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

    if config.gaussian_blur_sigma < 0:
        raise ValueError("gaussian_blur_sigma must be non-negative")
    if config.gaussian_blur_sigma > 0:
        gray = cv2.GaussianBlur(gray, (0, 0), config.gaussian_blur_sigma)

    if config.equalize_hist and config.clahe_clip_limit is not None:
        raise ValueError("choose global histogram equalisation or CLAHE, not both")
    if config.equalize_hist:
        gray = cv2.equalizeHist(gray)
    elif config.clahe_clip_limit is not None:
        if config.clahe_clip_limit <= 0:
            raise ValueError("CLAHE clip limit must be positive")
        clahe = cv2.createCLAHE(clipLimit=config.clahe_clip_limit, tileGridSize=config.clahe_grid)
        gray = clahe.apply(gray)
    return gray


def detect_features(
    gray: np.ndarray,
    *,
    max_corners: int = 500,
    quality_level: float = 0.01,
    min_distance_px: float = 8.0,
    block_size: int = 7,
) -> np.ndarray:
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise ValueError("detect_features expects uint8 grayscale image")
    if max_corners <= 0 or not (0 < quality_level <= 1) or min_distance_px < 0:
        raise ValueError("invalid feature detector configuration")
    pts = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=max_corners,
        qualityLevel=quality_level,
        minDistance=min_distance_px,
        blockSize=block_size,
        useHarrisDetector=False,
    )
    if pts is None:
        return np.empty((0, 2), dtype=np.float32)
    return pts.reshape(-1, 2).astype(np.float32)


def track_features(
    previous_gray: np.ndarray,
    current_gray: np.ndarray,
    previous_xy: np.ndarray,
    *,
    win_size: tuple[int, int] = (21, 21),
    max_level: int = 3,
    fb_max_error_px: float = 1.5,
) -> FeatureTrackResult:
    prev = np.asarray(previous_xy, dtype=np.float32).reshape(-1, 2)
    if previous_gray.ndim != 2 or current_gray.ndim != 2:
        raise ValueError("track_features expects grayscale images")
    if len(prev) == 0:
        return FeatureTrackResult(prev, prev.copy(), np.zeros(0, dtype=bool), np.zeros(0, dtype=np.float32))
    nxt, st_fwd, err = cv2.calcOpticalFlowPyrLK(
        previous_gray, current_gray, prev.reshape(-1, 1, 2), None,
        winSize=win_size, maxLevel=max_level,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    if nxt is None or st_fwd is None:
        return FeatureTrackResult(prev, np.zeros_like(prev), np.zeros(len(prev), dtype=bool), np.full(len(prev), np.inf, dtype=np.float32))
    back, st_back, _ = cv2.calcOpticalFlowPyrLK(
        current_gray, previous_gray, nxt, None,
        winSize=win_size, maxLevel=max_level,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    nxt2 = nxt.reshape(-1, 2)
    back2 = back.reshape(-1, 2) if back is not None else np.full_like(prev, np.nan)
    st = st_fwd.reshape(-1).astype(bool)
    if st_back is not None:
        st &= st_back.reshape(-1).astype(bool)
    fb_error = np.linalg.norm(back2 - prev, axis=1)
    st &= np.isfinite(fb_error) & (fb_error <= fb_max_error_px)
    lk_error = err.reshape(-1).astype(np.float32) if err is not None else fb_error.astype(np.float32)
    return FeatureTrackResult(prev, nxt2, st, lk_error)


def essential_matrix_inliers(
    previous_xy: np.ndarray,
    current_xy: np.ndarray,
    camera_matrix: np.ndarray,
    *,
    ransac_threshold_px: float = 1.0,
    probability: float = 0.999,
) -> GeometricFilterResult:
    p0 = np.asarray(previous_xy, dtype=np.float64).reshape(-1, 2)
    p1 = np.asarray(current_xy, dtype=np.float64).reshape(-1, 2)
    K = np.asarray(camera_matrix, dtype=np.float64)
    if len(p0) != len(p1):
        raise ValueError("point arrays must have equal length")
    if K.shape != (3, 3):
        raise ValueError("camera_matrix must be 3x3")
    if len(p0) < 5:
        return GeometricFilterResult(np.zeros(len(p0), dtype=bool), None)
    E, mask = cv2.findEssentialMat(
        p0, p1, K,
        method=cv2.RANSAC,
        prob=probability,
        threshold=ransac_threshold_px,
    )
    if E is None or mask is None:
        return GeometricFilterResult(np.zeros(len(p0), dtype=bool), None)
    inlier = mask.reshape(-1).astype(bool)
    if len(inlier) != len(p0):
        inlier = inlier[:len(p0)]
        if len(inlier) < len(p0):
            inlier = np.pad(inlier, (0, len(p0)-len(inlier)), constant_values=False)
    return GeometricFilterResult(inlier, np.asarray(E, dtype=np.float64))


def decide_keyframe(
    previous_xy: np.ndarray,
    current_xy: np.ndarray,
    *,
    frames_since_last_keyframe: int,
    min_tracked_features: int = 80,
    parallax_threshold_px: float = 18.0,
    max_frames_between_keyframes: int = 12,
) -> KeyframeDecision:
    p0 = np.asarray(previous_xy, dtype=np.float64).reshape(-1, 2)
    p1 = np.asarray(current_xy, dtype=np.float64).reshape(-1, 2)
    if len(p0) != len(p1):
        raise ValueError("point arrays must have equal length")
    if frames_since_last_keyframe < 0:
        raise ValueError("frames_since_last_keyframe must be non-negative")
    parallax = np.linalg.norm(p1 - p0, axis=1) if len(p0) else np.empty(0)
    median = float(np.median(parallax)) if len(parallax) else 0.0
    tracked = len(p0)
    if tracked < min_tracked_features:
        return KeyframeDecision(True, "low_feature_count", tracked, median, frames_since_last_keyframe)
    if median >= parallax_threshold_px:
        return KeyframeDecision(True, "sufficient_parallax", tracked, median, frames_since_last_keyframe)
    if frames_since_last_keyframe >= max_frames_between_keyframes:
        return KeyframeDecision(True, "max_interval", tracked, median, frames_since_last_keyframe)
    return KeyframeDecision(False, "continue_tracking", tracked, median, frames_since_last_keyframe)
