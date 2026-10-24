from __future__ import annotations

from bisect import bisect_left
from math import inf
from typing import Sequence


def _interp(times: Sequence[int], values: Sequence[float], t: int):
    if len(times) != len(values):
        raise ValueError("times and values length mismatch")
    if len(times) < 2:
        return None
    i = bisect_left(times, t)
    if i == 0 or i >= len(times):
        return None
    t0, t1 = times[i - 1], times[i]
    if t1 == t0:
        return float(values[i])
    u = (t - t0) / (t1 - t0)
    return float(values[i - 1]) * (1.0 - u) + float(values[i]) * u


def estimate_time_offset_ns(
    reference_times_ns: Sequence[int],
    reference_signal: Sequence[float],
    candidate_times_ns: Sequence[int],
    candidate_signal: Sequence[float],
    *,
    search_min_ns: int,
    search_max_ns: int,
    step_ns: int,
    min_pairs: int = 10,
):
    """Estimate candidate timestamp offset relative to reference.

    For each offset `o`, candidate values are interpolated at `reference_time + o`.
    The returned offset minimises mean squared signal residual.

    Positive result means the candidate signal is sampled later than reference.
    """
    if step_ns <= 0:
        raise ValueError("step_ns must be positive")
    if search_min_ns > search_max_ns:
        raise ValueError("invalid search range")
    if len(reference_times_ns) != len(reference_signal):
        raise ValueError("reference length mismatch")
    if len(candidate_times_ns) != len(candidate_signal):
        raise ValueError("candidate length mismatch")
    if sorted(candidate_times_ns) != list(candidate_times_ns):
        raise ValueError("candidate timestamps must be sorted")

    best = None
    best_mse = inf
    best_pairs = 0

    offset = search_min_ns
    while offset <= search_max_ns:
        sq = []
        for t, r in zip(reference_times_ns, reference_signal):
            c = _interp(candidate_times_ns, candidate_signal, t + offset)
            if c is not None:
                sq.append((float(r) - c) ** 2)
        if len(sq) >= min_pairs:
            mse = sum(sq) / len(sq)
            if mse < best_mse:
                best = offset
                best_mse = mse
                best_pairs = len(sq)
        offset += step_ns

    if best is None:
        raise ValueError("insufficient overlapping samples")
    return {
        "offset_ns": int(best),
        "mse": float(best_mse),
        "pairs": int(best_pairs),
    }
