from __future__ import annotations
from bisect import bisect_left
from typing import Iterable

def pair_nearest(reference_ns: Iterable[int], candidate_ns: Iterable[int], max_delta_ns: int):
    refs = list(reference_ns)
    cands = sorted(candidate_ns)
    out = []
    if max_delta_ns < 0:
        raise ValueError("max_delta_ns must be non-negative")
    for r in refs:
        i = bisect_left(cands, r)
        choices = []
        if i < len(cands):
            choices.append(cands[i])
        if i:
            choices.append(cands[i-1])
        if not choices:
            out.append((r, None, None))
            continue
        c = min(choices, key=lambda x: (abs(x-r), x))
        delta = c-r
        out.append((r, c if abs(delta) <= max_delta_ns else None,
                    delta if abs(delta) <= max_delta_ns else None))
    return out
