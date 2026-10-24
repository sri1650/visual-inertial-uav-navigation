from __future__ import annotations
from dataclasses import replace


def shift_timestamps(samples, offset_ns: int):
    return [replace(s, timestamp_ns=int(s.timestamp_ns)+int(offset_ns)) for s in samples]


def drop_every_n(samples, n: int):
    if n <= 1:
        raise ValueError("n must be > 1")
    return [s for i,s in enumerate(samples,1) if i % n != 0]


def drop_burst(samples, start_index: int, count: int):
    if start_index < 0 or count < 0:
        raise ValueError("invalid burst")
    end=start_index+count
    return [s for i,s in enumerate(samples) if not (start_index <= i < end)]
