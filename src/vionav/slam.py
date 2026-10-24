from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Keyframe:
    frame_id: int
    timestamp_ns: int
    position_xyz: tuple[float, float, float]
    descriptor: np.ndarray


@dataclass(frozen=True)
class LoopCandidate:
    query_id: int
    match_id: int
    similarity: float


class KeyframeDatabase:
    """Deterministic descriptor database for loop-closure candidate retrieval.

    This is deliberately backend-agnostic: production descriptors can come from
    ORB/BoW/learned retrieval without changing the API.
    """

    def __init__(self) -> None:
        self._frames: dict[int, Keyframe] = {}

    def __len__(self) -> int:
        return len(self._frames)

    def add(self, frame: Keyframe) -> None:
        if frame.frame_id in self._frames:
            raise ValueError(f"duplicate keyframe id {frame.frame_id}")
        desc = np.asarray(frame.descriptor, dtype=np.float64).reshape(-1)
        if desc.size == 0 or not np.all(np.isfinite(desc)):
            raise ValueError("descriptor must be finite and non-empty")
        norm = float(np.linalg.norm(desc))
        if norm <= 0:
            raise ValueError("descriptor norm must be positive")
        self._frames[frame.frame_id] = Keyframe(
            frame.frame_id,
            frame.timestamp_ns,
            tuple(float(x) for x in frame.position_xyz),
            (desc / norm).copy(),
        )

    def get(self, frame_id: int) -> Keyframe:
        return self._frames[frame_id]

    def query(
        self,
        query: Keyframe,
        *,
        top_k: int = 5,
        exclude_recent: int = 10,
        min_similarity: float = 0.7,
    ) -> list[LoopCandidate]:
        if top_k <= 0 or exclude_recent < 0:
            raise ValueError("invalid query parameters")
        q = np.asarray(query.descriptor, dtype=np.float64).reshape(-1)
        if q.size == 0:
            return []
        qn = np.linalg.norm(q)
        if qn <= 0:
            return []
        q = q / qn
        out=[]
        for frame_id, frame in self._frames.items():
            if abs(query.frame_id - frame_id) <= exclude_recent:
                continue
            if frame.descriptor.shape != q.shape:
                continue
            sim=float(np.dot(q, frame.descriptor))
            if sim >= min_similarity:
                out.append(LoopCandidate(query.frame_id, frame_id, sim))
        out.sort(key=lambda c:(-c.similarity,c.match_id))
        return out[:top_k]


@dataclass(frozen=True)
class RelativePositionConstraint:
    source: int
    target: int
    delta_xyz: tuple[float, float, float]
    weight: float = 1.0


class TranslationPoseGraph:
    """Small linear position-only pose-graph prototype.

    It is a testable correction boundary, not the final 6-DoF SLAM backend.
    Codex should replace/extend this with the chosen production pose-graph
    backend after VIO poses are available.
    """

    def __init__(self) -> None:
        self.positions: dict[int, np.ndarray] = {}
        self.constraints: list[RelativePositionConstraint] = []

    def add_node(self, node_id: int, position_xyz) -> None:
        if node_id in self.positions:
            raise ValueError("duplicate pose-graph node")
        p=np.asarray(position_xyz,dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            raise ValueError("position must be finite")
        self.positions[node_id]=p

    def add_constraint(self, constraint: RelativePositionConstraint) -> None:
        if constraint.source not in self.positions or constraint.target not in self.positions:
            raise KeyError("constraint references unknown node")
        if constraint.weight <= 0:
            raise ValueError("constraint weight must be positive")
        self.constraints.append(constraint)

    def optimise(self, *, anchor_id: int | None = None) -> dict[int, np.ndarray]:
        if not self.positions:
            return {}
        ids=sorted(self.positions)
        anchor=ids[0] if anchor_id is None else anchor_id
        if anchor not in self.positions:
            raise KeyError("anchor node missing")
        variable=[i for i in ids if i != anchor]
        index={node:i for i,node in enumerate(variable)}

        rows=[]
        rhs=[[],[],[]]
        # Relative constraints: p_target - p_source = delta.
        for c in self.constraints:
            row=np.zeros(len(variable),dtype=float)
            w=float(c.weight)**0.5
            if c.target != anchor:
                row[index[c.target]] += w
            if c.source != anchor:
                row[index[c.source]] -= w
            anchor_term=np.zeros(3)
            if c.target == anchor:
                anchor_term -= self.positions[anchor]
            if c.source == anchor:
                anchor_term += self.positions[anchor]
            delta=np.asarray(c.delta_xyz,dtype=float)
            rows.append(row)
            target=(delta + anchor_term)*w
            for axis in range(3):
                rhs[axis].append(target[axis])

        if not rows:
            return {k:v.copy() for k,v in self.positions.items()}

        A=np.vstack(rows)
        result={anchor:self.positions[anchor].copy()}
        solved=[]
        for axis in range(3):
            x,*_=np.linalg.lstsq(A,np.asarray(rhs[axis]),rcond=None)
            solved.append(x)
        for node in variable:
            i=index[node]
            result[node]=np.array([solved[0][i],solved[1][i],solved[2][i]],dtype=float)
        return result
