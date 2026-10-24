from __future__ import annotations
import math

def ate_rmse(estimated_xyz, ground_truth_xyz) -> float:
    est = list(estimated_xyz)
    gt = list(ground_truth_xyz)
    if len(est) != len(gt):
        raise ValueError("trajectory lengths differ")
    if not est:
        raise ValueError("trajectory is empty")
    squared = []
    for a, b in zip(est, gt):
        if len(a) != 3 or len(b) != 3:
            raise ValueError("positions must be xyz triples")
        squared.append(sum((float(x)-float(y))**2 for x, y in zip(a,b)))
    return math.sqrt(sum(squared)/len(squared))


def rpe_translation_rmse(estimated_xyz, ground_truth_xyz, delta: int = 1) -> float:
    est = list(estimated_xyz)
    gt = list(ground_truth_xyz)
    if len(est) != len(gt):
        raise ValueError("trajectory lengths differ")
    if delta <= 0:
        raise ValueError("delta must be positive")
    if len(est) <= delta:
        raise ValueError("trajectory too short for delta")
    errors = []
    for i in range(len(est) - delta):
        de = tuple(float(est[i+delta][k]) - float(est[i][k]) for k in range(3))
        dg = tuple(float(gt[i+delta][k]) - float(gt[i][k]) for k in range(3))
        errors.append(sum((de[k] - dg[k]) ** 2 for k in range(3)))
    return math.sqrt(sum(errors) / len(errors))
