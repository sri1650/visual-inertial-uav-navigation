from __future__ import annotations
import json
from pathlib import Path
from .benchmark import TrajectoryMetrics

def write_metrics_report(path: str | Path, metrics: TrajectoryMetrics, extra: dict | None=None):
    out={
      "ate_rmse_m":metrics.ate_rmse_m,
      "rpe_translation_rmse_m":metrics.rpe_translation_rmse_m,
      "samples":metrics.samples,
    }
    if extra:
        out.update(extra)
    Path(path).write_text(json.dumps(out,indent=2,sort_keys=True)+"\n",encoding="utf-8")
