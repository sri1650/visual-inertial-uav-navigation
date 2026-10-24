from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .benchmark import trajectory_metrics


def load_xyz_csv(path: Path):
    out=[]
    with path.open('r',encoding='utf-8',newline='') as f:
        for row in csv.reader(f):
            if not row or row[0].lstrip().startswith('#'):
                continue
            if len(row) < 3:
                raise ValueError(f'{path}: expected at least 3 columns')
            vals=row[-3:]
            out.append(tuple(float(x) for x in vals))
    return out


def main(argv=None):
    p=argparse.ArgumentParser(description='Compute aligned trajectory ATE/RPE from XYZ CSV files.')
    p.add_argument('--estimated',required=True,type=Path)
    p.add_argument('--ground-truth',required=True,type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--no-align',action='store_true')
    args=p.parse_args(argv)
    metrics=trajectory_metrics(load_xyz_csv(args.estimated),load_xyz_csv(args.ground_truth),align=not args.no_align)
    result={
        'ate_rmse_m':metrics.ate_rmse_m,
        'rpe_translation_rmse_m':metrics.rpe_translation_rmse_m,
        'samples':metrics.samples,
    }
    text=json.dumps(result,indent=2,sort_keys=True)
    if args.output:
        args.output.write_text(text+'\n',encoding='utf-8')
    print(text)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
