from __future__ import annotations

import argparse
import json
from pathlib import Path

from .euroc_replay import EurocAuditThresholds, audit_euroc_sequence


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Audit an extracted EuRoC camera/IMU sequence without reading ground truth"
    )
    parser.add_argument("root", type=Path)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--archive-checksum", required=True)
    parser.add_argument("--archive-checksum-algorithm", default="MD5")
    parser.add_argument("--minimum-stereo-pairs", type=int, default=3000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = audit_euroc_sequence(
        args.root,
        sequence=args.sequence,
        archive_checksum=args.archive_checksum,
        archive_checksum_algorithm=args.archive_checksum_algorithm,
        thresholds=EurocAuditThresholds(minimum_stereo_pairs=args.minimum_stereo_pairs),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report.to_dict(), indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report.to_dict(), sort_keys=True))
    return 0 if report.status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

