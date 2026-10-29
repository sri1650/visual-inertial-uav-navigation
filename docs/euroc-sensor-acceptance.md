# EuRoC sensor acquisition and replay acceptance

The executable sensor gate is:

```bash
python -m vionav.euroc_acceptance /path/to/MH_01_easy \
  --sequence MH_01_easy \
  --archive-checksum <verified archive checksum> \
  --output acceptance-data/MH_01_easy-sensor-audit.json
```

The default full-sequence criteria were fixed before a real sequence was available:

- at least 3,000 stereo pairs;
- at least 99.5% stereo pairing coverage;
- at most 1 ms left/right timestamp difference;
- at most 10 ms between adjacent IMU samples;
- at least 99.5% of stereo timestamps bracketed by IMU samples;
- every CSV-referenced camera file exists and no filename escapes its sensor data directory.

`vionav.euroc_replay.iter_sensor_events` merges camera and IMU measurements in timestamp order, with IMU first when timestamps match. It deliberately has no ground-truth input. Ground truth remains exclusive to the separate trajectory evaluator after an estimator has produced an estimate.

## Current data status

The official ETH Research Collection metadata artifact from GitHub Actions run `40888820740` records DOI `10.3929/ethz-b-000690084`. Its `machine_hall.zip` bitstream is 12,683,729,426 bytes with MD5 `363f5c2502b469cdd97ef85997714806`. No full sensor archive is cached in this repository or its Actions cache. Run `40894368252` encountered HTTP 429 during a bounded range request; its `BadZipFile` text is a wrapper error and is not evidence of archive corruption.

The deterministic fixture tests prove ingestion ordering, strict coverage/gap gates, provenance retention, path safety and ground-truth separation. They are not real EuRoC trajectory or accuracy evidence. P2 real-data accuracy therefore remains **BLOCKED** until an authorised full sequence is obtained once, verified against official provenance, and processed by a camera/IMU estimator that emits an independent trajectory.

