#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from vionav.px4_bridge import MavlinkExternalVisionPublisher, make_external_vision_sample


def main() -> int:
    parser=argparse.ArgumentParser(description="Stream a deterministic external-vision pose to PX4 SITL")
    parser.add_argument("--endpoint",default="udpin:127.0.0.1:14540")
    parser.add_argument("--duration",type=float,default=10.0)
    parser.add_argument("--rate-hz",type=float,default=30.0)
    parser.add_argument("--quality",type=float,default=.9)
    parser.add_argument("--reset-counter",type=int,default=0)
    args=parser.parse_args()
    if args.duration<=0 or args.rate_hz<=0:
        parser.error("duration and rate-hz must be positive")

    publisher=MavlinkExternalVisionPublisher.connect(args.endpoint,min_quality=.2)
    interval=1.0/args.rate_hz
    deadline=time.monotonic()+args.duration
    sent=0
    try:
        while time.monotonic()<deadline:
            started=time.monotonic()
            timestamp_ns=time.monotonic_ns()
            sample=make_external_vision_sample(
                timestamp_ns=timestamp_ns,
                position_enu_m=(0.0,0.0,0.0),
                velocity_enu_m_s=(0.0,0.0,0.0),
                quality=args.quality,
                reset_counter=args.reset_counter,
            )
            if publisher.send(sample,now_us=timestamp_ns//1_000):
                sent+=1
            time.sleep(max(0.0,interval-(time.monotonic()-started)))
    finally:
        publisher.close()
    print(f"sent {sent} MAVLink ODOMETRY messages to {args.endpoint}")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
