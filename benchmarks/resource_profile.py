#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,resource,sys,time
from pathlib import Path
import psutil

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from vionav.video_pipeline import process_frame,synthetic_video_frame

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--frames",type=int,default=100)
    ap.add_argument("--json-out",type=Path)
    args=ap.parse_args()
    if args.frames<=0: raise SystemExit("--frames must be positive")
    proc=psutil.Process()
    rss_before=proc.memory_info().rss
    cpu0=time.process_time(); wall0=time.perf_counter()
    features=0
    for i in range(args.frames):
        features += process_frame(synthetic_video_frame(i)).feature_count
    wall=time.perf_counter()-wall0
    cpu=time.process_time()-cpu0
    rss_after=proc.memory_info().rss
    max_rss_native=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    max_rss_bytes=max_rss_native if sys.platform=="darwin" else max_rss_native*1024
    result={
      "frames":args.frames,
      "wall_s":wall,
      "cpu_s":cpu,
      "mean_wall_ms_per_frame":1000*wall/args.frames,
      "mean_features":features/args.frames,
      "rss_before_mb":rss_before/(1024*1024),
      "rss_after_mb":rss_after/(1024*1024),
      "max_rss_mb":max_rss_bytes/(1024*1024),
    }
    text=json.dumps(result,indent=2,sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True,exist_ok=True)
        args.json_out.write_text(text+"\n",encoding="utf-8")
    print(text)
    return 0
if __name__=="__main__": raise SystemExit(main())
