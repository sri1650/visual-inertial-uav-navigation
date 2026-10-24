#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"src"))
from vionav.video_pipeline import benchmark_synthetic_frames

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--frames",type=int,default=100)
    ap.add_argument("--json-out",type=Path)
    args=ap.parse_args()
    result=benchmark_synthetic_frames(args.frames)
    text=json.dumps(result,indent=2,sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True,exist_ok=True)
        args.json_out.write_text(text+"\n",encoding="utf-8")
    print(text)
    return 0
if __name__=="__main__": raise SystemExit(main())
