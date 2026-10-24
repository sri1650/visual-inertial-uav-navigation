#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,json,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"src"))
from vionav.fault_campaign import run_retained_fault_campaign

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--csv-out",type=Path)
    args=ap.parse_args()
    rows=run_retained_fault_campaign()
    if args.csv_out:
        args.csv_out.parent.mkdir(parents=True,exist_ok=True)
        with args.csv_out.open("w",newline="",encoding="utf-8") as f:
            w=csv.writer(f);w.writerow(["case","state","score","reasons"])
            for r in rows:w.writerow([r.name,r.state.value,r.score,";".join(r.reasons)])
    print(json.dumps([{"case":r.name,"state":r.state.value,"score":r.score,"reasons":r.reasons} for r in rows],indent=2))
    return 0
if __name__=="__main__": raise SystemExit(main())
