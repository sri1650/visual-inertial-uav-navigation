#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,shutil,subprocess,sys,tempfile,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def run(name,cmd,timeout=180,cwd=ROOT):
    t=time.perf_counter()
    p=subprocess.run(cmd,cwd=cwd,capture_output=True,text=True,timeout=timeout)
    return {"name":name,"returncode":p.returncode,"elapsed_s":round(time.perf_counter()-t,3),
            "stdout_tail":p.stdout[-2500:],"stderr_tail":p.stderr[-1500:]}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--json-out",type=Path)
    args=ap.parse_args()
    build=Path(tempfile.gettempdir())/"vionav-pre-codex-cpp"
    if build.exists(): shutil.rmtree(build)
    checks=[
      run("pytest",[sys.executable,"-m","pytest","-q"],120),
      run("compileall",[sys.executable,"-m","compileall","-q","src","tests"],60),
      run("cmake_configure",["cmake","-S","cpp","-B",str(build)],60),
    ]
    if checks[-1]["returncode"]==0:
        checks.append(run("cpp_build",["cmake","--build",str(build),"--parallel"],120))
        if checks[-1]["returncode"]==0:
            checks.append(run("ctest",["ctest","--test-dir",str(build),"--output-on-failure"],60))
    checks += [
      run("video_benchmark",[sys.executable,"benchmarks/video/run.py","--frames","10"],60),
      run("fault_campaign",[sys.executable,"benchmarks/faults/run.py"],60),
      run("resource_profile",[sys.executable,"benchmarks/resource_profile.py","--frames","10"],60),
    ]
    payload={
      "status":"pass" if all(x["returncode"]==0 for x in checks) else "fail",
      "checks":checks,
      "reserved_for_codex":[
        "production loop closure / 6-DoF pose graph backend",
        "ARM hardware performance optimisation",
        "real EuRoC estimator runs and final performance claims",
      ]
    }
    text=json.dumps(payload,indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True,exist_ok=True)
        args.json_out.write_text(text+"\n",encoding="utf-8")
    print(text)
    return 0 if payload["status"]=="pass" else 2

if __name__=="__main__": raise SystemExit(main())
