from pathlib import Path
import pytest
from vionav.euroc import load_imu_csv, load_camera_csv

def test_load_imu(tmp_path: Path):
    p = tmp_path/"imu.csv"
    p.write_text("#timestamp,wx,wy,wz,ax,ay,az\n100,1,2,3,4,5,6\n200,7,8,9,10,11,12\n")
    rows = load_imu_csv(p)
    assert rows[0].timestamp_ns == 100
    assert rows[1].accel_m_s2 == (10.0,11.0,12.0)

def test_load_camera(tmp_path: Path):
    p = tmp_path/"cam.csv"
    p.write_text("#timestamp,filename\n100,100.png\n200,200.png\n")
    rows = load_camera_csv(p)
    assert rows[1].filename == "200.png"

def test_reject_non_monotonic_imu(tmp_path: Path):
    p = tmp_path/"imu.csv"
    p.write_text("200,1,2,3,4,5,6\n100,7,8,9,10,11,12\n")
    with pytest.raises(ValueError):
        load_imu_csv(p)
