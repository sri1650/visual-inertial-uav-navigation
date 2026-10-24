import json
from vionav.calibration import SensorCalibration
from vionav.calibration_report import build_calibration_report, write_calibration_report

def fixture():
    I=[[1,0,0],[0,1,0],[0,0,1]]
    return SensorCalibration.from_dict({
      "camera":{"fx":400,"fy":400,"cx":320,"cy":240,"width":640,"height":480},
      "T_body_camera":{"rotation":I,"translation_m":[0,0,0]},
      "T_body_imu":{"rotation":I,"translation_m":[0,0,0]},
      "camera_imu_time_offset_ns":12000000,
    })

def test_report_is_deterministic(tmp_path):
    r=build_calibration_report(fixture(),time_sync_mse=0.001)
    p=tmp_path/"report.json"
    write_calibration_report(p,r)
    a=p.read_text()
    write_calibration_report(p,r)
    assert p.read_text()==a
    assert json.loads(a)["quality_flags"]["intrinsics_valid"] is True
