import pytest
from vionav.calibration import SensorCalibration

IDENTITY = [[1,0,0],[0,1,0],[0,0,1]]

def valid():
    return {
        "camera":{"fx":400,"fy":401,"cx":320,"cy":240,"width":640,"height":480,"distortion":[0,0,0,0]},
        "T_body_camera":{"rotation":IDENTITY,"translation_m":[0.1,0,0]},
        "T_body_imu":{"rotation":IDENTITY,"translation_m":[0,0,0]},
        "camera_imu_time_offset_ns":12000000,
    }

def test_valid_calibration():
    c = SensorCalibration.from_dict(valid())
    assert c.camera.fx == 400
    assert c.camera_imu_time_offset_ns == 12000000

def test_reject_bad_focal_length():
    d = valid()
    d["camera"]["fx"] = 0
    with pytest.raises(ValueError):
        SensorCalibration.from_dict(d)

def test_reject_non_orthogonal_rotation():
    d = valid()
    d["T_body_camera"]["rotation"] = [[1,0,0],[1,1,0],[0,0,1]]
    with pytest.raises(ValueError):
        SensorCalibration.from_dict(d)
