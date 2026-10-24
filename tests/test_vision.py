import cv2
import numpy as np
import pytest

from vionav.vision import (
    PreprocessConfig,
    decide_keyframe,
    detect_features,
    essential_matrix_inliers,
    preprocess_grayscale,
    track_features,
)


def dot_image(points, shape=(240,320)):
    img=np.zeros(shape,dtype=np.uint8)
    for x,y in points:
        cv2.circle(img,(int(x),int(y)),3,255,-1,lineType=cv2.LINE_AA)
    return img


def test_preprocess_bgr_to_gray():
    bgr=np.zeros((20,30,3),dtype=np.uint8)
    bgr[...,1]=120
    out=preprocess_grayscale(bgr,PreprocessConfig(clahe_clip_limit=None))
    assert out.shape==(20,30)
    assert out.dtype==np.uint8


def test_detect_and_track_translation():
    pts=[(40+30*i,40+35*j) for i in range(8) for j in range(5)]
    a=dot_image(pts)
    M=np.float32([[1,0,4],[0,1,3]])
    b=cv2.warpAffine(a,M,(a.shape[1],a.shape[0]))
    features=detect_features(a,max_corners=100,min_distance_px=10)
    assert len(features)>=25
    tracks=track_features(a,b,features,fb_max_error_px=1.0)
    assert tracks.status.sum()>=20
    delta=tracks.valid_current_xy-tracks.valid_previous_xy
    med=np.median(delta,axis=0)
    assert med[0]==pytest.approx(4,abs=0.5)
    assert med[1]==pytest.approx(3,abs=0.5)


def project(points3, R, t, K):
    P=(R@points3.T).T+t
    uv=(K@P.T).T
    return uv[:,:2]/uv[:,2:3]


def test_essential_ransac_rejects_outliers():
    rng=np.random.default_rng(4)
    pts3=np.column_stack([rng.uniform(-2,2,80),rng.uniform(-1.2,1.2,80),rng.uniform(4,8,80)])
    K=np.array([[300,0,160],[0,300,120],[0,0,1]],dtype=float)
    angle=np.deg2rad(4)
    R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
    p0=project(pts3,np.eye(3),np.zeros(3),K)
    p1=project(pts3,R,np.array([0.15,0,0]),K)
    p1[:10]=rng.uniform([0,0],[319,239],(10,2))
    result=essential_matrix_inliers(p0,p1,K,ransac_threshold_px=1.0)
    assert result.essential_matrix is not None
    assert result.inlier_count>=60
    assert result.inlier_count<80


def test_keyframe_policy():
    p0=np.column_stack([np.arange(100),np.zeros(100)])
    p1=p0+np.array([2.0,0.0])
    d=decide_keyframe(p0,p1,frames_since_last_keyframe=2,min_tracked_features=50,parallax_threshold_px=10)
    assert not d.create_keyframe
    p2=p0+np.array([20.0,0.0])
    d2=decide_keyframe(p0,p2,frames_since_last_keyframe=3,min_tracked_features=50,parallax_threshold_px=10)
    assert d2.create_keyframe and d2.reason=="sufficient_parallax"
