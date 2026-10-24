import cv2, numpy as np
from vionav.geolocalization import *

def scene(seed):
    rng=np.random.default_rng(seed)
    im=np.zeros((320,420),dtype=np.uint8)
    for _ in range(60):
        x,y=rng.integers(15,405),rng.integers(15,305)
        r=int(rng.integers(2,7))
        cv2.circle(im,(int(x),int(y)),r,int(rng.integers(100,255)),-1)
    cv2.putText(im,f"REF-{seed}",(80,160),cv2.FONT_HERSHEY_SIMPLEX,1.3,255,2)
    return im

def test_offline_reference_localisation():
    a=scene(1); b=scene(2)
    M=np.float32([[1,0,5],[0,1,3]])
    query=cv2.warpAffine(a,M,(a.shape[1],a.shape[0]))
    refs=[MapReference("a",a,52.0,-1.9),MapReference("b",b,53.0,-2.0)]
    r=localise_against_references(query,refs,min_matches=8,min_inliers=6,min_confidence=.3)
    assert r.accepted
    assert r.reference_id=="a"
    assert r.latitude_deg==52.0

def test_low_texture_rejected():
    blank=np.zeros((200,200),dtype=np.uint8)
    r=localise_against_references(blank,[MapReference("a",scene(1),1,2)])
    assert not r.accepted
    assert r.reason=="insufficient_query_features"
