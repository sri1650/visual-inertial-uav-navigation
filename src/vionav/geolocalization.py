from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence
import cv2
import numpy as np


@dataclass(frozen=True)
class MapReference:
    reference_id: str
    image: np.ndarray
    latitude_deg: float
    longitude_deg: float


@dataclass(frozen=True)
class GeoLocalisationResult:
    reference_id: str | None
    latitude_deg: float | None
    longitude_deg: float | None
    confidence: float
    inliers: int
    matches: int
    accepted: bool
    reason: str


def _orb(image: np.ndarray, max_features: int=800):
    if image is None or image.size == 0:
        raise ValueError("image is empty")
    gray=image if image.ndim==2 else cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    detector=cv2.ORB_create(nfeatures=max_features)
    kp,desc=detector.detectAndCompute(gray,None)
    return kp,desc


def localise_against_references(
    query: np.ndarray,
    references: Sequence[MapReference],
    *,
    ratio_test: float=.75,
    min_matches: int=12,
    min_inliers: int=8,
    min_confidence: float=.35,
) -> GeoLocalisationResult:
    if not references:
        return GeoLocalisationResult(None,None,None,0,0,0,False,"no_references")
    qkp,qdesc=_orb(query)
    if qdesc is None or len(qkp)<4:
        return GeoLocalisationResult(None,None,None,0,0,0,False,"insufficient_query_features")
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    best=None
    for ref in references:
        rkp,rdesc=_orb(ref.image)
        if rdesc is None or len(rkp)<4:
            continue
        knn=matcher.knnMatch(qdesc,rdesc,k=2)
        good=[a for a,b in knn if a.distance < ratio_test*b.distance]
        if len(good)<4:
            candidate=(ref,0,len(good),0.0)
        else:
            src=np.float32([qkp[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            dst=np.float32([rkp[m.trainIdx].pt for m in good]).reshape(-1,1,2)
            _,mask=cv2.findHomography(src,dst,cv2.RANSAC,4.0)
            inliers=int(mask.sum()) if mask is not None else 0
            confidence=inliers/max(1,len(good))
            candidate=(ref,inliers,len(good),confidence)
        if best is None or (candidate[1],candidate[3],candidate[2],candidate[0].reference_id) > (best[1],best[3],best[2],best[0].reference_id):
            best=candidate

    if best is None:
        return GeoLocalisationResult(None,None,None,0,0,0,False,"no_match")
    ref,inliers,matches,confidence=best
    accepted=matches>=min_matches and inliers>=min_inliers and confidence>=min_confidence
    return GeoLocalisationResult(
        ref.reference_id,
        ref.latitude_deg if accepted else None,
        ref.longitude_deg if accepted else None,
        float(confidence),inliers,matches,accepted,
        "accepted" if accepted else "low_confidence",
    )
