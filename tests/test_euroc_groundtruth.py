from pathlib import Path
from vionav.euroc import load_groundtruth_csv

def test_load_groundtruth(tmp_path: Path):
    p=tmp_path/"gt.csv"
    p.write_text("#t,p,q,v,bg,ba\n100,1,2,3,1,0,0,0,4,5,6,0.1,0.2,0.3,0.4,0.5,0.6\n")
    g=load_groundtruth_csv(p)
    assert g[0].position_m == (1.0,2.0,3.0)
    assert g[0].quaternion_wxyz == (1.0,0.0,0.0,0.0)
