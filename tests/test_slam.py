import numpy as np
from vionav.slam import Keyframe,KeyframeDatabase,RelativePositionConstraint,TranslationPoseGraph

def test_keyframe_candidates_are_ranked_and_recent_excluded():
    db=KeyframeDatabase()
    for i,d in [(0,[1,0,0]),(20,[.98,.02,0]),(30,[0,1,0])]:
        db.add(Keyframe(i,i*10,(i,0,0),np.array(d,dtype=float)))
    q=Keyframe(40,400,(40,0,0),np.array([1,0,0],dtype=float))
    c=db.query(q,exclude_recent=5,min_similarity=.8)
    assert [x.match_id for x in c]==[0,20]
    assert c[0].similarity >= c[1].similarity

def test_duplicate_keyframe_rejected():
    db=KeyframeDatabase()
    k=Keyframe(1,1,(0,0,0),np.array([1,0]))
    db.add(k)
    import pytest
    with pytest.raises(ValueError): db.add(k)

def test_translation_pose_graph_loop_corrects_drift():
    g=TranslationPoseGraph()
    # Drifted initial estimates.
    g.add_node(0,(0,0,0))
    g.add_node(1,(1.1,0,0))
    g.add_node(2,(2.2,0,0))
    g.add_node(3,(3.3,0,0))
    for a,b in [(0,1),(1,2),(2,3)]:
        g.add_constraint(RelativePositionConstraint(a,b,(1,0,0),1.0))
    # Loop-style global correction says node 3 should be 3.0m from node 0.
    g.add_constraint(RelativePositionConstraint(0,3,(3,0,0),5.0))
    solved=g.optimise(anchor_id=0)
    assert abs(solved[3][0]-3.0) < 1e-9
    assert solved[1][0] < 1.1
