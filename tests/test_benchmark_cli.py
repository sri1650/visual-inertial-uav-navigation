import json
from vionav.benchmark_cli import main


def test_cli(tmp_path,capsys):
    gt=tmp_path/'gt.csv'
    est=tmp_path/'est.csv'
    out=tmp_path/'report.json'
    gt.write_text('0,0,0\n1,0,0\n2,0,0\n')
    est.write_text('5,2,0\n6,2,0\n7,2,0\n')
    assert main(['--estimated',str(est),'--ground-truth',str(gt),'--output',str(out)])==0
    data=json.loads(out.read_text())
    assert data['ate_rmse_m'] < 1e-9
    assert data['rpe_translation_rmse_m'] < 1e-9
