#!/usr/bin/env python3
"""Plot measured tracking and 60-second hover results, preserving failed runs."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('core_directory');a=p.parse_args()
    core=Path(a.core_directory).resolve();experiments=core.parent
    tracking=[];hover=[]
    for run in ['07','08']:
        directory=experiments/('20261003_core_live_'+run)/'three_cold_navigation_attempt_01/round_01'
        source=directory/'tracking_audit.json'
        data=json.loads(source.read_text())
        tracking.append({'label':'Position only '+run,'source':str(source),
                         'max_error_m':data['tracking_error_max_m'],
                         'p95_error_m':data['tracking_error_p95_m'],'whole_suite_passed':False})
        reports=sorted(directory.glob('hold_check*.json'))
        for index,source in enumerate(reports):
            data=json.loads(source.read_text())
            if 'truth_peak_to_peak' in data:
                hover.append({'label':'Run '+run+(' pre' if index==0 else ' post'),
                              'source':str(source),'range_m':data['truth_peak_to_peak'][2]})
    for directory in sorted((core/'three_cold_navigation_attempt_01').glob('round_*')):
        source=directory/'world_geometry_audit.json'
        if source.exists():
            data=json.loads(source.read_text())
            metric=data['navigation_tracking']
            tracking.append({'label':'Feedforward '+directory.name[-2:], 'source':str(source),
                             'max_error_m':metric['max_error_m'],'p95_error_m':metric['p95_error_m'],
                             'whole_suite_passed':json.loads((directory/'suite_result.json').read_text())['passed']})
        for index,source in enumerate(sorted(directory.glob('hold_check*.json'))):
            data=json.loads(source.read_text())
            if 'truth_peak_to_peak' in data:
                hover.append({'label':directory.name[-2:]+(' pre' if index==0 else ' post'),
                              'source':str(source),'range_m':data['truth_peak_to_peak'][2]})
    output=core/'optimization_comparison'
    output.mkdir(exist_ok=True)
    fig,axes=plt.subplots(1,2,figsize=(13,4.5))
    axes[0].bar(range(len(tracking)),[r['max_error_m'] for r in tracking],color=['#b45309']*2+['#0369a1']*(len(tracking)-2))
    axes[0].set_xticks(range(len(tracking)));axes[0].set_xticklabels([r['label'] for r in tracking],rotation=25,ha='right')
    axes[0].set(ylabel='Maximum command / latest FCU pose error (m)',title='Measured navigation tracking')
    axes[1].bar(range(len(hover)),[r['range_m'] for r in hover],color='#0369a1')
    axes[1].set_xticks(range(len(hover)));axes[1].set_xticklabels([r['label'] for r in hover],rotation=45,ha='right')
    axes[1].axhline(.15,color='red',linestyle='--',label='0.15 m limit');axes[1].legend()
    axes[1].set(ylabel='Truth height peak-to-peak (m)',title='60 wall-second hover')
    fig.tight_layout();fig.savefig(output/'tracking_and_hover.png',dpi=180);plt.close(fig)
    limits=['Tracking compares latest received FCU pose, without timestamp interpolation.',
            'Before/after flights have multiple implementation differences; this is not an isolated causal experiment.',
            'Failed prior suites are preserved and labelled, not certified by these charts.',
            'This plot alone does not certify fault cases or hardware readiness.']
    (output/'measurements.json').write_text(json.dumps({'tracking':tracking,'hover':hover,'limits':limits},indent=2)+'\n')
    lines=['# 导航优化实验数据','', '| 控制/轮次 | 最大跟踪偏差 m | P95 m | 整轮通过 |', '| --- | --- | --- | --- |']
    for row in tracking:
        lines.append('| %s | %.6f | %.6f | %s |'%(row['label'],row['max_error_m'],row['p95_error_m'],row['whole_suite_passed']))
    lines+=['','跟踪按最新接收的FCU位置计算，没有时间插值。前后实验并非单变量对照；保留旧失败结果。',
            '悬停均为60秒墙钟。图表不代表全部故障通过或实机可用。','', '![实验对比](tracking_and_hover.png)']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(str(output))


if __name__=='__main__':main()
