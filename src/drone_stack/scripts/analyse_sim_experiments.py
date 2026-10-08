#!/usr/bin/env python3
"""Summarise recorded experiments and generate standalone comparison plots."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
root=Path(args.directory);rows=[]
for directory in sorted({p.parent for p in root.rglob('hold_check*.json')}):
    for report in sorted(directory.glob('hold_check*.json')):
        data=json.loads(report.read_text())
        if 'truth_peak_to_peak' not in data:continue
        row={'experiment':str(directory.relative_to(root)),'report':report.name,
             'duration_wall_s':data['duration_wall_s'], 'height_peak_to_peak_m':data['truth_peak_to_peak'][2],
             'max_lio_truth_displacement_error_m':data['max_lio_truth_displacement_error'],
             'offboard_only':data['modes']==['OFFBOARD'],'hold_only':data['phases']==['HOLD']}
        rows.append(row)
        if 'trace' not in data:continue
        trace=data['trace'];truth=np.array([r['truth'] for r in trace]);lio=np.array([r['lio'] for r in trace]);fcu=np.array([r['local'] for r in trace])
        t=np.arange(len(trace))*.1;error=np.linalg.norm((lio-lio[0])-(truth-truth[0]),axis=1)
        fig,ax=plt.subplots(1,2,figsize=(11,4))
        ax[0].plot(t,truth[:,2],label='Gazebo truth')
        ax[0].plot(t,lio[:,2]+truth[0,2]-lio[0,2],label='LIO (initial origin aligned)')
        ax[0].plot(t,fcu[:,2]+truth[0,2]-fcu[0,2],label='FCU (initial origin aligned)')
        ax[0].set(xlabel='Wall time (s)',ylabel='Z (m)',title='Hover: %.3f m height range'%row['height_peak_to_peak_m']);ax[0].legend()
        ax[1].plot(t,error);ax[1].axhline(.1,color='red',linestyle='--');ax[1].set(xlabel='Wall time (s)',ylabel='Displacement error (m)',title='LIO versus truth');fig.tight_layout();fig.savefig(report.with_suffix('.png'),dpi=150);plt.close(fig)
for report in root.rglob('land*.json'):
    data=json.loads(report.read_text());trace=data.get('trace',[])
    if not trace:continue
    t=np.array([r['t'] for r in trace]);truth=np.array([r['truth'] for r in trace]);fig,ax=plt.subplots(1,2,figsize=(11,4))
    ax[0].plot(t-t[0],truth[:,2]);ax[0].set(xlabel='Simulation time (s)',ylabel='Truth Z (m)',title='Landing height')
    ax[1].plot(truth[:,0],truth[:,1]);ax[1].axis('equal');ax[1].set(xlabel='Truth X (m)',ylabel='Truth Y (m)',title='Landing horizontal motion');fig.tight_layout();fig.savefig(report.with_suffix('.png'),dpi=150);plt.close(fig)
(root/'experiment_comparison.json').write_text(json.dumps(rows,indent=2))
lines=['# 仿真实验对比','', '| 实验 | 悬停墙钟秒数 | 真值高度峰峰值 m | LIO 位移最大误差 m | OFFBOARD/HOLD |', '| --- | --- | --- | --- | --- |']
for r in rows:lines.append('| %s | %.0f | %.4f | %.4f | %s |'%(r['experiment'],r['duration_wall_s'],r['height_peak_to_peak_m'],r['max_lio_truth_displacement_error_m'],'保持' if r['offboard_only'] and r['hold_only'] else '未保持'))
lines+=['','不同悬停时长不能视为相同强度的验收。整轮通过情况以各目录 suite_result.json 为准；单项数值改善不等于全部通过。']
(root/'EXPERIMENT_COMPARISON.md').write_text('\n'.join(lines)+'\n')
if rows:
    fig,ax=plt.subplots(figsize=(10,max(3,len(rows)*.45)))
    labels=[r['experiment']+' (%gs)'%r['duration_wall_s'] for r in rows]
    ax.barh(labels,[r['height_peak_to_peak_m'] for r in rows]);ax.axvline(.15,color='red',linestyle='--',label='0.15 m acceptance');ax.set_xlabel('Truth height peak-to-peak (m)');ax.legend();fig.tight_layout();fig.savefig(root/'height_comparison.png',dpi=150)
print(json.dumps(rows,indent=2))
