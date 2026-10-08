#!/usr/bin/env python3
"""Named map-frame regions, isolated by map content hash, atomically persisted."""
import fcntl
import json
import math
from pathlib import Path
import re
import tempfile
import time
import uuid
from inspection_geometry import validate_polygon


def validate_region(data):
    name=str(data.get('name','')).strip()
    if not 1<=len(name)<=64 or any(ord(c)<32 for c in name):
        raise ValueError('区域名称需为1～64个字符，不得含控制字符')
    polygon,_=validate_polygon(data.get('polygon',[]))
    values={k:float(data[k]) for k in ['altitude','spacing','overlap','speed','angle_deg']}
    if not all(math.isfinite(v) for v in values.values()):raise ValueError('区域参数包含非法数值')
    if not .1<=values['spacing']<=5 or not 0<=values['overlap']<=.8 or not .1<=values['speed']<=1:
        raise ValueError('区域间距、重叠率或速度非法')
    if values['angle_deg']!=-1 and not 0<=values['angle_deg']<180:raise ValueError('区域方向参数非法')
    entry=int(data.get('entry',0))
    if not 0<=entry<=4:raise ValueError('入口参数非法')
    if not isinstance(data.get('auto_spacing',False),bool):raise ValueError('自动间距参数需为布尔值')
    return dict(name=name,polygon=polygon.tolist(),auto_spacing=data.get('auto_spacing',False),entry=entry,**values)


class RegionStore:
    def __init__(self,root,digest,map_path=''):
        if not re.fullmatch('[0-9a-f]{64}',digest):raise ValueError('地图标识无效，请重新加载地图')
        self.root=Path(root);self.digest=digest;self.map_path=str(map_path)
        self.file=self.root/(digest+'.json')

    def read(self):
        if not self.file.exists():return []
        if self.file.stat().st_size>4*1024*1024:raise ValueError('区域库超过4MB，拒绝读取')
        try:data=json.loads(self.file.read_text())
        except (OSError,ValueError) as exc:raise ValueError('区域库损坏或不可读：'+str(exc))
        if data.get('version')!=1 or data.get('map_digest')!=self.digest or not isinstance(data.get('regions'),list):
            raise ValueError('区域库版本或地图不匹配')
        records=[];ids=set();names=set()
        for r in data['regions']:
            clean=validate_region(r)
            if not re.fullmatch('[0-9a-f]{32}',str(r.get('id',''))) or r['id'] in ids or clean['name'] in names:
                raise ValueError('区域库ID或名称重复/非法')
            ids.add(r['id']);names.add(clean['name'])
            records.append(dict(clean,id=r['id'],created_at=float(r['created_at']),updated_at=float(r['updated_at'])))
        return records

    def change(self,action,data):
        self.root.mkdir(parents=True,exist_ok=True)
        with (self.root/(self.digest+'.lock')).open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            records=self.read();ident=str(data.get('id',''))
            if action=='delete':
                if not any(r['id']==ident for r in records):raise ValueError('该区域已不存在，请刷新列表')
                records=[r for r in records if r['id']!=ident]
            elif action in ('save','update'):
                clean=validate_region(data)
                original=next((r for r in records if r['id']==ident),None) if action=='update' else None
                if action=='update' and original is None:raise ValueError('该区域已不存在，请刷新列表')
                if len(records)>=256 and action=='save':raise ValueError('当前地图最多保存256个区域')
                if any(r['name']==clean['name'] and (original is None or r['id']!=ident) for r in records):
                    raise ValueError('名称已存在；更换名称或使用“更新所选区域”')
                now=time.time();clean.update(id=original['id'] if original else uuid.uuid4().hex,
                    created_at=original['created_at'] if original else now,updated_at=now)
                records=[r for r in records if original is None or r['id']!=ident]+[clean]
            else:raise ValueError('未知区域库操作')
            payload=dict(version=1,map_digest=self.digest,map_path=self.map_path,regions=records)
            temporary=None
            try:
                with tempfile.NamedTemporaryFile('w',dir=self.root,prefix=self.digest+'.',suffix='.tmp',delete=False) as f:
                    temporary=Path(f.name);json.dump(payload,f,ensure_ascii=False,indent=2);f.flush()
                    import os
                    os.fsync(f.fileno())
                temporary.replace(self.file)
            finally:
                if temporary is not None:temporary.unlink(missing_ok=True)
            return records
