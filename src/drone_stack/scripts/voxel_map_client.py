#!/usr/bin/env python3
"""Versioned voxel deltas, immutable bitmap snapshots, fail-closed resync."""
import math
import json
import time
import threading
import numpy as np
import rospy
from std_msgs.msg import String
from plan_env.msg import VoxelUpdate
from plan_env.srv import GetVoxelSnapshot
from trajectory_guard import PackedCells


class VoxelMapClient:
    def __init__(self, origin, resolution, shape, update, invalidate):
        self.origin=tuple(origin);self.resolution=resolution;self.shape=tuple(shape)
        self.update=update;self.invalidate=invalidate;self.lock=threading.RLock()
        self.status_pub=rospy.Publisher('~voxel_stream_status',String,queue_size=1,latch=True)
        self.status_last=0.;self.resyncs=0
        self.epoch=0;self.revision=0;self.stamp=rospy.Time(0);self.occupied=None;self.free=None
        self.snapshot=rospy.ServiceProxy('/grid_map/get_voxel_snapshot',GetVoxelSnapshot)
        self.subscriber=rospy.Subscriber('/grid_map/voxel_delta',VoxelUpdate,self.receive,
                                        queue_size=100,buff_size=8*1024*1024,tcp_nodelay=True)

    def apply(self,m):
        if (m.header.frame_id!='odom' or m.header.stamp==rospy.Time(0) or not m.epoch or not m.revision or
                tuple(m.shape)!=self.shape or not math.isfinite(m.resolution) or abs(m.resolution-self.resolution)>1e-9 or
                not np.allclose([m.origin.x,m.origin.y,m.origin.z],self.origin,rtol=0,atol=1e-8)):
            raise ValueError('Voxel update metadata mismatch')
        if m.epoch<self.epoch or (m.epoch==self.epoch and m.revision<=self.revision):return False
        if m.header.stamp<self.stamp:raise ValueError('Voxel observation time regressed')
        if not m.full and (m.epoch!=self.epoch or m.base_revision!=self.revision or m.revision!=m.base_revision+1):
            raise ValueError('Voxel revision gap')
        size=math.prod(self.shape)
        arrays=[np.asarray(v,dtype=np.int64) for v in [m.occupied_added,m.occupied_removed,m.free_added,m.free_removed]]
        if any(len(a)>size or (len(a) and (a.min()<0 or a.max()>=size)) for a in arrays):raise ValueError('Invalid voxel IDs')
        if m.full and (m.base_revision or len(arrays[1]) or len(arrays[3])):raise ValueError('Malformed full voxel snapshot')
        if m.full:
            occupied=np.zeros(size,dtype=np.bool_);free=np.zeros(size,dtype=np.bool_)
        else:
            occupied=self.occupied.codes.copy() if len(arrays[0])+len(arrays[1]) else self.occupied.codes
            free=self.free.codes.copy() if len(arrays[2])+len(arrays[3]) else self.free.codes
        if len(arrays[1]):occupied[arrays[1]]=False
        if len(arrays[3]):free[arrays[3]]=False
        if len(arrays[0]):occupied[arrays[0]]=True
        if len(arrays[2]):free[arrays[2]]=True
        if np.any(occupied & free):raise ValueError('Occupied/free voxel overlap')
        occupied.setflags(write=False);free.setflags(write=False)
        if self.occupied is None or occupied is not self.occupied.codes:self.occupied=PackedCells(occupied,self.shape,bitmap=True)
        if self.free is None or free is not self.free.codes:self.free=PackedCells(free,self.shape,bitmap=True)
        self.epoch=m.epoch;self.revision=m.revision;self.stamp=m.header.stamp
        return True

    def publish_status(self,valid,force=False,reason=''):
        now=time.monotonic()
        if not force and now-self.status_last<1.:return
        self.status_last=now
        self.status_pub.publish(String(data=json.dumps({'valid':valid,'epoch':self.epoch,'revision':self.revision,
            'stamp':self.stamp.to_sec(),'occupied':len(self.occupied) if self.occupied else 0,
            'free':len(self.free) if self.free else 0,'resyncs':self.resyncs,'reason':reason})))

    def receive(self,m):
        with self.lock:
            try:
                changed=self.apply(m)
            except (ValueError,TypeError,IndexError) as exc:
                self.invalidate();self.resyncs+=1;self.publish_status(False,True,str(exc))
                rospy.logwarn_throttle(2.,'Voxel stream resync required: %s',exc)
                try:
                    reply=self.snapshot()
                    if not reply.success:return
                    self.apply(reply.update)
                    changed=True
                except (rospy.ServiceException,ValueError,TypeError,IndexError):return
            if changed:
                self.update(self.occupied,self.free,self.stamp,self.epoch,self.revision)
                self.publish_status(True,force=m.full)
