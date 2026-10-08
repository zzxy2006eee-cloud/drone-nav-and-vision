#!/usr/bin/env python3
"""Independent world geometry checks for simulation acceptance.

Uses every static collision box in the selected SDF, with model/link/collision
poses composed. Rejects unsupported geometry instead of declaring it clear.
The vehicle is enclosed by a conservative body box including its sensors;
this is an envelope check, not a Gazebo contact-sensor measurement.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def pose_transform(element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('Named SDF pose frames require explicit resolution')
    p = [float(v) for v in pose.text.split()] if pose is not None else [0.0]*6
    if len(p) != 6 or not all(math.isfinite(v) for v in p):
        raise ValueError('Expected six finite pose values')
    x, y, z, r, pitch, yaw = p
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3,:3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                  [-sp, cp*sr, cp*cr]]
    out[:3,3] = [x,y,z]
    return out


def quaternion_rotation(q):
    q = np.asarray(q,dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1) > .05:
        raise ValueError('Invalid vehicle quaternion')
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def box_separation(a_center, a_rotation, a_half, b_center, b_rotation, b_half):
    """Positive separating-axis gap or negative overlap (not Euclidean distance)."""
    axes = [*a_rotation.T, *b_rotation.T]
    axes.extend(np.cross(a_rotation[:,i],b_rotation[:,j]) for i in range(3) for j in range(3))
    delta = np.asarray(b_center)-a_center
    gaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-10:
            continue
        axis = axis/norm
        radius = np.dot(a_half,abs(a_rotation.T@axis)) + np.dot(b_half,abs(b_rotation.T@axis))
        gaps.append(float(abs(np.dot(delta,axis))-radius))
    return max(gaps)


def quaternion_rotations(q):
    q = np.asarray(q,dtype=float)
    norm = np.linalg.norm(q,axis=1)
    if q.ndim != 2 or q.shape[1] != 4 or not np.all(np.isfinite(q)) or np.any(abs(norm-1)>.05):
        raise ValueError('Invalid vehicle quaternion batch')
    x,y,z,w = (q/norm[:,None]).T
    r = np.empty((len(q),3,3))
    r[:,0,0]=1-2*(y*y+z*z);r[:,0,1]=2*(x*y-z*w);r[:,0,2]=2*(x*z+y*w)
    r[:,1,0]=2*(x*y+z*w);r[:,1,1]=1-2*(x*x+z*z);r[:,1,2]=2*(y*z-x*w)
    r[:,2,0]=2*(x*z-y*w);r[:,2,1]=2*(y*z+x*w);r[:,2,2]=1-2*(x*x+y*y)
    return r


def box_separations(positions, rotations, vehicle_half, center, rotation, half):
    """Vectorized counterpart of box_separation for complete high-rate bags."""
    count=len(positions)
    axes=[rotations[:,:,i] for i in range(3)]
    axes.extend(np.broadcast_to(rotation[:,j],(count,3)) for j in range(3))
    axes.extend(np.cross(rotations[:,:,i],rotation[:,j]) for i in range(3) for j in range(3))
    delta=center-positions; maximum=np.full(count,-np.inf)
    for axis in axes:
        norm=np.linalg.norm(axis,axis=1);valid=norm>1e-10
        axis=axis/np.maximum(norm,1e-10)[:,None]
        radius=(np.sum(abs(np.einsum('nij,ni->nj',rotations,axis))*vehicle_half,axis=1)+
                np.sum(abs(axis@rotation)*half,axis=1))
        gap=abs(np.sum(delta*axis,axis=1))-radius
        maximum=np.maximum(maximum,np.where(valid,gap,-np.inf))
    return maximum


class WorldGeometry:
    def __init__(self, path, vehicle_half=(.32,.37,.18), margin=.03):
        self.path = Path(path).resolve()
        self.vehicle_half = np.asarray(vehicle_half)+margin
        if np.any(self.vehicle_half <= 0) or not np.all(np.isfinite(self.vehicle_half)):
            raise ValueError('Invalid vehicle envelope')
        self.boxes = []
        world = ET.parse(self.path).getroot().find('world')
        for include in world.findall('include'):
            if include.findtext('uri') not in ('model://sun','model://ground_plane'):
                raise ValueError('Unsupported included model: '+str(include.findtext('uri')))
        for model in world.findall('model'):
            if model.findtext('static') != 'true':
                raise ValueError('Dynamic world models need a time-indexed geometry checker')
            for link in model.findall('link'):
                for collision in link.findall('collision'):
                    box = collision.find('geometry/box')
                    if box is None:
                        raise ValueError('Unsupported collision geometry in '+model.get('name'))
                    half = np.asarray([float(v) for v in box.findtext('size').split()])/2
                    if half.shape != (3,) or not np.all(np.isfinite(half)) or np.any(half<=0):
                        raise ValueError('Invalid collision box size')
                    transform = pose_transform(model)@pose_transform(link)@pose_transform(collision)
                    self.boxes.append((model.get('name')+'/'+link.get('name')+'/'+collision.get('name'),
                                       transform[:3,3], transform[:3,:3], half))
        if not self.boxes:
            raise ValueError('No static collision geometry found')

    def check(self, position, quaternion=(0,0,0,1)):
        position = np.asarray(position,dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('Invalid vehicle position')
        rotation = quaternion_rotation(quaternion)
        gaps = [(name,box_separation(position,rotation,self.vehicle_half,center,orientation,half))
                for name,center,orientation,half in self.boxes]
        name,gap = min(gaps,key=lambda item:item[1])
        return {'envelope_clear':gap>0,'closest_obstacle':name,'separating_axis_gap_m':gap,
                'overlapping_obstacles':[label for label,value in gaps if value<=0]}

    def check_many(self, positions, quaternions):
        positions=np.asarray(positions,dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 3 or not np.all(np.isfinite(positions)):
            raise ValueError('Invalid vehicle position batch')
        rotations=quaternion_rotations(quaternions)
        if len(positions)!=len(rotations):raise ValueError('Mismatched pose counts')
        gaps=np.stack([box_separations(positions,rotations,self.vehicle_half,c,r,h)
                       for _,c,r,h in self.boxes],axis=1)
        closest=gaps.argmin(axis=1)
        return gaps[np.arange(len(gaps)),closest],closest
