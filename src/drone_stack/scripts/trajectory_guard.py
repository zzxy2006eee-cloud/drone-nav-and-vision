#!/usr/bin/env python3
"""Conservative continuous cubic B-spline checks in the planner's ENU map."""
import math
import time
import numpy as np
from scipy.interpolate import PPoly, BSpline
from functools import lru_cache



class PackedCells:
    """Immutable voxel membership without one Python tuple per map point."""
    def __init__(self, codes, shape, bitmap=False):
        self.bitmap=bitmap
        self.count=int(np.count_nonzero(codes)) if bitmap else len(codes)
        self.codes = codes
        self.shape = tuple(int(v) for v in shape)

    def __contains__(self, cell):
        x,y,z = (int(v) for v in cell)
        nx,ny,nz = self.shape
        if not (0 <= x < nx and 0 <= y < ny and 0 <= z < nz):return False
        code=(x*ny+y)*nz+z
        return bool(self.codes[code]) if self.bitmap else code in self.codes

    def __len__(self):
        return self.count

    @classmethod
    def from_cloud(cls, msg, origin, resolution, shape):
        if not msg.width or not msg.height:
            return cls(set(), shape)
        fields = {f.name:f for f in msg.fields}
        if any(k not in fields or fields[k].count != 1 or fields[k].datatype not in (7,8) for k in 'xyz'):
            raise ValueError('Map XYZ fields must be scalar float')
        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({'names':list('xyz'),
                          'formats':[endian+('f4' if fields[k].datatype==7 else 'f8') for k in 'xyz'],
                          'offsets':[fields[k].offset for k in 'xyz'], 'itemsize':msg.point_step})
        a = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                       strides=(msg.row_step,msg.point_step))
        xyz = np.column_stack([a[k].reshape(-1) for k in 'xyz'])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        grid = np.floor((xyz-np.asarray(origin,dtype=float))/resolution)
        grid = grid[np.all((grid >= 0) & (grid < np.asarray(shape)),axis=1)].astype(np.int64)
        codes = (grid[:,0]*shape[1]+grid[:,1])*shape[2]+grid[:,2]
        return cls(set(codes.tolist()),shape)

class MapConversionCache:
    """Reuse voxel encoding for an identical payload; never cache freshness.

    Metadata and bytes must both match. A new acquisition stamp still reaches
    the caller's normal freshness checks, even when geometry is unchanged.
    """
    def __init__(self):
        self.key = None
        self.data = None
        self.cells = None

    def convert(self, msg, origin, resolution, shape):
        key = (msg.header.frame_id, msg.width, msg.height, msg.point_step,
               msg.row_step, msg.is_bigendian, msg.is_dense,
               tuple((f.name, f.offset, f.datatype, f.count) for f in msg.fields),
               tuple(origin), resolution, tuple(shape))
        if self.cells is not None and key == self.key and msg.data == self.data:
            return self.cells
        cells = PackedCells.from_cloud(msg, origin, resolution, shape)
        self.key, self.data, self.cells = key, msg.data, cells
        return cells


@lru_cache(maxsize=24)
def _curve_geometry(order, knots_tuple, points_tuple):
    knots=np.asarray(knots_tuple,dtype=float);points=np.asarray(points_tuple,dtype=float)
    if (order!=3 or points.ndim!=2 or points.shape[1]!=3 or not 4<=len(points)<=256 or
            len(knots)!=len(points)+4 or not np.isfinite(points).all() or not np.isfinite(knots).all() or
            np.any(np.diff(knots)<0) or not 0<knots[len(points)]-knots[3]<=120):
        raise ValueError('Malformed or unsupported EGO B-spline')
    polynomials=tuple(PPoly.from_spline((knots,points[:,axis],3)) for axis in range(3))
    # Installed SciPy evaluates through a writable Cython memoryview. Keep
    # dedicated writable spline buffers; freeze only the shared geometry data.
    spline=BSpline(knots.copy(),points.copy(),order,extrapolate=False)
    knots.setflags(write=False);points.setflags(write=False)
    return knots,points,polynomials,spline


def curve_geometry(message):
    # Cache geometry only. Timestamps, task IDs and map checks are never cached.
    return _curve_geometry(message.order,tuple(message.knots),
                           tuple((p.x,p.y,p.z) for p in message.pos_pts))


def validate_curve(message, occupied, origin, resolution, lower, upper,
                   wall_budget=2.0, node_budget=10000, cpu_budget=.05, stats=None, observed_free=None, curve_window=None):
    begin = time.monotonic()
    cpu_begin = time.thread_time()
    nodes = 0
    try:
        knots,points,polynomials,_=curve_geometry(message)
        if not math.isfinite(resolution) or resolution<=0:
            return 'Invalid map resolution'
        start, end = knots[3], knots[len(points)]
        if not 0 < end-start <= 120:
            return 'Invalid EGO B-spline duration'
        window_start, window_end = start, end
        if curve_window is not None:
            if len(curve_window)!=2 or not all(math.isfinite(v) for v in curve_window) or not 0 <= curve_window[0] < curve_window[1]:
                return 'Invalid EGO curve validation window'
            window_start=max(start,start+curve_window[0]);window_end=min(end,start+curve_window[1])
        origin = np.asarray(origin)
        lower, upper = np.asarray(lower), np.asarray(upper)
        nodes = 0
        for index in range(3, len(points)):
            left=max(knots[index],window_start);right=min(knots[index+1],window_end)
            span = right-left
            if span <= 0:
                continue
            c = np.stack([poly.c[:,index] for poly in polynomials], axis=1)
            offset=left-knots[index]
            c=np.array([c[0],c[1]+3*c[0]*offset,
                        c[2]+2*c[1]*offset+3*c[0]*offset**2,
                        c[3]+c[2]*offset+c[1]*offset**2+c[0]*offset**3])
            controls = np.array([c[3], c[3]+c[2]*span/3,
                                 c[3]+2*c[2]*span/3+c[1]*span*span/3,
                                 c[3]+c[2]*span+c[1]*span*span+c[0]*span**3])
            stack = [(controls, 0)]
            while stack:
                hull, depth = stack.pop()
                nodes += 1
                if (nodes > node_budget or time.monotonic()-begin > wall_budget or
                        time.thread_time()-cpu_begin > cpu_budget):
                    return 'Full EGO trajectory validation exceeded its bounded budget'
                lo, hi = hull.min(axis=0), hull.max(axis=0)
                outside = bool(np.any(lo < lower-1e-9) or np.any(hi > upper+1e-9))
                if np.any(hi < lower-1e-9) or np.any(lo > upper+1e-9):
                    return 'Full EGO trajectory is outside configured flight volume'
                # Include both cells at an exact voxel face. A curve lies in
                # its Bezier control hull; subdivision only shrinks that hull.
                hit = False
                unknown = False
                if not outside:
                    first = np.floor((lo-origin)/resolution-1e-8).astype(int)
                    last = np.floor((hi-origin)/resolution+1e-8).astype(int)
                    cell_count = math.prod(int(v) for v in last-first+1)
                    if cell_count > 100000:
                        return 'Full EGO trajectory spans an excessive voxel range'
                    hit = any((x,y,z) in occupied
                              for x in range(first[0],last[0]+1)
                              for y in range(first[1],last[1]+1)
                              for z in range(first[2],last[2]+1))
                    if observed_free is not None:
                        unknown = any((x,y,z) not in observed_free
                                      for x in range(first[0],last[0]+1)
                                      for y in range(first[1],last[1]+1)
                                      for z in range(first[2],last[2]+1))
                if not outside and not hit and not unknown:
                    continue
                if depth >= 18 or np.linalg.norm(hi-lo) < resolution/1024:
                    return ('Full EGO trajectory is outside configured flight volume' if outside
                            else 'Full EGO trajectory intersects an inflated obstacle voxel' if hit
                            else 'Full EGO trajectory enters unobserved space')
                a = (hull[:-1]+hull[1:])/2
                b = (a[:-1]+a[1:])/2
                middle = (b[0]+b[1])/2
                stack.append((np.array([middle,b[1],a[2],hull[3]]),depth+1))
                stack.append((np.array([hull[0],a[0],b[0],middle]),depth+1))
        return ''
    except (ValueError, TypeError, IndexError, OverflowError):
        return 'Malformed EGO B-spline'
    finally:
        if stats is not None:
            stats.update(wall_s=time.monotonic()-begin,
                         cpu_s=time.thread_time()-cpu_begin, nodes=nodes)
