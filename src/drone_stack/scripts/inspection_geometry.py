#!/usr/bin/env python3
"""Planar region geometry and camera footprints; no flight or ROS side effects."""
import math
import numpy as np


def inside(points, polygon):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    poly=np.asarray(polygon,dtype=float)
    result=np.zeros(len(points),dtype=bool)
    x,y=points.T
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        if abs(b[1]-a[1])<1e-12:continue
        crossing=((a[1]>y)!=(b[1]>y)) & (x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
        result ^= crossing
    return result


def validate_polygon(polygon):
    p=np.asarray(polygon,dtype=float)
    if p.shape[0]>3 and np.linalg.norm(p[0]-p[-1])<1e-8:p=p[:-1]
    if p.ndim!=2 or p.shape[1]!=2 or not 3<=len(p)<=64 or not np.isfinite(p).all():
        raise ValueError('区域需为3～64个有限坐标顶点')
    if any(np.linalg.norm(a-b)<.05 for a,b in zip(p,np.roll(p,-1,axis=0))):
        raise ValueError('区域顶点重复或边长小于5cm')
    def cross(a,b,c):return float(np.cross(b-a,c-a))
    edges=list(zip(p,np.roll(p,-1,axis=0)))
    for i,(a,b) in enumerate(edges):
        for j,(c,d) in enumerate(edges):
            if j<=i or j==i+1 or (i==0 and j==len(p)-1):continue
            if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0 and (
                    np.maximum(np.minimum(a,b),np.minimum(c,d))<=np.minimum(np.maximum(a,b),np.maximum(c,d))+1e-9).all():
                raise ValueError('区域存在自交，请重新圈选')
    area=abs(float(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))))/2
    if area<.10:raise ValueError('区域面积小于0.1平方米')
    if np.any(np.abs(p)>50):raise ValueError('区域超出地图允许范围')
    return p,area


def strips(polygon, spacing, angle):
    """Clip horizontal parallel lanes against a possibly concave polygon."""
    c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[c,-s],[s,c]])
    p=np.asarray(polygon)@rotation
    low,high=p[:,1].min(),p[:,1].max()
    count=max(1,int(math.ceil((high-low)/spacing)))
    if count>256:raise ValueError('条带过多，请增加间距或缩小区域')
    result=[]
    first=low+((high-low)-(count-1)*spacing)/2.
    for row,y in enumerate(first+np.arange(count)*spacing):
        hits=[]
        for a,b in zip(p,np.roll(p,-1,axis=0)):
            if (a[1]<=y<b[1]) or (b[1]<=y<a[1]):
                hits.append(a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
        hits.sort();pairs=list(zip(hits[::2],hits[1::2]))
        if row%2:pairs.reverse()
        for x0,x1 in pairs:
            if x1-x0<.10:continue
            a,b=np.array([x0,y])@rotation.T,np.array([x1,y])@rotation.T
            result.append((b,a) if row%2 else (a,b))
    return result


def orient_strips(lines, choice):
    if choice in (2,3):lines=list(reversed(lines))
    return [(b,a) for a,b in lines] if choice in (1,3) else list(lines)


def raster(polygon, resolution):
    p=np.asarray(polygon);low=p.min(axis=0)
    shape=np.maximum(1,np.ceil((p.max(axis=0)-low)/resolution).astype(int))
    if int(np.prod(shape))>200000:raise ValueError('覆盖网格过大，请缩小区域')
    x,y=np.meshgrid(low[0]+(np.arange(shape[0])+.5)*resolution,
                    low[1]+(np.arange(shape[1])+.5)*resolution,indexing='ij')
    xy=np.column_stack([x.ravel(),y.ravel()])
    return xy,inside(xy,p),low,shape


def footprint(camera_position, camera_rotation, intrinsics, surface_z):
    """Intersect optical image-corner rays with the configured horizontal plane."""
    fx,fy,cx,cy,width,height=intrinsics
    if min(fx,fy,width,height)<=0:return None
    pixels=np.array([[0,0],[width,0],[width,height],[0,height]],dtype=float)
    rays=np.column_stack([(pixels[:,0]-cx)/fx,(pixels[:,1]-cy)/fy,np.ones(4)])@np.asarray(camera_rotation).T
    if np.any(rays[:,2]>=-1e-5) or camera_position[2]<=surface_z:return None
    distances=(surface_z-camera_position[2])/rays[:,2]
    return (np.asarray(camera_position)+rays*distances[:,None])[:,:2]


def automatic_spacing(body_height, camera, surface_z, overlap):
    h=body_height+float(camera['xyz_body'][2])-surface_z
    if h<=.05:raise ValueError('相机距配置地面不足5cm，请核对地面Z与巡检高度')
    # Nominal level attitude, heading +X. Include the configured optical mount.
    rotation=np.asarray(camera['optical_rotation_body'],dtype=float).reshape(3,3)
    intrinsics=[float(camera[k]) for k in ('fx','fy','cx','cy','width','height')]
    polygon=footprint(np.array([0.,0.,body_height])+np.asarray(camera['xyz_body']),rotation,intrinsics,surface_z)
    if polygon is None:raise ValueError('相机视场无法完整投影到配置地面，请核对安装角和高度')
    width=float(np.ptp(polygon[:,1]))
    return width*(1.-overlap),width


def line_points(a,b,spacing=.08):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    return [tuple(p) for p in np.linspace(a,b,max(2,int(math.ceil(np.linalg.norm(b-a)/spacing))+1))]


def split_free(a,b,height,safe):
    pts=line_points((*a,height),(*b,height),.05)
    result=[];begin=None
    for i,p in enumerate(pts):
        usable=safe(p,p) and (i==0 or safe(pts[i-1],p))
        if usable and begin is None:begin=i
        if begin is not None and (not usable or i==len(pts)-1):
            end=i if usable else i-1
            if end>begin and math.dist(pts[begin],pts[end])>=.15:result.append((pts[begin],pts[end]))
            begin=None
    return result


def forward_projection(position, route, progress, advance):
    p=np.asarray(route);length=np.linalg.norm(np.diff(p,axis=0),axis=1);arc=np.r_[0.,np.cumsum(length)]
    best=None
    first=max(0,min(len(length)-1,int(np.searchsorted(arc,progress,side='right'))-1))
    last=min(len(length),int(np.searchsorted(arc,progress+advance,side='right'))+1)
    for i in range(first,last):
        d=length[i]
        if d<1e-8 or arc[i+1]<progress or arc[i]>progress+advance:continue
        u=float(np.clip(np.dot(np.asarray(position)-p[i],p[i+1]-p[i])/d**2,
                        max(0.,(progress-arc[i])/d),min(1.,(progress+advance-arc[i])/d)))
        point=p[i]+u*(p[i+1]-p[i]);gap=float(np.linalg.norm(point-position));along=arc[i]+u*d
        if best is None or gap<best[0]-1e-8 or (abs(gap-best[0])<1e-8 and along>best[1]):best=(gap,along,i,point)
    return best,arc


def route_window(route, projection, arc, horizon):
    _,along,i,point=projection
    result=[tuple(point)];end=min(arc[-1],along+horizon)
    for j in range(i+1,len(route)):
        if arc[j]<=end+1e-9:result.append(tuple(route[j]));continue
        previous=np.asarray(result[-1]);distance=end-max(along,arc[j-1])
        direction=np.asarray(route[j])-previous
        result.append(tuple(previous+direction*distance/max(1e-9,np.linalg.norm(direction))));break
    clean=[]
    for p in result:
        if not clean or math.dist(p,clean[-1])>1e-5:clean.append(p)
    if len(clean)>1800:return clean[:1800],True
    return clean,end<arc[-1]-1e-5
