"""Numerical acquisition-time TF interpolation/composition for offline captures."""
from bisect import bisect_left
from collections import defaultdict
import math
import numpy as np


def quaternion_matrix(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(float(q@q)-1)>0.01:
        raise ValueError('Invalid transform quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def slerp(a,b,fraction):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    a,b=a/np.linalg.norm(a),b/np.linalg.norm(b)
    dot=float(a@b)
    if dot<0:b=-b;dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995:
        value=a+(b-a)*fraction
        return value/np.linalg.norm(value)
    theta=math.acos(dot)
    return (math.sin((1-fraction)*theta)*a+math.sin(fraction*theta)*b)/math.sin(theta)


class CaptureTf:
    def __init__(self,max_gap_seconds=.1):
        self.max_gap=int(max_gap_seconds*1e9)
        self.static={};self.dynamic=defaultdict(dict);self.parents={};self._ordered={}

    def add(self,value,*,static=False):
        parent,child=value['parent'],value['child']
        if not parent or not child or parent==child or (child in self.parents and self.parents[child]!=parent):
            raise ValueError('TF has invalid/conflicting frame parents')
        self.parents[child]=parent
        ancestor=parent;seen={child}
        while ancestor in self.parents:
            if ancestor in seen:raise ValueError('TF frame cycle')
            seen.add(ancestor);ancestor=self.parents[ancestor]
        q=np.array([value['rotation'][key] for key in ('x','y','z','w')],dtype=float)
        position=np.array([value['translation'][key] for key in ('x','y','z')],dtype=float)
        quaternion_matrix(q)
        q=q/np.linalg.norm(q)
        if not np.isfinite(position).all():raise ValueError('Non-finite TF translation')
        edge=(parent,child)
        if (static and edge in self.dynamic) or (not static and edge in self.static):
            raise ValueError('TF edge has both static and dynamic authorities')
        target=self.static if static else self.dynamic[edge]
        key=edge if static else int(value['stamp_ns'])
        if not static and key<=0:raise ValueError('Invalid dynamic TF stamp')
        if key in target:
            old_position,old_q=target[key]
            if not np.allclose(old_position,position,atol=1e-9) or abs(float(old_q@q))<.999999:
                raise ValueError('Conflicting transforms at the same stamp')
        target[key]=(position,q)
        self._ordered.pop(edge,None)

    def matrix(self,edge,stamp):
        if edge in self.static:
            position,q=self.static[edge]
        else:
            values=self.dynamic[edge]
            if edge not in self._ordered:self._ordered[edge]=sorted(values)
            stamps=self._ordered[edge]
            index=bisect_left(stamps,stamp)
            if index<len(stamps) and stamps[index]==stamp:
                position,q=values[stamp]
            elif 0<index<len(stamps) and stamps[index]-stamps[index-1]<=self.max_gap:
                a,b=stamps[index-1],stamps[index]
                f=(stamp-a)/(b-a)
                pa,qa=values[a];pb,qb=values[b]
                position=pa+(pb-pa)*f;q=slerp(qa,qb,f)
            else:raise ValueError('TF would require extrapolation or interpolation across a gap')
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q);matrix[:3,3]=position
        return matrix

    def lookup(self,target,source,stamp):
        if not target or not source:raise ValueError('Missing TF frame')
        queue=[(source,np.eye(4))];seen={source}
        for frame,accumulated in queue:
            if frame==target:return accumulated
            for edge in set(self.static).union(self.dynamic):
                parent,child=edge
                if frame not in edge:continue
                neighbor=parent if frame==child else child
                if neighbor in seen:continue
                try:matrix=self.matrix(edge,stamp)
                except ValueError:continue
                if frame==parent:matrix=np.linalg.inv(matrix)
                seen.add(neighbor);queue.append((neighbor,matrix@accumulated))
        raise ValueError('Complete acquisition-time TF chain unavailable')
