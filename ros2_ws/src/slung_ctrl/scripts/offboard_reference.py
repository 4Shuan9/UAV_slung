#!/usr/bin/env python3
# Codex-generated file: reference geometry for the interactive Offboard task layer.
# Existing observer/control mathematics and parameter files are not changed.
import math


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def valid_point(p):
    return len(p) == 3 and all(math.isfinite(x) for x in p)


def minimum_height(cable, radius, suspension_offset, clearance):
    # Conservative: ground-start PX4 body origin is treated as ground height.
    # The offset is a bound on suspension position relative to that origin.
    return cable + radius + abs(suspension_offset) + clearance


def admissible_height(z, required):
    return math.isfinite(z) and z >= required


def at_home(p, velocity, height, position_tolerance, speed_tolerance):
    return norm((p[0], p[1], p[2]-height)) <= position_tolerance and norm(velocity) <= speed_tolerance


class Reference:
    """Position-only references consumed by the unchanged baseline controllers.

    Public command coordinates: North, East, Up, relative to ground-start home.
    The existing figure-eight geometry and steady frequency are retained.
    """
    def __init__(self, kind, now, start, goal=None, speed=1.0, accel=.5,
                 scale=8.0, omega=.150, loops=0, side=9.0, ramp=2.0):
        self.kind=kind; self.t0=now; self.start=tuple(start)
        self.goal=tuple(goal or start); self.speed=speed; self.accel=accel
        self.scale=scale; self.omega=omega; self.loops=loops; self.side=side; self.ramp=ramp
        self.elapsed=0.0
        distance=norm(tuple(b-a for a,b in zip(self.start,self.goal)))
        self.duration=max(.02,1.875*distance/speed,math.sqrt(5.774*distance/accel))

    def sample(self, now):
        self.elapsed=max(0.,now-self.t0); t=self.elapsed
        if self.kind in ('setpoint','home','rejoin'):
            u=min(1.,t/self.duration)
            s=10*u**3-15*u**4+6*u**5
            ds=(30*u**2-60*u**3+30*u**4)/self.duration if u<1 else 0.
            d=tuple(b-a for a,b in zip(self.start,self.goal))
            return tuple(a+s*x for a,x in zip(self.start,d)),tuple(ds*x for x in d),u>=1
        if self.kind=='eight':
            effective=t
            rate=1.
            if self.ramp>0:
                if t<self.ramp:
                    u=t/self.ramp;effective=self.ramp*(u**3-.5*u**4);rate=3*u*u-2*u**3
                else:effective=t-.5*self.ramp
            phase=self.omega*effective
            end=2*math.pi*self.loops
            if self.loops>0 and phase>=end:return self.start,(0.,0.,0.),True
            a=self.scale;w=self.omega*rate
            return (self.start[0]+a*math.sin(phase),self.start[1]+a*.5*math.sin(2*phase),self.start[2]),(a*w*math.cos(phase),a*w*math.cos(2*phase),0.),False
        if self.kind=='square':
            # Original node traces a rhombus with the configured side length.
            d=self.side*0.70710678
            offsets=((0.,0.),(d,d),(2*d,0.),(d,-d),(0.,0.))
            distance=t*self.speed
            segment=int(distance/self.side)
            if segment>=4:return self.start,(0.,0.,0.),True
            u=(distance-segment*self.side)/self.side
            a,b=offsets[segment],offsets[segment+1]
            xy=tuple(self.start[i]+a[i]+u*(b[i]-a[i]) for i in range(2))
            v=tuple((b[i]-a[i])*self.speed/self.side for i in range(2))
            return (*xy,self.start[2]),(*v,0.),False
        raise ValueError('Unknown reference kind')

    def resume(self, now):
        self.t0=now-self.elapsed
