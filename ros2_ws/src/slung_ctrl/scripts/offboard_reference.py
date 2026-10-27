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

# Codex-generated mission-only C2 reference bridges. No controller/observer gains.
import copy
import numpy as np


def reference_kinematics(motion, now):
    """Position, velocity and acceleration in home-relative NEU."""
    if isinstance(motion,SmoothReference):return motion.kinematics(now)
    if getattr(motion,'angle_motion',False):
        motion.sample(now)
        return (motion.start[0]+motion.state[0],motion.start[1],motion.start[2]),(motion.state[1],0.,0.),(motion.physical_acceleration,0.,0.)
    p,v,_=motion.sample(now)
    step=.01
    before=copy.copy(motion);after=copy.copy(motion)
    _,vb,_=before.sample(now-step);_,va,_=after.sample(now+step)
    return tuple(p),tuple(v),tuple((a-b)/(2*step) for a,b in zip(va,vb))


class SmoothReference:
    """Quintic P/V/A bridge; bounds derived from existing mission/control settings."""
    def __init__(self,kind,now,start,end,duration,fixed_yaw=False,after=None,next_motion=None):
        self.kind=kind;self.t0=now;self.duration=duration;self.elapsed=0.;self.fixed_yaw=fixed_yaw
        self.after=after;self.next_motion=next_motion
        p0,v0,a0=map(lambda x:np.asarray(x,float),start);p1,v1,a1=map(lambda x:np.asarray(x,float),end)
        T=duration;c0=p0;c1=T*v0;c2=.5*T*T*a0;d=p1-c0-c1-c2;e=T*v1-c1-2*c2;f=T*T*a1-2*c2
        self.c=np.array([c0,c1,c2,10*d-4*e+.5*f,-15*d+7*e-f,6*d-3*e+.5*f])
        self.goal=tuple(p1)
    def values(self,elapsed):
        u=np.clip(np.asarray(elapsed)/self.duration,0,1);c=self.c;u=np.expand_dims(u,-1);T=self.duration
        p=c[0]+u*(c[1]+u*(c[2]+u*(c[3]+u*(c[4]+u*c[5]))))
        v=(c[1]+u*(2*c[2]+u*(3*c[3]+u*(4*c[4]+u*5*c[5]))))/T
        a=(2*c[2]+u*(6*c[3]+u*(12*c[4]+u*20*c[5])))/(T*T)
        j=(6*c[3]+u*(24*c[4]+u*60*c[5]))/(T**3)
        return p,v,a,j
    def kinematics(self,now):return tuple(tuple(x) for x in self.values(max(0.,now-self.t0))[:3])
    def sample(self,now):
        self.elapsed=max(0.,now-self.t0);p,v,a=self.values(self.elapsed)[:3]
        return tuple(p),(0.,0.,0.) if self.fixed_yaw else tuple(v),self.elapsed>=self.duration
    def resume(self,now):self.t0=now-self.elapsed


def smoother(u):
    u=min(1.,max(0.,u))
    return 35*u**4-84*u**5+70*u**6-20*u**7,140*u**3-420*u**4+420*u**5-140*u**6,420*u*u-1680*u**3+2100*u**4-840*u**5,7*u**5-14*u**6+10*u**7-2.5*u**8


class AngleStopReference(SmoothReference):
    """Model-based angle release, followed by a measured-motion stop."""
    angle_motion=True
    def __init__(self,*args,source,**kw):
        start,end=args[2:4]
        super().__init__(*args,fixed_yaw=True,**kw)
        self.source=source;self.source_elapsed=source.elapsed;self.source_initial_state=tuple(source.state)
        self.meta=source.meta;self.start=source.start;self.repeat_period=source.repeat_period
        self.rows=source.rows if hasattr(source,'rows') else source.source.rows
        self.state=(0.,)*6;self.physical_acceleration=0.
        # The free translation must obey the angle dynamics as well. A smooth
        # counter-lobe balances the required velocity change; its amplitude is
        # solved, not a new controller gain or an observer working-domain gate.
        self.balance=0.;self.grid=np.linspace(0,self.duration,201)
        base=np.array([self.raw_angular_state(t)[:3] for t in self.grid])
        u=self.grid/self.duration
        self.basis=np.array([64*u**3*(1-u)**3,
            64*(3*u*u-12*u**3+15*u**4-6*u**5)/self.duration,
            64*(6*u-36*u*u+60*u**3-30*u**4)/(self.duration**2)]).T
        limit=max(abs(r[4]) for r in source.rows) if hasattr(source,'rows') else max(abs(x[0]) for x in base)
        limit=max(limit,abs(base[0,0]),abs(base[-1,0]))
        mask=self.basis[:,0]>1e-8
        lo=float(np.max((-limit-base[mask,0])/self.basis[mask,0]));hi=float(np.min((limit-base[mask,0])/self.basis[mask,0]))
        def acceleration(c):
            z=base+c*self.basis
            return -(float(self.meta['l'])*z[:,2]+float(self.meta['g'])*np.sin(z[:,0]))/np.cos(z[:,0])
        desired=float(end[1][0]-start[1][0])
        def residual(c):return float(np.trapz(acceleration(c),self.grid))-desired
        fl,fh=residual(lo),residual(hi)
        if lo>hi or fl*fh>0:raise ValueError('No velocity-balanced angle bridge at this duration')
        for _ in range(36):
            mid=(lo+hi)/2;fm=residual(mid)
            if fl*fm<=0:hi=mid;fh=fm
            else:lo=mid;fl=fm
        self.balance=(lo+hi)/2;self.accel_grid=acceleration(self.balance)
        dt=np.diff(self.grid)
        self.velocity_grid=np.r_[float(start[1][0]),float(start[1][0])+np.cumsum((self.accel_grid[:-1]+self.accel_grid[1:])*dt/2)]
        self.position_grid=np.r_[float(start[0][0]),float(start[0][0])+np.cumsum(self.velocity_grid[:-1]*dt+(2*self.accel_grid[:-1]+self.accel_grid[1:])*dt*dt/6)]
        self.jerk_grid=np.gradient(self.accel_grid,self.grid,edge_order=2)
        self.goal=(float(self.position_grid[-1]),self.goal[1],self.goal[2])
    def raw_angular_state(self,elapsed):
        if not self.repeat_period:
            T=self.duration;u=min(1.,max(0.,elapsed/T))
            _,_,ff0,t,w,a=self.source_initial_state
            p0,v0,a0,p1,v1,a1=t,w,a,0.,0.,0.
            c0=p0;c1=T*v0;c2=.5*T*T*a0;d=p1-c0-c1-c2;e=T*v1-c1-2*c2;f=T*T*a1-2*c2
            c3=10*d-4*e+.5*f;c4=-15*d+7*e-f;c5=6*d-3*e+.5*f
            th=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))))
            om=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/T
            al=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(T*T)
            A,_,D,_,K=self.meta['input_model_A_beta_D_tau_K'];g=self.meta['g']
            def nominal(t,w,a):return -(A*a+D*w+g*math.sin(t))/(K*math.cos(t))
            h=1-smoother(u)[0]
            return th,om,al,nominal(th,om,al)+h*(ff0-nominal(t,w,a)),h
        T=self.duration;u=min(1.,max(0.,elapsed/T));S,d,dd,I=smoother(u)
        h=1-S;hd=-d/T;hdd=-dd/(T*T);s=self.source_elapsed+T*(u-I)
        if s<0:theta=omega=alpha=feedforward=0.
        else:
            self.source.sample(self.source.t0+s)
            _,_,feedforward,theta,omega,alpha=self.source.state
        th=h*theta;om=hd*theta+h*h*omega;al=hdd*theta+3*h*hd*omega+h**3*alpha
        A,_,D,_,K=self.meta['input_model_A_beta_D_tau_K'];g=self.meta['g']
        def nominal(t,w,a):return -(A*a+D*w+g*math.sin(t))/(K*math.cos(t))
        ff=nominal(th,om,al)+h*(feedforward-nominal(theta,omega,alpha))
        return th,om,al,ff,h
    def angular_state(self,elapsed):
        th,om,al,ff,h=self.raw_angular_state(elapsed)
        u=min(1.,max(0.,elapsed/self.duration));T=self.duration
        b=64*u**3*(1-u)**3;bd=64*(3*u*u-12*u**3+15*u**4-6*u**5)/T
        bdd=64*(6*u-36*u*u+60*u**3-30*u**4)/(T*T)
        A,_,D,_,K=self.meta['input_model_A_beta_D_tau_K'];g=self.meta['g']
        def nominal(t,w,a):return -(A*a+D*w+g*math.sin(t))/(K*math.cos(t))
        t,w,a=th+self.balance*b,om+self.balance*bd,al+self.balance*bdd
        return t,w,a,ff+nominal(t,w,a)-nominal(th,om,al),h
    def values(self,elapsed):
        vals=list(super().values(elapsed));t=np.clip(np.asarray(elapsed),0,self.duration)
        i=np.clip(np.searchsorted(self.grid,t,side='right')-1,0,len(self.grid)-2)
        dt=self.grid[i+1]-self.grid[i];h=t-self.grid[i]
        a0=self.accel_grid[i];j=(self.accel_grid[i+1]-a0)/dt
        p=self.position_grid[i]+self.velocity_grid[i]*h+a0*h*h/2+j*h**3/6
        v=self.velocity_grid[i]+a0*h+j*h*h/2
        a=a0+j*h
        for result,value in zip(vals,(p,v,a,j)):result[...,0]=value
        return tuple(vals)
    def sample(self,now):
        p,v,done=super().sample(now);pv,vel,acc=self.kinematics(now)
        th,om,al,ff,h=self.angular_state(self.elapsed);self.state=(pv[0]-self.start[0],vel[0],ff,th,om,al)
        self.physical_acceleration=acc[0]
        return p,v,done


def plan_bridge(kind,now,start,end_factory,*,speed,accel,ramp,fixed_yaw=False,source=None,after=None,next_motion=None):
    """Search transition duration; never alter the frozen control settings."""
    source=copy.deepcopy(source) if source is not None else None
    frozen_elapsed=source.elapsed if source is not None else None
    minimum=ramp*(2 if source is not None else 1);jerk=accel/ramp
    for k in range(70):
        if source is not None:source.elapsed=frozen_elapsed
        T=minimum*1.075**k;end=end_factory(T)
        kwargs=dict(after=after,next_motion=next_motion)
        try:
            motion=(AngleStopReference(kind,now,start,end,T,source=source,**kwargs) if source is not None else SmoothReference(kind,now,start,end,T,fixed_yaw=fixed_yaw,**kwargs))
        except ValueError:
            if source is not None:continue
            raise
        _,v,a,j=motion.values(np.linspace(0,T,101))
        vmax=max(speed,norm(start[1]),norm(end[1]))+max(norm(start[2]),norm(end[2]))**2/(2*jerk)
        if source is not None:
            rows=np.asarray(source.rows,float) if hasattr(source,'rows') else None
            if rows is not None:
                vmax+=math.sqrt(2*source.meta['g']*source.meta['l']*(1-math.cos(max(abs(rows[:,4])))))
                allowed_jerk=max(jerk,float(np.max(np.abs(np.gradient(rows[:,3],rows[:,0])))),accel*math.sqrt(source.meta['g']/source.meta['l']))
            else:allowed_jerk=jerk
        else:allowed_jerk=jerk
        if np.max(np.linalg.norm(v,axis=1))>vmax+1e-6 or np.max(np.linalg.norm(a,axis=1))>accel+1e-6 or np.max(np.linalg.norm(j,axis=1))>allowed_jerk+1e-6:continue
        if source is not None and max(abs(motion.angular_state(t)[3]) for t in np.linspace(0,T,41))>accel:continue
        motion.elapsed=0.
        return motion
    raise ValueError('No smooth reference bridge meets the existing acceleration/jerk bounds; hold and retry from a settled state')


# Codex-generated consolidation: existing offline angle references and reference adaptation.
import bisect
import csv
import json
from pathlib import Path

def hermite(p0,v0,a0,p1,v1,a1,h,u):
    c0=p0;c1=h*v0;c2=.5*h*h*a0
    d=p1-c0-c1-c2;e=h*v1-c1-2*c2;f=h*h*a1-2*c2
    c3=10*d-4*e+.5*f;c4=-15*d+7*e-f;c5=6*d-3*e+.5*f
    p=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))))
    v=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/h
    a=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(h*h)
    return p,v,a

class AngleReference:
    angle_motion=True
    def __init__(self,path,now,start,kind):
        lines=Path(path).read_text().splitlines()
        self.meta=json.loads(next(x[2:] for x in lines if x.startswith('# {')))
        self.rows=[tuple(float(v) for v in row) for row in list(csv.reader(x for x in lines if not x.startswith('#')))[1:]]
        self.times=[r[0] for r in self.rows]
        if len(self.rows)<2 or any(len(r)!=8 or not all(math.isfinite(x) for x in r) for r in self.rows):raise ValueError('Invalid offline angle trajectory')
        if self.times[0]!=0 or any(b<=a for a,b in zip(self.times,self.times[1:])):raise ValueError('Invalid trajectory time grid')
        self.kind=kind;self.t0=now;self.start=tuple(start);self.elapsed=0.;self.state=(0.,)*6
        self.repeat_period=float(self.meta.get('repeat_period_s',0.));self.repeat_from=float(self.meta.get('repeat_from_s',0.))
        if not math.isfinite(self.repeat_period) or self.repeat_period<0:raise ValueError('Invalid repeating period')
        self.duration=math.inf if self.repeat_period else self.times[-1]
        if any(abs(v)>1e-5 for v in self.rows[0][1:]):raise ValueError('Angle trajectory must start at rest')
        if self.repeat_period:
            if not 0<=self.repeat_from<self.times[-1] or abs(self.repeat_from+self.repeat_period-self.times[-1])>1e-8:raise ValueError('Invalid repeating interval')
            j=min(range(len(self.rows)),key=lambda i:abs(self.times[i]-self.repeat_from))
            if abs(self.times[j]-self.repeat_from)>1e-8 or any(abs(x-y)>1e-7 for x,y in zip(self.rows[j][1:],self.rows[-1][1:])):raise ValueError('Repeating trajectory seam must match position, derivatives and feedforward')
        elif any(abs(v)>1e-5 for v in self.rows[-1][1:]):raise ValueError('Finite angle trajectory must end at rest at its origin')
        self.max_reference_speed=0.
        for a,b in zip(self.rows,self.rows[1:]):
            h=b[0]-a[0];c1=h*a[2];c2=.5*h*h*a[3]
            d=b[1]-a[1]-c1-c2;e=h*b[2]-c1-2*c2;f=h*h*b[3]-2*c2
            c3=10*d-4*e+.5*f;c4=-15*d+7*e-f;c5=6*d-3*e+.5*f
            roots=np.roots([20*c5,12*c4,6*c3,2*c2])
            u=[0.,1.]+[float(z.real) for z in roots if abs(z.imag)<1e-8 and 0<z.real<1]
            self.max_reference_speed=max(self.max_reference_speed,max(abs(hermite(*a[1:4],*b[1:4],h,t)[1]) for t in u))
    def sample(self,now):
        self.elapsed=max(0.,now-self.t0);t=min(self.elapsed,self.duration)
        if self.repeat_period and t>=self.repeat_from:t=self.repeat_from+(t-self.repeat_from)%self.repeat_period
        i=min(len(self.rows)-2,max(0,bisect.bisect_right(self.times,t)-1));a,b=self.rows[i:i+2];h=b[0]-a[0];u=(t-a[0])/h
        p,v,acc=hermite(*a[1:4],*b[1:4],h,u);theta,omega,alpha=hermite(*a[4:7],*b[4:7],h,u)
        feedforward=(1-u)*a[7]+u*b[7]
        self.physical_acceleration=acc
        self.state=(p,v,feedforward,theta,omega,alpha)
        # TEMP fixed yaw: the inherited terminal uses only this velocity return for heading.
        # Actual planned velocity remains in self.state for tracking/feedforward.
        return (self.start[0]+p,self.start[1],self.start[2]),(0.,0.,0.),self.elapsed>=self.duration

def reference_offset(controller,profile,state):
    # state[2] is command feedforward; physical acceleration is stored separately in the table.
    _,velocity,acc,theta,omega,_=state;kp=float(profile['kp_pos']);kd=float(profile['kd_pos'])
    if kp<=0:raise ValueError('Position Kp must be positive for reference adaptation')
    if controller=='adrc':
        l=float(profile.get('l',1.));g=float(profile.get('g',9.81));ks=float(profile['omega_c'])**2
        ds=2*float(profile['zeta'])*float(profile['omega_c']) if profile.get('enable_rate_term',True) else 0.
        if profile.get('nonlinear_antiswing',False):anti=(l*ks*theta+l*ds*omega-g*math.sin(theta))/math.cos(theta)
        else:anti=l*(ks-g/l)*theta+l*ds*omega
    else:anti=float(profile['kp_swing'])*math.sin(theta)+float(profile['kd_swing'])*omega*math.cos(theta)
    # Exact algebraic adaptation of the frozen controller's existing reference input:
    # a = a_ref + Kp(p_ref-p) + Kd(v_ref-v) + F(theta,rate,f)-F(theta_ref,rate_ref,0).
    return (acc+kd*velocity-anti)/kp

def measured_tracking_command(owner,motion):
    profile=owner.profiles[owner.profile_name()];p,v,feedforward,theta_ref,omega_ref,_=motion.state
    data=owner.sources[owner.estimator]
    if owner.controller=='adrc':
        theta=data['state'].theta_north_rad;omega=data['state'].rate_north_rad_s
    else:
        theta=data['angle'].vector.y;omega=data['rate'].vector.y
    k=owner.angle_feedback;x=owner.pos()[0];xd=owner.vel()[0]
    kp=k['sine_position_p'] if motion.repeat_period else k['position_p']
    kd=k['sine_position_d'] if motion.repeat_period else k['position_d']
    desired=feedforward-kp*(x-motion.start[0]-p)-kd*(xd-v)+k['angle_p']*(theta-theta_ref)+k['angle_d']*(omega-omega_ref)
    original_feedback=-float(profile['kp_pos'])*reference_offset(owner.controller,profile,(0.,0.,0.,theta,omega,0.))
    return x+(desired+float(profile['kd_pos'])*xd-original_feedback)/float(profile['kp_pos'])
