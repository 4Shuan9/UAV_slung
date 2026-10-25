#!/usr/bin/env python3
# Codex-generated EXPERIMENTAL extension, 2026-10-25.
# Reuses frozen executables; EXPERIMENTAL angle tracking uses its own error-feedback settings.
# Offline preview references include the measured PX4 command dynamics.
# TEMP: angle tasks keep yaw fixed. Baseline gains/algorithms/entry point are untouched.
import bisect
import csv
import json
import math
import shlex
import sys
import subprocess
import threading
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_prefix
sys.path.insert(0,str(Path(get_package_prefix('slung_ctrl'))/'lib'/'slung_ctrl'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point,Vector3Stamped
from std_msgs.msg import String
from offboard_console import Offboard


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
        theta=math.radians(data['state'].theta_north_deg);omega=math.radians(data['state'].rate_north_deg_s)
    else:
        theta=math.radians(data['angle'].vector.y);omega=math.radians(data['rate'].vector.y)
    k=owner.angle_feedback;x=owner.pos()[0];xd=owner.vel()[0]
    kp=k['sine_position_p'] if motion.repeat_period else k['position_p']
    kd=k['sine_position_d'] if motion.repeat_period else k['position_d']
    desired=feedforward-kp*(x-motion.start[0]-p)-kd*(xd-v)+k['angle_p']*(theta-theta_ref)+k['angle_d']*(omega-omega_ref)
    original_feedback=-float(profile['kp_pos'])*reference_offset(owner.controller,profile,(0.,0.,0.,theta,omega,0.))
    return x+(desired+float(profile['kd_pos'])*xd-original_feedback)/float(profile['kp_pos'])


class AngleOffboard(Offboard):
    def __init__(self):
        super().__init__()
        self.pub_angle=self.create_publisher(Vector3Stamped,'/uav/reference/angle',10)
        self.pub_angle_rate=self.create_publisher(Vector3Stamped,'/uav/reference/rate',10)
        self.pub_virtual=self.create_publisher(Point,'/uav/reference/controller_position',10)
        self.pub_angle_task=self.create_publisher(String,'/uav/reference/angle_task',10)
        self.angle_feedback=yaml.safe_load((self.share/'config'/'angle_tasks'/'feedback.yaml').read_text())
        for key in ('position_p','position_d','sine_position_p','sine_position_d','angle_p','angle_d','experimental_max_acceleration'):
            value=float(self.angle_feedback[key])
            if not math.isfinite(value) or value<0:raise ValueError('Invalid experimental feedback '+key)
        if self.angle_feedback['TEMP_fixed_yaw'] is not True:raise ValueError('This experimental version supports TEMP fixed yaw only')
        self.experimental_accel=float(self.angle_feedback['experimental_max_acceleration'])
        if self.experimental_accel<=0:raise ValueError('Experimental acceleration must be positive')
        for name in ('control_eso','control_ekf','control_pd'):self.profiles[name]['max_comp_acc']=self.experimental_accel
        self.last_angle_output=-1.
    def spawn_backend(self):
        exe='slung_control_'+self.controller
        cmd=[str(Path(get_package_prefix('slung_ctrl'))/'lib/slung_ctrl'/exe),'--ros-args','--params-file',str(self.share/'config'/(self.profile_name()+'.yaml')),
             '-p','use_sim_time:='+str(bool(self.get_parameter('use_sim_time').value)).lower()]
        cmd+=['-p','max_comp_acc:='+str(self.experimental_accel)]
        for a,b in [('/fmu/in/trajectory_setpoint',self.P+'trajectory_setpoint'),('/fmu/in/offboard_control_mode',self.P+'offboard_control_mode'),('/fmu/in/vehicle_command',self.P+'vehicle_command'),('/fmu/out/vehicle_odometry',self.P+'odometry'),('/fmu/out/vehicle_local_position',self.P+'local_position'),('/uav/reference/position',self.P+'reference_position'),('/uav/reference/yaw',self.P+'reference_yaw'),('/uav/control/active',self.P+'active')]:cmd+=['-r',a+':='+b]
        if self.controller=='adrc':cmd+=['-p','state_topic:=/uav/swing/'+self.estimator+'/state']
        else:
            for f in ('angle','rate'):cmd+=['-r',f'/uav/swing/eso_delay/{f}:=/uav/swing/{self.estimator}/{f}']
        self.backend=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
        def drain(p):
            for line in p.stdout:self.log_queue.put(line.strip())
        threading.Thread(target=drain,args=(self.backend,),daemon=True).start()
        self.phase='PRESTREAM';self.boot_time=self.now_s()
        self.say('EXPERIMENTAL: '+self.profile_name()+'.yaml plus max_comp_acc='+str(self.experimental_accel)+' m/s2 override. Original offboard entry is unchanged; prestreaming.')
    def show_menu(self,detailed=False):
        super().show_menu(detailed)
        self.say('EXPERIMENTAL: sweep: continuous +/-30 deg at 0.45 Hz, until pause | maneuver: 20 deg, 1.5 s. TEMP fixed yaw; experimental acceleration limit 4.5 m/s2.')
    def handle_command(self,line):
        parts=shlex.split(line.strip())
        if not parts:return
        cmd=parts[0].lower()
        if cmd in ('sweep','maneuver'):
            if len(parts)!=1:raise ValueError('Fixed offline trajectory: '+cmd+' takes no arguments')
            self.require_flight()
            if not self.can_switch():raise ValueError('Angle tasks start only at stable HOME; use home and wait')
            if not self.source_ready():raise ValueError('Selected feedback is missing or stale')
            filename='angle_sine.csv' if cmd=='sweep' else 'angle_hold.csv'
            try:
                motion=AngleReference(self.share/'config'/'angle_tasks'/filename,self.now_s(),(0.,0.,self.height),cmd)
            except (OSError,StopIteration,csv.Error) as exc:
                raise ValueError('Cannot load offline angle plan: '+str(exc)) from exc
            if not all(key in motion.meta for key in ('l','height_m','duration_s')):
                raise ValueError('Offline angle plan metadata is incomplete')
            profile=self.profiles[self.profile_name()]
            if abs(float(motion.meta['l'])-float(profile.get('l',self.cfg['cable_length'])))>1e-9:raise ValueError('Offline plan cable length does not match controller')
            if abs(self.height-float(motion.meta['height_m']))>1e-9:raise ValueError('Offline plan requires its recorded 8 m height')
            self.last_cmd=line;self.resume_motion=None;self.clear_paused_trajectory();self.install_motion(motion);self.task=cmd
            self.say('Started '+cmd+(' (continuous fixed-frequency sine; pause cancels and brakes; next start begins at zero).' if motion.repeat_period else ' (finite 20-deg hold, then smooth return; stopping cancels progress).')+' TEMP fixed yaw.');return
        if cmd=='params':
            super().handle_command(line)
            self.say('Angle plans: North plane, 8 m; sweep +/-30 deg, fixed 0.45 Hz continuously until pause; hold +20 deg for 1.5 s. Experimental max acceleration 4.5 m/s2; original offboard remains 3. Hold includes preparation/recovery; sine repeats until pause. Offline preview includes measured command dynamics; experimental tracking feedback from angle_tasks/feedback.yaml. TEMP fixed yaw.');return
        super().handle_command(line)
    def controller_reference(self,msg):
        out=super().controller_reference(msg);motion=self.motion
        if getattr(motion,'angle_motion',False) and self.antiswing_enabled:
            angular=measured_tracking_command(self,motion) if self.source_ready() else out.x+reference_offset(self.controller,self.profiles[self.profile_name()],motion.state)
            out.x=angular
        return out
    def publish_reference(self,msg):
        super().publish_reference(msg);motion=self.motion
        theta=omega=alpha=0.;label='hold';p=v=a=0.
        if getattr(motion,'angle_motion',False):p,v,a,theta,omega,alpha=motion.state;label=motion.kind
        private=self.last_private_reference
        self.pub_virtual.publish(Point(x=float(private[0]+self.home[0]),y=float(private[1]+self.home[1]),z=float(msg.z+self.home[2])))
        for pub,value in [(self.pub_angle,theta*180/math.pi),(self.pub_angle_rate,omega*180/math.pi)]:
            ref=Vector3Stamped();ref.header.stamp=self.get_clock().now().to_msg();ref.header.frame_id='world';ref.vector.y=value;pub.publish(ref)
        if self.now_s()-self.last_angle_output>=.1:
            self.last_angle_output=self.now_s()
            self.pub_angle_task.publish(String(data=json.dumps(dict(stamp_sec=self.now_s(),task=label,elapsed=motion.elapsed if motion else 0.,theta_north_deg=theta*180/math.pi,north_offset_m=p,north_velocity_m_s=v,north_acceleration_m_s2=a,TEMP_fixed_yaw=getattr(motion,'angle_motion',False)))))
    def finish_motion(self,motion):
        if isinstance(motion,AngleReference):
            self.stop_work(save=False,after='home')
            self.say('Angle work complete. Smooth stop and return to takeoff point.');return
        super().finish_motion(motion)
    def pause(self,request_hold):
        interrupted=getattr(self.motion,'angle_motion',False) or getattr(self.resume_motion,'angle_motion',False)
        super().pause(request_hold)
        if interrupted:
            self.clear_paused_trajectory();self.saved_ref=self.ref;self.task='hold'
            self.say('Manual/failsafe takeover canceled the angular reference; resume restores holding control.')


def main():
    rclpy.init();node=None;executor=SingleThreadedExecutor()
    try:
        node=AngleOffboard();executor.add_node(node)
        while rclpy.ok() and not node.shutdown_requested:executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close()
        executor.shutdown()
        if node:node.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__':main()
