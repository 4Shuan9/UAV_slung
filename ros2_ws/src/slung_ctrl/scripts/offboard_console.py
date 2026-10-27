#!/usr/bin/env python3
# Codex-generated file: interactive mission/reference layer around UNCHANGED baseline executables.
# Codex-generated consolidation, 2026-10-26: position + angle tasks in one entry.
# CLI: home-relative North/East/Up [m]. Public reference/PX4 topics retain absolute NED.
import copy
import csv
import json
import math
import os
import queue
import select
import shlex
import signal
import subprocess
import sys
import threading
import textwrap
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Point, Vector3Stamped
from std_msgs.msg import String, Float32, Bool
from px4_msgs.msg import (VehicleOdometry, VehicleLocalPosition, VehicleStatus,
                         VehicleCommand, TrajectorySetpoint, OffboardControlMode)
from slung_ctrl.msg import SwingADRCState
from offboard_reference import Reference, SmoothReference, AngleStopReference, reference_kinematics, plan_bridge, smoother, norm, valid_point, minimum_height, admissible_height, at_home, AngleReference, reference_offset, measured_tracking_command


class Offboard(Node):
    METHODS=('eso','eso_delay','ekf2','ekf3')
    P='/uav/offboard/backend/'
    def __init__(self):
        super().__init__('slung_offboard')
        defaults=dict(default_estimator='eso_delay',default_controller='adrc',startup_mode='idle',
                      after_takeoff_task='hold',control_hz=50.,prestream_sec=1.2,input_timeout=.5,
                      backend_timeout=.3,status_timeout=1.5,handshake_timeout=10.,home_position_tolerance=.25,
                      home_speed_tolerance=.15,home_stable_sec=2.,transition_speed=1.,
                      transition_acceleration=.5,trajectory_ramp_sec=2.,eight_loops=0,
                      cable_length=1.,payload_radius=.045,suspension_offset_bound=.24,
                      minimum_payload_clearance=.10,angle_reference_speed_limit=6.2,command_fifo='')
        self.cfg={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        self.estimator=self.cfg['default_estimator'];self.controller=self.cfg['default_controller']
        self.validate_selection(self.estimator,self.controller)
        if self.cfg['startup_mode'] not in ('idle','takeoff'):raise ValueError('startup_mode must be idle or takeoff')
        if self.cfg['after_takeoff_task'] not in ('hold','eight','square'):raise ValueError('Invalid after_takeoff_task')
        for k,v in self.cfg.items():
            if isinstance(v,(int,float)) and (not math.isfinite(v) or v<0):raise ValueError('Invalid '+k)
        for k in ('control_hz','prestream_sec','input_timeout','backend_timeout','status_timeout','handshake_timeout',
                  'home_position_tolerance','home_speed_tolerance','home_stable_sec','transition_speed',
                  'transition_acceleration','cable_length','payload_radius','angle_reference_speed_limit'):
            if self.cfg[k]<=0:raise ValueError(k+' must be positive')
        if self.cfg['control_hz']!=50.:raise ValueError('Baseline control_hz is fixed at 50 Hz')
        self.share=Path(get_package_share_directory('slung_ctrl'))
        self.profiles={}
        for name,node in [('control_eso','slung_control_adrc'),('control_ekf','slung_control_adrc'),('control_pd','slung_control_pd'),('eight','trajectory_eight'),('square','trajectory_square')]:
            self.profiles[name]=yaml.safe_load((self.share/'config'/f'{name}.yaml').read_text())[node]['ros__parameters']
        # Codex-generated: one console owns all position and angle tasks.
        self.angle_feedback=yaml.safe_load((self.share/'config'/'angle_tasks'/'feedback.yaml').read_text())
        for key in ('position_p','position_d','sine_position_p','sine_position_d','angle_p','angle_d','experimental_max_acceleration'):
            value=float(self.angle_feedback[key])
            if not math.isfinite(value) or value<0:raise ValueError('Invalid angle feedback '+key)
        if self.angle_feedback['TEMP_fixed_yaw'] is not True:raise ValueError('Angle tasks currently require fixed yaw')
        self.angle_acceleration=float(self.angle_feedback['experimental_max_acceleration'])
        if self.angle_acceleration<=0:raise ValueError('Angle acceleration must be positive')
        self.pub_angle=self.create_publisher(Vector3Stamped,'/uav/reference/angle',10)
        self.pub_angle_rate=self.create_publisher(Vector3Stamped,'/uav/reference/rate',10)
        self.pub_virtual=self.create_publisher(Point,'/uav/reference/controller_position',10)
        self.pub_angle_task=self.create_publisher(String,'/uav/reference/angle_task',10)
        self.last_angle_output=-1.
        self.height=float(self.profiles[self.profile_name()]['takeoff_height'])
        if self.cfg['eight_loops']<0:raise ValueError('eight_loops must be >=0 (0 means continuous)')
        self.required_z=minimum_height(self.cfg['cable_length'],self.cfg['payload_radius'],self.cfg['suspension_offset_bound'],self.cfg['minimum_payload_clearance'])
        if self.height<self.required_z:raise ValueError('Configured takeoff height violates payload clearance')
        self.phase='IDLE';self.task='hold';self.ref=(0.,0.,self.height);self.yaw=0.
        self.home=None;self.odom=None;self.local=None;self.vehicle=None
        self.odom_time=self.local_time=self.status_time=-1.;self.last_clock=None;self.resets=None
        self.motion=None;self.saved_motion=None;self.saved_ref=None
        self.trajectory_paused=False;self.antiswing_enabled=True
        self.stationary_since=None;self.takeoff_point=(0.,0.,self.height)
        self.last_private_reference=None;self.last_equivalent_reference=None;self.reference_offset=(0.,0.);self.offset_t0=0.;self.offset_duration=1.
        self.home_since=None;self.backend=None;self.retiring=None;self.boot_time=0.;self.reason=''
        self.backend_sp_time=-1.;self.control_ready=False;self.offboard_seen=False
        self.command_queue=queue.Queue();self.log_queue=queue.Queue();self.last_status_output=-1.
        self.reader_stop=threading.Event();self.reader=None;self.reader_fd=None;self.auto_started=False
        self.sources={m:{'state':None,'angle':None,'rate':None} for m in self.METHODS}
        self.last_mode_request=-1.;self.last_cmd='';self.shutdown_requested=False
        self.pub_sp=self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',10)
        self.pub_mode=self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',10)
        self.pub_cmd=self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
        self.pub_ref=self.create_publisher(Point,self.P+'reference_position',10)
        self.pub_yaw=self.create_publisher(Float32,self.P+'reference_yaw',10)
        self.pub_odom=self.create_publisher(VehicleOdometry,self.P+'odometry',qos_profile_sensor_data)
        self.pub_local=self.create_publisher(VehicleLocalPosition,self.P+'local_position',qos_profile_sensor_data)
        self.pub_plot=self.create_publisher(Point,'/uav/reference/position',10)
        self.pub_plot_yaw=self.create_publisher(Float32,'/uav/reference/yaw',10)
        self.pub_active=self.create_publisher(Bool,'/uav/control/active',10)
        self.pub_status=self.create_publisher(String,'/uav/offboard/status',10)
        self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position',self.on_local,qos_profile_sensor_data)
        for topic in ('/fmu/out/vehicle_status_v1','/fmu/out/vehicle_status'):
            self.create_subscription(VehicleStatus,topic,self.on_status,qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint,self.P+'trajectory_setpoint',self.on_backend_sp,10)
        self.create_subscription(OffboardControlMode,self.P+'offboard_control_mode',self.on_backend_mode,10)
        self.create_subscription(VehicleCommand,self.P+'vehicle_command',self.on_backend_command,10)
        self.create_subscription(String,'/uav/offboard/command',lambda m:self.command_queue.put(m.data),10)
        for m in self.METHODS:
            if m!='ekf2':
                self.create_subscription(SwingADRCState,f'/uav/swing/{m}/state',lambda v,m=m:self.record_source(m,'state',v),10)
            for field in ('angle','rate'):
                self.create_subscription(Vector3Stamped,f'/uav/swing/{m}/{field}',lambda v,m=m,f=field:self.record_source(m,f,v),10)
        self.add_on_set_parameters_callback(self.parameter_guard)
        self.timer=self.create_timer(1/self.cfg['control_hz'],self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.start_reader()
        self.show_menu()

    def now_s(self):return self.get_clock().now().nanoseconds*1e-9
    def stamp(self):return int(self.now_s()*1e6)
    def armed(self):return self.vehicle is not None and self.vehicle.arming_state==VehicleStatus.ARMING_STATE_ARMED
    def ready(self):
        t=self.now_s()
        return self.odom is not None and self.local is not None and self.vehicle is not None and 0<=t-self.odom_time<self.cfg['input_timeout'] and 0<=t-self.local_time<self.cfg['input_timeout'] and 0<=t-self.status_time<self.cfg['status_timeout']
    def pos(self):
        if self.home is None or self.odom is None:return (0.,0.,0.)
        return (float(self.odom.position[0])-float(self.home[0]),float(self.odom.position[1])-float(self.home[1]),float(self.home[2])-float(self.odom.position[2]))
    def vel(self):
        if self.odom is None:return (0.,0.,0.)
        return (float(self.odom.velocity[0]),float(self.odom.velocity[1]),-float(self.odom.velocity[2]))
    def profile_name(self):return 'control_pd' if self.controller=='pd' else ('control_ekf' if self.estimator=='ekf3' else 'control_eso')
    @staticmethod
    def validate_selection(estimator,controller):
        if estimator not in Offboard.METHODS or controller not in ('adrc','pd'):raise ValueError('Unknown estimator/controller')
        if controller=='adrc' and estimator=='ekf2':raise ValueError('EKF2 supports PD only; select controller pd first, at home')
    def parameter_guard(self,params):
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=False,reason='Edit offboard.yaml before restart; use guarded terminal commands to switch source/controller')
    def prompt(self):
        return f'[SLUNG:{self.phase} | {self.estimator}+{self.controller}] > '
    def say(self,text):
        print('\n'+text+'\n'+self.prompt(),end='',flush=True)
    def panel(self,title,lines):
        width=74
        border='+'+'-'*(width+2)+'+'
        rendered=[border,'| '+title.center(width)+' |',border]
        for line in lines:
            for part in textwrap.wrap(str(line),width=width,replace_whitespace=False) or ['']:
                rendered.append('| '+part.ljust(width)+' |')
        rendered.append(border)
        self.say('\n'.join(rendered))
    def show_menu(self, detailed=False):
        loop_default='continuous' if self.cfg['eight_loops']==0 else str(self.cfg['eight_loops'])+' loops'
        startup=('Waiting for commands; startup does not arm the vehicle.' if self.cfg['startup_mode']=='idle'
                 else 'Configured auto-start: TAKEOFF after required inputs are ready.')
        if not detailed:
            self.panel('UAV_slung | INTERACTIVE OFFBOARD',[
                f'{self.phase}    Observer: {self.estimator}    Controller: {self.controller.upper()}',
                'Coordinates: home-relative NORTH / EAST / UP [m].',
                '',
                f'takeoff / land       Take off to {self.height:g} m / land',
                f'hold / home          Stop here / clear work and return at {self.height:g} m',
                'eight [loops]        Figure-eight (default: '+loop_default+')',
                'square               Rhombus trajectory',
                'sine                 +/-30 deg sine, 0.45 Hz; continuous',
                'maneuver             20 deg hold for 1.5 s; then return',
                'setpoint N E U       Move to a 3-D target',
                'pause / resume       Smooth stop / continue saved work',
                'control off          Disable anti-swing only after stopping',
                'reset                Clear work, hold here, anti-swing ON',
                'estimator NAME       eso | eso_delay | ekf2 | ekf3',
                'controller NAME      adrc | pd (EKF2: PD only)',
                'Angles: HOME start, fixed yaw; pause cancels, no resume.',
                'status / params / help / quit',
                '',
                f'Switch at HOME only: {self.height:g} m, low speed, stable {self.cfg["home_stable_sec"]:g} s.',
                f'Minimum UP: {self.required_z:.2f} m. '+('Startup: IDLE (no auto-arm).' if self.cfg['startup_mode']=='idle' else 'Auto-start: TAKEOFF.')])
            return
        self.panel('UAV_slung | INTERACTIVE OFFBOARD',[
            f'State: {self.phase}    Observer: {self.estimator}    Controller: {self.controller.upper()}',
            'Coordinates: home-relative NORTH / EAST / UP [m]; positive Z is UP.',
            '',
            'FLIGHT       takeoff              Take off to '+f'{self.height:g} m',
            '             hold                 Anti-swing hold at current position',
            '             home                 Return above takeoff home at '+f'{self.height:g} m',
            '             land                 Land and wait for disarming',
            '',
            'TRAJECTORY   eight [loops]        Default: '+loop_default+'; 0 = continuous',
            '             square               Run the original rhombus trajectory',
            '             sine                 +/-30 deg sine, 0.45 Hz; continuous',
            '             maneuver             20 deg hold for 1.5 s; then return',
            '             setpoint N E U       Move to a 3-D coordinate [m]',
            '',
            'MISSION      pause / resume       Smooth stop / continue saved work',
            '             control off          Disable anti-swing only when stopped',
            '             reset                Clear work; hold here with anti-swing',
            'SELECTION    estimator NAME       eso | eso_delay | ekf2 | ekf3',
            '             controller NAME      adrc | pd (EKF2 requires PD)',
            'INSPECT      status | params | help',
            'EXIT         quit                 Exit after landing or PX4 Hold',
            '',
            'Switch only at HOME: '+f'{self.height:g} m, low speed, stable for {self.cfg["home_stable_sec"]:g} s.',
            f'Setpoint minimum UP: {self.required_z:.2f} m (configured payload clearance).',
            'Angles: HOME start, fixed yaw; pause cancels, no resume.',
            f'Angle reference speed <= {self.cfg["angle_reference_speed_limit"]:g} m/s (simulation).',
            'Quick flow: takeoff -> eight / sine / maneuver -> home -> land',
            startup])
    def show_status(self):
        state=self.publish_status()
        nav='WAITING'
        if self.vehicle is not None:
            nav=next((name.replace('NAVIGATION_STATE_','') for name in dir(VehicleStatus)
                      if name.startswith('NAVIGATION_STATE_') and getattr(VehicleStatus,name)==self.vehicle.nav_state),
                     str(self.vehicle.nav_state))
        fmt=lambda values:'['+', '.join(f'{x:+.3f}' for x in values)+']'
        pose=fmt(self.pos())+' m' if self.home is not None else 'Home not captured; run takeoff first'
        goal=fmt(self.ref)+' m' if self.home is not None else 'Not active'
        lines=[f'Phase / task    : {self.phase} / {self.task}',
               f'PX4 mode        : {nav}    Armed: {"YES" if self.armed() else "NO"}',
               'PX4 feedback    : '+('READY' if self.ready() else 'WAITING / STALE'),
               f'Observer        : {self.estimator}    Controller: {self.controller.upper()}',
               'Anti-swing      : '+('ON' if self.antiswing_enabled else 'OFF - position-only mode'),
               'Trajectory      : '+('PAUSED' if self.trajectory_paused else (self.task.upper() if self.phase=='FLIGHT' else self.phase)),
               f'Controller file : {self.profile_name()}.yaml',
               f'Acceleration cap: {self.acceleration_limit():g} m/s2 per horizontal axis',
               'Swing feedback  : '+('READY' if self.source_ready() else 'WAITING / STALE'),
               '', 'Position N/E/U  : '+pose, 'Target N/E/U    : '+goal,
               'Velocity N/E/U  : '+fmt(self.vel())+' m/s']
        if isinstance(self.motion,SmoothReference):lines.append(f'Smooth transition: {max(0.,self.motion.duration-self.motion.elapsed):.1f} s remaining')
        if self.local is not None:lines.append(f'Heading         : {math.degrees(self.local.heading):+.1f} deg (North=0, East=90)')
        data=self.sources[self.estimator]
        angle=data['angle'];rate=data['rate'];estimate=data['state']
        if angle is not None:lines.append('Swing East/North: '+f'{angle.vector.x:+.3f} / {angle.vector.y:+.3f} deg')
        if rate is not None:lines.append('Rate East/North : '+f'{rate.vector.x:+.3f} / {rate.vector.y:+.3f} deg/s')
        if estimate is not None:lines.append('Disturbance E/N : '+f'{estimate.disturbance_east_rad_s2:+.4f} / {estimate.disturbance_north_rad_s2:+.4f} rad/s^2')
        lines.extend(['', 'Selection gate  : '+('ALLOWED - stable at home' if state['switch_allowed'] else 'LOCKED - return home and wait for stable hover'),
                      f'Home condition  : position <= {self.cfg["home_position_tolerance"]:g} m; speed <= {self.cfg["home_speed_tolerance"]:g} m/s',
                      f'Minimum UP      : {self.required_z:.2f} m',
                      'Control off gate: '+('ALLOWED' if self.can_disable_antiswing() else 'LOCKED - stop and settle first'),
                      'Last command    : '+(self.last_cmd or '(none)')])
        self.panel('UAV_slung | FLIGHT STATUS',lines)
    def record_source(self,m,f,msg):self.sources[m][f]=msg
    def source_present(self):
        fields=('state',) if self.controller=='adrc' else ('angle','rate')
        return all(self.get_publishers_info_by_topic(f'/uav/swing/{self.estimator}/{f}') for f in fields)
    def source_ready(self,m=None,c=None):
        m=m or self.estimator;c=c or self.controller;t=self.now_s()
        fields=('state',) if c=='adrc' else ('angle','rate')
        for f in fields:
            v=self.sources[m][f]
            if v is None:return False
            ts=v.header.stamp.sec+v.header.stamp.nanosec*1e-9
            if not -.02<=t-ts<=.10:return False
            values=([v.theta_east_deg,v.theta_north_deg,v.rate_east_deg_s,v.rate_north_deg_s,v.disturbance_east_rad_s2,v.disturbance_north_rad_s2] if f=='state' else [v.vector.x,v.vector.y])
            if not all(math.isfinite(x) for x in values):return False
        return True
    def on_odom(self,msg):
        if msg.pose_frame!=VehicleOdometry.POSE_FRAME_NED or msg.velocity_frame!=VehicleOdometry.VELOCITY_FRAME_NED or not all(math.isfinite(x) for x in [*msg.position,*msg.velocity]):return
        self.odom=msg;self.odom_time=self.now_s()
        if self.home is None or not at_home(self.pos(),self.vel(),self.height,self.cfg['home_position_tolerance'],self.cfg['home_speed_tolerance']):self.home_since=None
        if self.backend_receives():
            v=copy.deepcopy(msg);v.position=[float(msg.position[i]-self.home[i]) for i in range(3)];self.pub_odom.publish(v)
    def on_local(self,msg):
        if not math.isfinite(msg.heading):return
        reset=(msg.xy_reset_counter,msg.z_reset_counter,msg.heading_reset_counter)
        if self.resets is not None:
            if self.home is not None:
                if reset[0]!=self.resets[0] and not all(math.isfinite(x) for x in msg.delta_xy):return
                if reset[1]!=self.resets[1] and not math.isfinite(msg.delta_z):return
                if reset[0]!=self.resets[0]:
                    self.home[0]=float(self.home[0])+float(msg.delta_xy[0]);self.home[1]=float(self.home[1])+float(msg.delta_xy[1])
                    self.home_since=None
                if reset[1]!=self.resets[1]:
                    self.home[2]=float(self.home[2])+float(msg.delta_z);self.home_since=None
            if reset[2]!=self.resets[2] and self.motion is None and math.isfinite(msg.delta_heading):
                self.yaw=math.atan2(math.sin(self.yaw+msg.delta_heading),math.cos(self.yaw+msg.delta_heading))
        self.resets=reset;self.local=msg;self.local_time=self.now_s()
        if self.backend_receives():self.pub_local.publish(msg)
    def finish_disarmed(self):
        landed=self.phase=='LANDING'
        self.stop_backend();self.phase='IDLE';self.task='hold'
        self.motion=self.saved_motion=self.saved_ref=None
        self.trajectory_paused=False
        self.home_since=None;self.offboard_seen=False;self.control_ready=False
        self.ref=(0.,0.,self.height)
        self.say('PX4 confirms disarmed. '+('Landing complete.' if landed else 'Flight task cleared; terminal is IDLE.'))
    def on_status(self,msg):
        was_armed=self.armed()
        self.vehicle=msg;self.status_time=self.now_s()
        if was_armed and not self.armed() and self.phase!='IDLE':self.finish_disarmed()
        if not self.armed() or msg.nav_state not in (VehicleStatus.NAVIGATION_STATE_OFFBOARD,VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):self.home_since=None
    def backend_receives(self):
        return self.home is not None and self.backend is not None and self.backend.poll() is None and self.phase in ('PRESTREAM','TAKEOFF','FLIGHT') and self.now_s()-self.boot_time>=self.cfg['prestream_sec']
    def on_backend_command(self,msg):
        if self.backend_receives() and msg.timestamp>=int(self.boot_time*1e6):self.pub_cmd.publish(msg)
    def on_backend_mode(self,msg):
        if self.backend_receives() and msg.timestamp>=int(self.boot_time*1e6):self.pub_mode.publish(msg)
    def on_backend_sp(self,msg):
        if not self.backend_receives() or msg.timestamp<int(self.boot_time*1e6):return
        if not (math.isfinite(msg.position[2]) and math.isfinite(msg.yaw)):return
        if any(math.isinf(x) for x in [*msg.position,*msg.velocity,*msg.acceleration]):return
        self.backend_sp_time=self.now_s()
        self.control_ready=math.isfinite(msg.acceleration[0]) and math.isfinite(msg.acceleration[1])
        v=copy.deepcopy(msg)
        # Backend headroom is not a global increase: normal tasks retain YAML caps.
        limit=self.acceleration_limit()
        for axis in range(2):
            if math.isfinite(v.acceleration[axis]):v.acceleration[axis]=float(max(-limit,min(limit,v.acceleration[axis])))
        backend_acceleration=tuple(float(x) for x in v.acceleration[:2])
        v.position=[float(x+self.home[i]) if math.isfinite(x) else float('nan') for i,x in enumerate(msg.position)]
        if not self.antiswing_enabled and all(math.isfinite(x) for x in msg.acceleration[:2]):
            # Explicit POSITION-ONLY mode. Keep estimator/backend state untouched;
            # forward the unchanged position PD law without its swing/disturbance terms.
            profile=self.profiles[self.profile_name()];position=self.pos();velocity=self.vel()
            limit=float(profile['max_comp_acc'])
            for axis in range(2):
                command=float(profile['kp_pos'])*(self.ref[axis]-position[axis])-float(profile['kd_pos'])*velocity[axis]
                v.acceleration[axis]=float(max(-limit,min(limit,command)))
        if self.last_private_reference is not None and self.control_ready:
            kp=float(self.profiles[self.profile_name()]['kp_pos'])
            self.last_equivalent_reference=tuple(self.last_private_reference[i]+(float(v.acceleration[i])-backend_acceleration[i])/kp for i in range(2))
        self.pub_sp.publish(v)
        if self.phase=='PRESTREAM' and self.reason=='takeoff':self.phase='TAKEOFF'
        if self.control_ready and self.armed() and self.vehicle.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.phase=='TAKEOFF' and abs(self.pos()[2]-self.height)<.3 and self.source_ready():
                self.phase='FLIGHT';self.task='hold';self.ref=(0.,0.,self.height);self.takeoff_point=tuple(self.ref);self.home_since=None
                self.say('Takeoff complete. Holding position; anti-swing '+('ON' if self.antiswing_enabled else 'OFF')+'.')
                if self.cfg['after_takeoff_task']!='hold':self.command_queue.put(self.cfg['after_takeoff_task'])
            elif self.phase=='PRESTREAM' and self.reason!='takeoff':
                self.phase='FLIGHT';self.task='hold';self.home_since=None
                if self.reason=='resume' and self.saved_ref is not None:
                    if self.saved_motion is not None:self.resume_trajectory()
                    else:
                        self.motion=self.point_reference('rejoin',self.saved_ref)
                        self.task='rejoin'
                        self.clear_paused_trajectory()
                self.say('Offboard feedback connected. Current task: '+self.task)
    def command_px4(self,command,p1=0.,p2=0.,p3=0.):
        v=VehicleCommand();v.timestamp=self.stamp();v.command=command
        v.param1=float(p1);v.param2=float(p2);v.param3=float(p3)
        v.target_system=1;v.target_component=1;v.source_system=1;v.source_component=1;v.from_external=True
        self.pub_cmd.publish(v)
    def manual_hold(self):
        if self.home is None:return
        mode=OffboardControlMode();mode.timestamp=self.stamp();mode.position=True;self.pub_mode.publish(mode)
        sp=TrajectorySetpoint();sp.timestamp=self.stamp()
        sp.position=[float(self.home[0]+self.ref[0]),float(self.home[1]+self.ref[1]),float(self.home[2]-self.ref[2])]
        sp.velocity=[float('nan')]*3;sp.acceleration=[float('nan')]*3;sp.jerk=[float('nan')]*3;sp.yaw=float(self.yaw);sp.yawspeed=float('nan')
        self.pub_sp.publish(sp)
    def start_reader(self):
        path=self.cfg['command_fifo']
        self.reader_fd=os.open(path,os.O_RDWR|os.O_NONBLOCK) if path else sys.stdin.fileno()
        def read():
            pending=b''
            while not self.reader_stop.is_set():
                try:
                    ready,_,_=select.select([self.reader_fd],[],[],.1)
                    if not ready:continue
                    data=os.read(self.reader_fd,4096)
                    if not data:break
                    pending+=data
                    while b'\n' in pending:
                        line,pending=pending.split(b'\n',1);self.command_queue.put(line.decode(errors='replace').strip())
                except (OSError,ValueError):break
        self.reader=threading.Thread(target=read,daemon=True);self.reader.start()
    def stop_backend(self):
        if self.backend is None:return
        p=self.backend;self.backend=None;self.retiring=p
        try:p.send_signal(signal.SIGINT)
        except ProcessLookupError:pass
        def reap():
            try:p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:p.wait(timeout=1)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        threading.Thread(target=reap,daemon=True).start()
    def begin_backend(self,reason):
        self.last_equivalent_reference=None;self.last_private_reference=None
        self.stop_backend();self.reason=reason;self.phase='RECONFIGURE';self.boot_time=self.now_s();self.backend_sp_time=-1.
        self.control_ready=False;self.offboard_seen=False;self.home_since=None;self.last_mode_request=-1.
    def spawn_backend(self):
        exe='slung_control_'+self.controller
        cmd=[str(Path(get_package_prefix('slung_ctrl'))/'lib/slung_ctrl'/exe),'--ros-args','--params-file',str(self.share/'config'/(self.profile_name()+'.yaml')),
             '-p','use_sim_time:='+str(bool(self.get_parameter('use_sim_time').value)).lower()]
        cmd+=['-p','max_comp_acc:='+str(max(float(self.profiles[self.profile_name()]['max_comp_acc']),self.angle_acceleration))]
        for a,b in [('/fmu/in/trajectory_setpoint',self.P+'trajectory_setpoint'),('/fmu/in/offboard_control_mode',self.P+'offboard_control_mode'),('/fmu/in/vehicle_command',self.P+'vehicle_command'),('/fmu/out/vehicle_odometry',self.P+'odometry'),('/fmu/out/vehicle_local_position',self.P+'local_position'),('/uav/reference/position',self.P+'reference_position'),('/uav/reference/yaw',self.P+'reference_yaw'),('/uav/control/active',self.P+'active')]:cmd+=['-r',a+':='+b]
        if self.controller=='adrc':cmd+=['-p','state_topic:=/uav/swing/'+self.estimator+'/state']
        else:
            for f in ('angle','rate'):cmd+=['-r',f'/uav/swing/eso_delay/{f}:=/uav/swing/{self.estimator}/{f}']
        self.backend=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
        def drain(p):
            for line in p.stdout:self.log_queue.put(line.strip())
        threading.Thread(target=drain,args=(self.backend,),daemon=True).start()
        self.phase='PRESTREAM';self.boot_time=self.now_s()
        self.say('Loaded '+self.profile_name()+'.yaml; normal acceleration cap '+str(self.profiles[self.profile_name()]['max_comp_acc'])+', angle-task cap '+str(self.angle_acceleration)+' m/s2; prestreaming.')
    def point_reference(self,kind,target):
        return Reference(kind,self.now_s(),self.pos(),goal=target,speed=self.cfg['transition_speed'],accel=self.cfg['transition_acceleration'])
    def can_switch(self):
        return self.phase in ('FLIGHT','PAUSED') and self.home is not None and self.motion is None and self.home_since is not None and self.now_s()-self.home_since>=self.cfg['home_stable_sec'] and self.ready() and self.armed() and at_home(self.pos(),self.vel(),self.height,self.cfg['home_position_tolerance'],self.cfg['home_speed_tolerance']) and self.vehicle.nav_state==(VehicleStatus.NAVIGATION_STATE_OFFBOARD if self.phase=='FLIGHT' else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)
    def work_motion(self):
        if self.saved_motion is not None:return self.saved_motion
        if isinstance(self.motion,SmoothReference):return self.motion.next_motion
        return self.motion
    def publish_status(self):
        out=dict(phase=self.phase,task=self.task,estimator=self.estimator,controller=self.controller,armed=self.armed(),
                 nav_state=self.vehicle.nav_state if self.vehicle else None,ready=self.ready(),home_captured=self.home is not None,
                 position_neu=self.pos(),velocity_neu=self.vel(),reference_neu=tuple(float(x) for x in self.ref),switch_allowed=self.can_switch(),
                 source_ready=self.source_ready(),minimum_setpoint_z=self.required_z,profile=self.profile_name(),last_command=self.last_cmd,
                 antiswing_enabled=self.antiswing_enabled,trajectory_paused=self.trajectory_paused,
                 acceleration_limit_m_s2=self.acceleration_limit(),angle_reference_speed_limit_m_s=self.cfg['angle_reference_speed_limit'],
                 control_off_allowed=self.can_disable_antiswing(),work_kind=self.work_motion().kind if self.work_motion() else None,
                 saved_progress_s=float(self.saved_motion.elapsed) if self.saved_motion else None,
                 progress_s=float(self.work_motion().elapsed) if self.work_motion() else None,
                 takeoff_point_neu=self.takeoff_point,transition_remaining_s=max(0.,self.motion.duration-self.motion.elapsed) if isinstance(self.motion,SmoothReference) else 0.)
        self.pub_status.publish(String(data=json.dumps(out,allow_nan=False)))
        return out
    def show_params(self):
        def value(v):
            if isinstance(v,bool):return 'ON' if v else 'OFF'
            return f'{v:g}' if isinstance(v,(float,int)) else str(v)
        lines=[]
        for title,items in [('OPERATION - offboard.yaml',{k:v for k,v in self.cfg.items() if k!='command_fifo'}),
                            ('CONTROLLER - '+self.profile_name()+'.yaml',self.profiles[self.profile_name()]),
                            ('FIGURE-EIGHT - eight.yaml',self.profiles['eight']),
                            ('RHOMBUS - square.yaml',self.profiles['square']),
                            ('ANGLE TASKS - feedback.yaml (active only in angular work)',self.angle_feedback)]:
            if lines:lines.append('')
            lines.append(title)
            lines.extend(f'  {k:<30} : {value(v)}' for k,v in items.items())
        self.panel('UAV_slung | PARAMETERS (READ ONLY)',lines)
    def require_flight(self):
        if self.phase!='FLIGHT' or not self.ready():raise ValueError('Requires active Offboard flight; takeoff or resume first')
    def handle_command(self,line):
        line=line.strip()
        if not line:return
        parts=shlex.split(line)
        if not parts:return
        self.last_cmd=line;cmd=parts[0].lower();args=parts[1:]
        if cmd=='help':self.show_menu(detailed=True);return
        if cmd=='status':self.show_status();return
        if cmd=='params':self.show_params();return
        if cmd=='control':
            if args!=['off']:raise ValueError('Usage: control off; resume/new tasks/reset/home enable anti-swing automatically')
            if not self.can_disable_antiswing():raise ValueError('Stop the trajectory and wait for stable position/speed before control off')
            self.antiswing_enabled=False
            self.say('Anti-swing OFF. Position hold remains active; work/resume/reset/home will enable anti-swing automatically.');return
        if cmd in ('estimator','controller'):
            if len(args)!=1:raise ValueError('Usage: '+cmd+' NAME')
            if not self.can_switch():raise ValueError('Switch refused: finish/pause trajectory with hold, command home, and wait for stable home at 8 m')
            e=args[0] if cmd=='estimator' else self.estimator;c=args[0] if cmd=='controller' else self.controller
            self.validate_selection(e,c)
            if not self.source_ready(e,c):raise ValueError('Selected feedback is missing or stale; run estimation estimator:=all')
            if (e,c)==(self.estimator,self.controller):self.say('Already selected.');return
            self.estimator=e;self.controller=c;self.height=float(self.profiles[self.profile_name()]['takeoff_height'])
            self.motion=None;self.clear_paused_trajectory();self.saved_ref=(0.,0.,self.height);self.ref=self.saved_ref
            if self.phase=='PAUSED':
                self.say('Selection updated at stable home. PX4 Hold remains active; use resume to re-enter Offboard.')
            else:self.begin_backend('switch')
            return
        if args and cmd not in ('setpoint','eight'):raise ValueError(cmd+' takes no arguments')
        if cmd=='takeoff':
            if self.phase!='IDLE' or not self.ready() or self.armed():raise ValueError('Takeoff requires IDLE, fresh PX4 feedback and a disarmed vehicle')
            if not self.source_ready():
                if not self.source_present():
                    raise ValueError('Estimator node missing; run estimation estimator:=all')
                self.say('Ground visual feedback not initialized yet. Using the unchanged position takeoff phase; trajectory commands remain locked until fresh swing feedback arrives at hover.')
            if len(self.get_publishers_info_by_topic('/fmu/in/trajectory_setpoint'))>1:
                raise ValueError('Another PX4 setpoint publisher exists; stop the other controller/terminal')
            self.home=[float(x) for x in self.odom.position];self.yaw=float(self.local.heading)
            self.ref=(0.,0.,0.);self.motion=None;self.clear_paused_trajectory();self.task='takeoff'
            self.antiswing_enabled=True;self.begin_backend('takeoff');return
        if cmd=='reset' and self.phase=='IDLE':
            self.clear_paused_trajectory();self.motion=None;self.antiswing_enabled=True;self.task='hold'
            self.say('Work records cleared; anti-swing will be enabled when flying.');return
        if cmd=='pause':
            self.pause_trajectory();return
        if cmd=='resume':
            if self.phase=='FLIGHT':
                if not self.trajectory_paused:raise ValueError('No paused trajectory to resume')
                if self.motion is not None or not self.stopped():raise ValueError('Still braking/settling; wait until stopped before resume')
                self.resume_trajectory();return
            if self.phase!='PAUSED' or not self.ready() or not self.armed():raise ValueError('Resume requires PAUSED and an armed vehicle with fresh feedback')
            if not self.source_ready():raise ValueError('Selected estimator not ready')
            if len(self.get_publishers_info_by_topic('/fmu/in/trajectory_setpoint'))>1:
                raise ValueError('Another PX4 setpoint publisher exists; stop it before resume')
            self.antiswing_enabled=True;self.ref=self.pos();self.motion=None;self.begin_backend('resume');return
        if cmd=='land':
            if not self.ready() or not self.armed():raise ValueError('Vehicle is not armed or feedback is unavailable')
            self.stop_backend();self.motion=None;self.clear_paused_trajectory();self.phase='LANDING';self.last_mode_request=-1.
            self.command_px4(VehicleCommand.VEHICLE_CMD_NAV_LAND);self.say('Landing requested; waiting for PX4 landed/disarmed confirmation.');return
        if cmd in ('quit','exit'):
            if self.armed() and self.phase!='PAUSED':raise ValueError('Land or select PX4 Hold in QGC before quitting')
            self.shutdown_requested=True;return
        if cmd not in ('hold','home','reset','setpoint','eight','square','sine','maneuver'):
            raise ValueError('Unknown command; use setpoint N E U for a 3D target, or type help')
        self.require_flight()
        if cmd in ('sine','maneuver'):
            self.start_angle_task(cmd);return
        if cmd in ('hold','reset'):
            self.stop_work(save=False,after='hold');self.say('Work cleared. Smooth stop and position hold; anti-swing ON.');return
        if cmd=='home':
            self.stop_work(save=False,after='home');self.say('Work cleared. Smooth stop, then return to takeoff-complete point; anti-swing ON.');return
        if cmd=='setpoint':
            if len(args)!=3:raise ValueError('Usage: setpoint NORTH EAST UP (metres)')
            p=tuple(float(x) for x in args)
            if not valid_point(p) or not admissible_height(p[2],self.required_z):raise ValueError(f'Unsafe/invalid setpoint: minimum UP z is {self.required_z:.2f} m (rope + sphere + suspension bound + clearance)')
            if self.pos()[2]<self.required_z:raise ValueError('Current clearance is below the conservative task bound; use home first')
            self.clear_paused_trajectory();self.install_motion(self.point_reference('setpoint',p));self.task='setpoint';self.say('Moving to NEU '+str(p));return
        if cmd in ('eight','square'):
            if self.motion is not None:raise ValueError('Use hold before starting another cyclic task')
            if self.pos()[2]<self.required_z:raise ValueError('Insufficient payload clearance; use home first')
            loops=int(args[0]) if args else int(self.cfg['eight_loops'])
            if len(args)>1 or loops<0:raise ValueError('Usage: eight [nonnegative integer loops]; 0 = continuous')
            kw=dict(scale=float(self.profiles['eight']['eight_scale']),omega=float(self.profiles['eight']['omega']),loops=loops,
                    side=float(self.profiles['square']['square_side_length']),speed=float(self.profiles['square']['cruise_speed']),ramp=self.cfg['trajectory_ramp_sec'])
            self.clear_paused_trajectory();self.install_motion(Reference(cmd,self.now_s(),self.pos(),**kw));self.task=cmd
            mode=(' (continuous)' if loops==0 else f' ({loops} loops)') if cmd=='eight' else ''
            self.say('Started '+cmd+mode+'; geometry/speed read from original trajectory YAML.');return
        raise ValueError('Unknown command; type help')
    def clear_paused_trajectory(self):
        self.saved_motion=self.saved_ref=None
        self.trajectory_paused=False
    def acceleration_limit(self):
        angular=getattr(self.motion,'angle_motion',False) or getattr(self.motion,'angle_stop',False)
        return self.angle_acceleration if angular else float(self.profiles[self.profile_name()]['max_comp_acc'])
    def start_angle_task(self,cmd):
        if not self.can_switch():raise ValueError('Angle tasks start only at stable HOME; use home and wait')
        if not self.source_ready():raise ValueError('Selected feedback is missing or stale')
        filename='angle_sine.csv' if cmd=='sine' else 'angle_hold.csv'
        try:motion=AngleReference(self.share/'config'/'angle_tasks'/filename,self.now_s(),(0.,0.,self.height),cmd)
        except (OSError,StopIteration,csv.Error) as exc:raise ValueError('Cannot load offline angle plan: '+str(exc)) from exc
        if not all(k in motion.meta for k in ('l','height_m','duration_s')):raise ValueError('Incomplete angle plan metadata')
        profile=self.profiles[self.profile_name()]
        if abs(float(motion.meta['l'])-float(profile.get('l',self.cfg['cable_length'])))>1e-9:raise ValueError('Angle plan cable length mismatch')
        if abs(self.height-float(motion.meta['height_m']))>1e-9:raise ValueError('Angle plan requires recorded 8 m height')
        if motion.max_reference_speed>self.cfg['angle_reference_speed_limit']+1e-6:
            raise ValueError(f"Plan requires {motion.max_reference_speed:.2f} m/s, above configured angle reference limit {self.cfg['angle_reference_speed_limit']:.2f}; replan rather than clip")
        self.clear_paused_trajectory();self.install_motion(motion);self.task=cmd
        self.say(f'Started {cmd}; reference peak {motion.max_reference_speed:.2f} m/s, acceleration cap {self.angle_acceleration:g} m/s2. Fixed yaw; pause cancels, no angular resume.')
    def controller_reference(self,msg):
        out=copy.deepcopy(msg);motion=self.motion
        if getattr(motion,'angle_motion',False) and self.antiswing_enabled:
            out.x=measured_tracking_command(self,motion) if self.source_ready() else out.x+reference_offset(self.controller,self.profiles[self.profile_name()],motion.state)
        return out
    def publish_reference(self,msg):
        out=self.controller_reference(msg)
        u=max(0.,min(1.,(self.now_s()-self.offset_t0)/self.offset_duration));fade=1-smoother(u)[0]
        out.x+=fade*self.reference_offset[0];out.y+=fade*self.reference_offset[1]
        self.last_private_reference=(out.x,out.y)
        self.pub_ref.publish(out)
        self.publish_angle_reference(msg)
    def publish_angle_reference(self,msg):
        motion=self.motion;theta=omega=0.;label='hold';p=v=a=0.
        if getattr(motion,'angle_motion',False):p,v,a,theta,omega,_=motion.state;label=motion.kind
        private=self.last_private_reference
        self.pub_virtual.publish(Point(x=float(private[0]+self.home[0]),y=float(private[1]+self.home[1]),z=float(msg.z+self.home[2])))
        for pub,value in ((self.pub_angle,theta*180/math.pi),(self.pub_angle_rate,omega*180/math.pi)):
            ref=Vector3Stamped();ref.header.stamp=self.get_clock().now().to_msg();ref.header.frame_id='world';ref.vector.y=value;pub.publish(ref)
        if self.now_s()-self.last_angle_output>=.1:
            self.last_angle_output=self.now_s()
            self.pub_angle_task.publish(String(data=json.dumps(dict(stamp_sec=self.now_s(),task=label,elapsed=motion.elapsed if motion else 0.,theta_north_deg=theta*180/math.pi,north_offset_m=p,north_velocity_m_s=v,north_acceleration_m_s2=a,TEMP_fixed_yaw=getattr(motion,'angle_motion',False)))))
    def install_motion(self,motion):
        old=self.last_equivalent_reference
        self.antiswing_enabled=True;self.motion=motion;self.stationary_since=None
        self.ref,_,_=motion.sample(self.now_s())
        candidate=self.controller_reference(Point(x=float(self.ref[0]),y=float(self.ref[1]),z=float(-self.ref[2])))
        self.reference_offset=tuple(old[i]-[candidate.x,candidate.y][i] for i in range(2)) if old is not None else (0.,0.)
        self.offset_t0=self.now_s();self.offset_duration=motion.duration if isinstance(motion,SmoothReference) else self.cfg['trajectory_ramp_sec']
    def stopped(self):
        return self.phase=='FLIGHT' and self.ready() and self.motion is None and self.stationary_since is not None and self.now_s()-self.stationary_since>=self.cfg['home_stable_sec']
    def can_disable_antiswing(self):
        return (self.phase=='IDLE' and not self.armed()) or self.stopped()
    def current_reference_state(self):
        if self.motion is None:return (tuple(self.ref),(0.,0.,0.),(0.,0.,0.))
        self.motion.sample(self.now_s())
        return reference_kinematics(self.motion,self.now_s())
    def build_bridge(self,kind,start,end_factory,*,source=None,after=None,next_motion=None,cruise=False,fixed_yaw=False):
        speed=self.cfg['transition_speed'] if cruise else max(self.cfg['transition_speed'],norm(self.vel()),float(source.meta.get('max_speed_m_s',0.)) if source is not None else 0.)
        angular=source is not None or getattr(self.motion,'angle_motion',False) or getattr(self.motion,'angle_stop',False)
        limit=self.angle_acceleration if angular else float(self.profiles[self.profile_name()]['max_comp_acc'])
        motion=plan_bridge(kind,self.now_s(),start,end_factory,speed=speed,accel=self.cfg['transition_acceleration'] if cruise else limit,ramp=self.cfg['trajectory_ramp_sec'],fixed_yaw=fixed_yaw or source is not None,source=source,after=after,next_motion=next_motion)
        if angular and not cruise and not getattr(motion,'angle_motion',False):motion.angle_stop=True
        return motion
    def stop_work(self,save,after):
        self.require_flight()
        if save and self.trajectory_paused:
            self.say('Work is already pausing/paused; anti-swing remains ON.');return
        current=self.motion;start=self.current_reference_state()
        angular=getattr(current,'angle_motion',False)
        source=current if angular else None
        actual=self.pos();velocity=self.vel()
        if angular:
            # Angular work has a deliberately free North position. Its nominal
            # position/velocity must not pull a stop back to the offline table.
            # Hand the unchanged anti-swing controller a measured-motion stop;
            # install_motion fades the equivalent private reference continuously.
            start=(tuple(actual),tuple(velocity),(0.,0.,0.))
        def end(T):
            p=tuple(actual[i]+.5*velocity[i]*T for i in range(3))
            return p,(0.,0.,0.),(0.,0.,0.)
        motion=self.build_bridge('braking',start,end,source=source,after=after,fixed_yaw=angular)
        if save and current is not None:
            if isinstance(current,SmoothReference) and current.next_motion is not None:self.saved_motion=current.next_motion
            else:self.saved_motion=current
            self.saved_ref=tuple(self.ref);self.trajectory_paused=True
        else:self.clear_paused_trajectory()
        self.install_motion(motion);self.task='pausing' if save else ('resetting' if after=='hold' else 'returning')
    def pause_trajectory(self):
        if self.task=='stopping':
            self.say('Angle task already canceled and stopping; anti-swing remains ON.');return
        if getattr(self.motion,'angle_motion',False):
            if isinstance(self.motion,AngleStopReference):
                self.say('Angle task already canceled and stopping; anti-swing remains ON.');return
            self.stop_work(save=False,after='hold');self.task='stopping'
            self.say('Angle task canceled. Smooth stop with anti-swing ON; home then sine/maneuver starts from the beginning. No angular resume record.')
            return
        self.stop_work(save=True,after='pause')
        self.say('Smooth stop started; progress saved and anti-swing ON. Wait until stopped before resume/control off.')
    def resume_trajectory(self):
        self.require_flight()
        target=self.saved_motion
        if target is None:raise ValueError('No saved work: reset/home or angular stop cleared the record')
        if getattr(target,'angle_motion',False):raise ValueError('Angle tasks restart from the beginning; use home, then sine/maneuver')
        start=self.current_reference_state();end=reference_kinematics(target,target.t0+target.elapsed)
        bridge=self.build_bridge('resuming',start,lambda T:end,next_motion=target)
        self.install_motion(bridge);self.task='resuming';self.trajectory_paused=False
        self.clear_paused_trajectory()
        self.say('Smoothly rejoining saved progress; anti-swing ON. Transition time does not consume remaining work.')
    def finish_motion(self,motion):
        if isinstance(motion,AngleReference):
            self.stop_work(save=False,after='home');self.say('Angle work complete. Smooth stop and return to takeoff point.');return
        if isinstance(motion,SmoothReference):
            if isinstance(motion,AngleStopReference) and motion.next_motion is None:
                # Model-based angle release is followed by a measured-motion
                # stop. Do not pull the aircraft toward a model-predicted point
                # when accumulated angle/model errors changed its translation.
                actual=tuple(self.pos());velocity=tuple(self.vel())
                end=lambda T:(tuple(actual[i]+velocity[i]*T/2 for i in range(3)),(0.,0.,0.),(0.,0.,0.))
                stop=self.build_bridge('braking',(actual,velocity,(0.,0.,0.)),end,after=motion.after,fixed_yaw=True)
                self.install_motion(stop)
                return
            if motion.next_motion is not None:
                self.motion=motion.next_motion;self.motion.resume(self.now_s());self.task=self.motion.kind
                self.reference_offset=(0.,0.)
                self.say('Saved work continues at the paused progress.');return
            self.motion=None
            if motion.after=='home':
                rest=(tuple(self.ref),(0.,0.,0.),(0.,0.,0.));target=(tuple(self.takeoff_point),(0.,0.,0.),(0.,0.,0.))
                self.install_motion(self.build_bridge('home',rest,lambda T:target,cruise=True,fixed_yaw=motion.fixed_yaw));self.task='home';return
            self.task='paused' if motion.after=='pause' and self.saved_motion is not None else 'hold'
            self.stationary_since=None;return
        self.motion=None;self.task='hold';self.stationary_since=None
        self.say('Reference finished. Holding its endpoint.')
    def pause(self,request_hold):
        interrupted=getattr(self.motion,'angle_motion',False)
        self.saved_motion=self.motion;self.saved_ref=self.ref;self.motion=None;self.trajectory_paused=False
        if self.task=='takeoff':self.saved_ref=(0.,0.,self.height)
        self.ref=self.pos();self.home_since=None;self.stop_backend()
        self.phase='PAUSING' if request_hold else 'PAUSED';self.last_mode_request=-1.
        if request_hold:self.command_px4(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.,4.,3.)
        if interrupted:
            self.clear_paused_trajectory();self.saved_ref=self.ref;self.task='hold'
        self.say('Task paused. PX4 Hold/manual takeover; resume re-enters from current pose.'+(' Angular task canceled.' if interrupted else ''))
    def tick(self):
        t=self.now_s()
        if self.last_clock is not None and t<self.last_clock-.001:
            self.stop_backend();self.phase='PAUSED' if self.armed() else 'IDLE';self.home=None;self.motion=None;self.clear_paused_trajectory()
            self.say('Simulation clock reset: restart the terminal and observers before a new flight.')
        self.last_clock=t
        for _ in range(30):
            try:line=self.command_queue.get_nowait()
            except queue.Empty:break
            try:self.handle_command(line)
            except (ValueError,OverflowError) as e:self.say('[REJECT] '+str(e))
        for _ in range(10):
            try:line=self.log_queue.get_nowait()
            except queue.Empty:break
            if any(x in line for x in ('ERROR','WARN','Takeoff Done','Mid-air Restart')):self.say('[backend] '+line)
        if self.shutdown_requested:return
        if t<=0:return
        if self.cfg['startup_mode']=='takeoff' and not self.auto_started and self.phase=='IDLE' and self.ready() and self.source_present():
            self.auto_started=True
            try:self.handle_command('takeoff')
            except ValueError as e:self.say('[REJECT] auto-start: '+str(e))
        if self.phase in ('FLIGHT','PAUSED','LANDING') and self.vehicle is not None and not self.armed():self.finish_disarmed()
        if self.phase in ('RECONFIGURE','PRESTREAM','TAKEOFF','FLIGHT') and not self.ready():
            self.pause(True)
        if self.phase=='RECONFIGURE':
            self.manual_hold()
            if self.retiring is None or self.retiring.poll() is not None:
                self.retiring=None;self.spawn_backend()
        elif self.phase=='PRESTREAM':
            self.manual_hold()
            if self.backend is None or self.backend.poll() is not None:self.pause(True)
            elif t-self.boot_time>self.cfg['handshake_timeout']:
                self.say('Offboard handshake timed out; no automatic retry.');self.pause(True)
        elif self.phase in ('TAKEOFF','FLIGHT'):
            if self.vehicle.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:self.offboard_seen=True
            elif self.offboard_seen and self.armed():self.pause(False)
            if self.phase in ('TAKEOFF','FLIGHT'):
                if self.backend is None or self.backend.poll() is not None or (self.backend_sp_time>=0 and t-self.backend_sp_time>self.cfg['backend_timeout']):
                    self.say('Controller output stopped; handing back to PX4 Hold.');self.pause(True)
                elif not self.offboard_seen and t-self.boot_time>self.cfg['handshake_timeout']:self.pause(True)
        elif self.phase=='PAUSING':
            if self.vehicle is not None and self.vehicle.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.phase='PAUSED' if self.armed() else 'IDLE'
            else:
                self.manual_hold()
                if t-self.last_mode_request>.5:
                    self.command_px4(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.,4.,3.);self.last_mode_request=t
        elif self.phase=='LANDING':
            if not self.armed():self.finish_disarmed()
            elif self.vehicle.nav_state!=VehicleStatus.NAVIGATION_STATE_AUTO_LAND and t-self.last_mode_request>.5:
                self.command_px4(VehicleCommand.VEHICLE_CMD_NAV_LAND);self.last_mode_request=t
        # Task-level handshake: do not rely on a one-shot backend command
        # reaching a newly discovered DDS subscription. Never retry after takeover.
        if self.phase in ('PRESTREAM','TAKEOFF') and self.backend_receives() and self.backend_sp_time>=self.boot_time and t-self.last_mode_request>.5:
            if self.vehicle.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.command_px4(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.,6.)
            elif self.reason=='takeoff' and not self.armed():
                self.command_px4(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.)
            self.last_mode_request=t
        if self.phase=='FLIGHT':
            if self.motion is not None:
                self.ref,v,done=self.motion.sample(t)
                if math.hypot(v[0],v[1])>1e-5:self.yaw=math.atan2(v[1],v[0])
                if done:self.finish_motion(self.motion)
            self.publish_reference(Point(x=float(self.ref[0]),y=float(self.ref[1]),z=float(-self.ref[2])))
            self.pub_yaw.publish(Float32(data=float(self.yaw)))
            p=Point(x=float(self.home[0]+self.ref[0]),y=float(self.home[1]+self.ref[1]),z=float(self.home[2]-self.ref[2]))
            self.pub_plot.publish(p);self.pub_plot_yaw.publish(Float32(data=float(self.yaw)))
            if self.motion is None and at_home(self.pos(),self.vel(),self.height,self.cfg['home_position_tolerance'],self.cfg['home_speed_tolerance']):
                if self.home_since is None:self.home_since=t
            else:self.home_since=None
        elif self.phase=='PAUSED' and self.ready() and at_home(self.pos(),self.vel(),self.height,self.cfg['home_position_tolerance'],self.cfg['home_speed_tolerance']):
            if self.home_since is None:self.home_since=t
        else:self.home_since=None
        stable=self.phase=='FLIGHT' and self.motion is None and self.ready() and norm(self.vel())<=self.cfg['home_speed_tolerance'] and norm(tuple(a-b for a,b in zip(self.pos(),self.ref)))<=self.cfg['home_position_tolerance']
        if stable:
            if self.stationary_since is None:self.stationary_since=t
        else:self.stationary_since=None
        self.pub_active.publish(Bool(data=self.phase=='FLIGHT' and self.antiswing_enabled))
        if t-self.last_status_output>=.2:self.publish_status();self.last_status_output=t
    def close(self):
        self.reader_stop.set()
        if self.reader:self.reader.join(timeout=.3)
        if self.cfg['command_fifo'] and self.reader_fd is not None:os.close(self.reader_fd)
        self.stop_backend()
        if self.retiring is not None:
            try:self.retiring.wait(timeout=3)
            except subprocess.TimeoutExpired:self.retiring.kill();self.retiring.wait()


def main():
    rclpy.init();node=None;executor=SingleThreadedExecutor()
    try:
        node=Offboard();executor.add_node(node)
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close()
        executor.shutdown()
        if node:node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
