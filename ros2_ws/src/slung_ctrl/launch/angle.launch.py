# Codex-generated EXPERIMENTAL angle-task launcher. Baseline entry point is unchanged.
# Like sim.launch.py, this optional entry uses the project checkout via project_root.
# Humble ExecuteProcess does not forward stdin itself: use an explicitly owned FIFO.
import os
import sys
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit, OnShutdown, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def setup(context):
    folder=tempfile.TemporaryDirectory(prefix='slung_offboard_cli_')
    fifo=Path(folder.name)/'commands';os.mkfifo(fifo,0o600)
    writer=os.open(fifo,os.O_RDWR|os.O_NONBLOCK)
    loop=context.asyncio_loop
    state={'reader':False,'closed':False}
    script=Path(LaunchConfiguration('project_root').perform(context)).expanduser()/'ros2_ws/src/slung_ctrl/scripts/angle_console.py'
    if not script.is_file():raise RuntimeError('Experimental source entry not found: '+str(script))
    node=ExecuteProcess(cmd=[sys.executable,str(script),'--ros-args',
              '--params-file',LaunchConfiguration('params_file').perform(context),
              '-p','use_sim_time:='+LaunchConfiguration('use_sim_time').perform(context),
              '-p','command_fifo:='+str(fifo)],output='log',emulate_tty=False)
    # Relay raw console bytes instead of ROS launch's line-prefixed screen output.
    # Keep normal launch log files, while showing the prompt immediately.
    def display_stdout(event):
        sys.stdout.write(event.text.decode(errors='replace'));sys.stdout.flush()
    def display_stderr(event):
        sys.stderr.write(event.text.decode(errors='replace'));sys.stderr.flush()
    def forward():
        try:
            data=os.read(sys.stdin.fileno(),4096)
            if data:os.write(writer,data)
            elif state['reader']:
                loop.remove_reader(sys.stdin.fileno());state['reader']=False
        except (OSError,ValueError):
            if state['reader']:loop.remove_reader(sys.stdin.fileno());state['reader']=False
    def started(event,ctx):
        try:
            loop.add_reader(sys.stdin.fileno(),forward);state['reader']=True
        except (OSError,ValueError,PermissionError):
            print('[OFFBOARD] stdin unavailable; use /uav/offboard/command or launch from an interactive terminal.')
    def close(event,ctx):
        if state['closed']:return
        state['closed']=True
        if state['reader']:loop.remove_reader(sys.stdin.fileno())
        os.close(writer);folder.cleanup()
    return [RegisterEventHandler(OnProcessIO(target_action=node,on_stdout=display_stdout,on_stderr=display_stderr)),
            RegisterEventHandler(OnProcessStart(target_action=node,on_start=started)),
            RegisterEventHandler(OnShutdown(on_shutdown=close)),
            RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason='Offboard terminal exited'))])),node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root',default_value=os.environ.get('UAV_SLUNG_ROOT',os.path.expanduser('~/UAV_slung'))),
        DeclareLaunchArgument('params_file',default_value=os.path.join(get_package_share_directory('slung_ctrl'),'config','offboard.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='true',choices=['true','false']),
        OpaqueFunction(function=setup)])
