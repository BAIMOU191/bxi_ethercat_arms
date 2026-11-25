"""Run after sourcing the built overlay; launches an isolated mock node, never hardware."""
import os, shutil, signal, subprocess, sys, tempfile, time
from pathlib import Path
os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='197', ROS2CLI_NO_DAEMON='1')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from communication.msg import ActuatorCmds, ActuatorStates
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from arms_ros.srv import JointOperation

rclpy.init(); test=rclpy.create_node('arms_smoke'); ns='/arms_test_'+str(os.getpid())
seen={}; subs=[]
for cls,key in [(JointState,'joint_states'),(ActuatorStates,'actuator_states'),(DiagnosticArray,'arms/diagnostics')]:
    subs.append(test.create_subscription(cls,ns+'/hardware/'+key,lambda msg,k=key:seen.update({k:msg}),qos_profile_sensor_data))
pub=test.create_publisher(ActuatorCmds,ns+'/hardware/actuators_cmds',qos_profile_sensor_data)
def until(predicate):
    end=time.monotonic()+12
    while not predicate():
        assert time.monotonic()<end,'ROS condition timed out'
        rclpy.spin_once(test,timeout_sec=.04)
def call(name,confirm=False,joint=21):
    cli=test.create_client(JointOperation,ns+'/arms/'+name)
    assert cli.wait_for_service(timeout_sec=12),name
    q=JointOperation.Request();q.joints=1<<joint;q.confirm=confirm
    f=cli.call_async(q);until(f.done);r=f.result();test.destroy_client(cli);return r
try:
    config_dir = tempfile.TemporaryDirectory()
    config = Path(config_dir.name) / 'arms.json'
    shutil.copyfile(sys.argv[1], config)
    cmd=['ros2','run','arms_ros','arms_node','--ros-args','-r','__ns:='+ns,'-p','mock:=true','-p','config:='+str(config)]
    proc=subprocess.Popen(cmd,start_new_session=True, env=dict(os.environ, ARMS_MOCK_FAULT_CYCLE="1"))
    try:
        until(lambda:len(seen)==3);assert len(seen['joint_states'].name)==14
        assert seen['arms/diagnostics'].status[0].level in (0,b'\x00'),seen['arms/diagnostics']
        assert call('selftest').success and not call('enable').success
        # 抱闸模式需要确认；enable / free 会把强制松闸、强制抱闸自动切回正常控制
        assert not call('brake_release').success
        assert call('brake_release',True).success
        def after_disable(name):
            # disable 立即返回；强制抱闸要求关节已失能，等反馈确认后再设置
            end=time.monotonic()+3
            while not call(name,True).success:
                assert time.monotonic()<end,name
                time.sleep(.05)
        assert call('enable',True).success and call('disable').success
        after_disable('brake_hold')
        assert call('free',True).success and call('disable').success
        after_disable('brake_hold')
        assert call('brake_normal',True).success
        # 零力矩拖动会使能关节
        assert call('free',True).success
        # free 后校零与已失能后校零共用驱动路径；ROS 的授权字段保持兼容。
        assert not call('zero').success
        assert call('zero',True).success and call('zero',True).success
        assert call('free',True).success
        # 使能中拒绝强制松闸
        assert not call('brake_release',True).success
        m=ActuatorCmds();m.actuators_name=['l_wrist_z_joint'];m.pos=[.1];m.vel=[0.];m.kp=[5.];m.kd=[0.];m.torque=[0.]
        def moved():
            m.header.stamp=test.get_clock().now().to_msg();pub.publish(m)
            s=seen['joint_states'];return abs(s.position[s.name.index('l_wrist_z_joint')]-.1)<.02
        until(moved)
        m.pos=[];pub.publish(m)
        until(lambda:any(k.key=='rejected_commands' and int(k.value)>0 for k in seen['arms/diagnostics'].status[0].values))
        assert call('disable').success
        # 注入 15 号驱动器故障：WKC 正常也不能把这条臂报告为 online。
        call('enable', True, joint=15)
        until(lambda: any(d.message == 'drive fault' and any(
            kv.key == 'fault_joints' and int(kv.value) & (1 << 15) for kv in d.values
        ) for d in seen['arms/diagnostics'].status))
    finally:
        if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        proc.wait(timeout=15)
        config_dir.cleanup()
finally:
    test.destroy_node();rclpy.shutdown()
print('PASS: ROS feedback, diagnostics, validated commands, explicit enable and brake interlocks')
