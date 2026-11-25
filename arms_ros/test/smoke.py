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
def call(name, *, joint=21):
    cli=test.create_client(JointOperation,ns+'/arms/'+name)
    assert cli.wait_for_service(timeout_sec=12),name
    q=JointOperation.Request(joints=1<<joint)
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
        assert set(JointOperation.Request.get_fields_and_field_types()) == {'joints', 'hardware_zero'}
        assert call('selftest').success and call('reset').success
        assert not call('enable', joint=0).success  # 未打开的关节仍被拒绝
        # 非 normal 期间仍拒绝使能和零力矩拖动
        assert call('brake_release').success
        assert not call('enable').success and not call('free').success
        assert call('brake_hold').success
        assert not call('enable').success
        assert call('brake_normal').success
        # 零力矩拖动会使能关节
        assert call('free').success
        # free 后校零与已失能后校零共用驱动路径。
        assert call('zero').success and call('zero').success
        assert call('enable').success
        # 已使能时切换 Pn075 不失能；恢复 normal 后无需再次 enable 就能接受后续目标。
        for service in ('brake_release', 'brake_hold', 'brake_release', 'brake_normal'):
            assert call(service).success, service
        m=ActuatorCmds();m.actuators_name=['l_wrist_z_joint'];m.pos=[.1];m.vel=[0.];m.kp=[5.];m.kd=[0.];m.torque=[0.]
        def moved():
            m.header.stamp=test.get_clock().now().to_msg();pub.publish(m)
            s=seen['joint_states'];return abs(s.position[s.name.index('l_wrist_z_joint')]-.1)<.02
        until(moved)
        def rejected_count():
            return int(next(k.value for k in seen['arms/diagnostics'].status[0].values if k.key == 'rejected_commands'))
        old = rejected_count()
        m.pos=[]; m.header.stamp=test.get_clock().now().to_msg(); pub.publish(m)
        until(lambda: rejected_count() > old)
        # 两个已使能关节：第一项合法、第二项超限时，第一项也不能被下发。
        assert call('enable', joint=22).success
        m.actuators_name=['l_wrist_z_joint','r_shoulder_y_joint']; m.pos=[-.1,1e6]
        m.vel=[0.,0.]; m.kp=[5.,5.]; m.kd=[0.,0.]; m.torque=[0.,0.]
        old = rejected_count(); m.header.stamp=test.get_clock().now().to_msg(); pub.publish(m)
        until(lambda: rejected_count() > old)
        state=seen['joint_states']
        assert abs(state.position[state.name.index('l_wrist_z_joint')]-.1)<.02
        assert call('disable', joint=22).success
        assert call('disable').success
        # 注入 15 号驱动器故障：WKC 正常也不能把这条臂报告为 online。
        call('enable', joint=15)
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
