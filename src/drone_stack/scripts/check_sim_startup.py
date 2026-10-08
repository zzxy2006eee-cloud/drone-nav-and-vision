#!/usr/bin/env python3
"""Observe startup data without arming, changing modes or stopping flight nodes."""
import argparse
import json
import math
import threading
import time
from pathlib import Path

import rospy
import rostopic
import rosgraph


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['mapping', 'inspection'], required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=600., help='Wall seconds after spawn request submission')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rospy.init_node('drone_startup_check', anonymous=True)
    topics = [
        '/clock', '/livox/lidar', '/livox/imu', '/cloud_registered',
        '/drone/lio/odom', '/mavros/odometry/out', '/mavros/local_position/odom',
        '/mavros/state', '/drone/mapping/rays', '/grid_map/voxel_delta',
        '/drone/front/image_processed', '/drone/down/image_processed',
        '/drone/manager_heartbeat', '/drone/flight_state', '/drone/flight_health',
        '/drone/map_alignment_ready', '/drone/map_session_status',
        '/drone/inspection/status',
        '/drone/ai/status',
    ]
    latched = {'/drone/map_alignment_ready', '/drone/map_session_status'}
    lock = threading.Lock()
    records, subscribers = {}, {}

    def receive(msg, topic):
        valid = True
        stamp = None
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.to_sec()
            valid = math.isfinite(stamp) and stamp > 0
        if hasattr(msg, 'clock'):
            stamp = msg.clock.to_sec()
            valid = math.isfinite(stamp) and stamp > 0
        if topic == '/drone/manager_heartbeat':
            stamp = msg.stamp.to_sec()
            valid = math.isfinite(stamp) and stamp > 0
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            valid &= all(math.isfinite(x) for x in values) and abs(sum(x*x for x in values[3:])-1) < .05
        if hasattr(msg, 'points'):
            valid &= len(msg.points) > 0
            if hasattr(msg, 'point_num'):
                valid &= msg.point_num == len(msg.points)
        if hasattr(msg, 'width'):
            valid &= msg.width > 0 and msg.height > 0 and bool(msg.data)
            if hasattr(msg, 'fields'):
                valid &= {'x', 'y', 'z'} <= {f.name for f in msg.fields}
                valid &= msg.point_step > 0 and msg.row_step >= msg.width*msg.point_step and len(msg.data) >= msg.row_step*msg.height
        if topic == '/grid_map/voxel_delta':
            valid &= math.isfinite(msg.resolution) and msg.resolution > 0 and all(x > 0 for x in msg.shape)
        if topic == '/livox/imu':
            valid &= all(math.isfinite(x) for v in [msg.angular_velocity, msg.linear_acceleration] for x in [v.x, v.y, v.z])
        value = None
        if topic == '/mavros/state': value = msg.connected
        elif hasattr(msg, 'data') and topic in latched | {'/drone/flight_state', '/drone/flight_health'}:
            value = msg.data
        with lock:
            old = records.get(topic, {})
            records[topic] = dict(count=old.get('count', 0)+1, wall=time.monotonic(),
                                  valid=bool(valid), stamp=stamp, value=value,
                                  advances=old.get('advances', 0)+int(stamp is not None and stamp != old.get('stamp')))

    started = None
    previous_report = 0.
    stable_since = None
    print('等待出生位姿确认和仿真时钟；巡检模式在 Qt 操作，等待期间不计算初始化超时。', flush=True)
    while not rospy.is_shutdown():
        for topic in topics:
            if topic not in subscribers:
                try:
                    cls, _, _ = rostopic.get_topic_class(topic, blocking=False)
                    if cls is not None:
                        subscribers[topic] = rospy.Subscriber(topic, cls, receive, callback_args=topic, queue_size=1)
                except Exception:
                    pass
        now = time.monotonic()
        with lock:
            snapshot = {k: dict(v) for k, v in records.items()}
        clock = snapshot.get('/clock', {})
        if started is None and rospy.has_param('/drone/simulation_spawn_request'):
            started = now
        problems = []
        sim_now = clock.get('stamp')
        for topic in topics:
            r = snapshot.get(topic)
            if r is None:
                problems.append(topic+': 未收到数据')
                continue
            if not r['valid']:
                problems.append(topic+': 内容无效')
            if topic not in latched and (now-r['wall'] > 5 or r['count'] < 2):
                problems.append(topic+': 断流或尚未连续接收')
            if topic not in latched and r['stamp'] is not None and sim_now is not None and topic != '/clock':
                if not -.05 <= sim_now-r['stamp'] <= 2.:
                    problems.append(topic+': 采集时间过期或超前')
                if r['advances'] < 2:
                    problems.append(topic+': 采集时间未推进')
        for topic, expected in [('/mavros/state', True), ('/drone/flight_state', 'READY'),
                                ('/drone/flight_health', 'HEALTHY'), ('/drone/map_alignment_ready', True)]:
            if snapshot.get(topic, {}).get('value') != expected:
                problems.append(topic+': 未达到 '+str(expected))
        try:
            session = json.loads(snapshot.get('/drone/map_session_status', {}).get('value') or '{}')
        except (ValueError, TypeError):
            session = {}
        expected_mode = 'PRIOR_NAV' if args.mode == 'inspection' else 'ONLINE'
        if session.get('mode') != expected_mode or not session.get('ready'):
            problems.append('地图会话未就绪或与启动模式不一致')
        try:
            master = rosgraph.Master('/drone_startup_check')
            master.lookupService('/planning/start')
            nodes = {n for section in master.getSystemState() for _, names in section for n in names}
            if '/ego_planner_node' not in nodes or '/drone_operator_gui' not in nodes:
                problems.append('EGO 或 Qt 节点未注册')
        except Exception:
            problems.append('EGO /planning/start 服务未就绪')
        if problems:
            stable_since = None
        elif stable_since is None:
            stable_since = now
        passed = stable_since is not None and now-stable_since >= 3
        failed = started is not None and now-started > args.timeout
        report = dict(mode=args.mode, passed=passed, timed_out=failed, problems=problems,
                      wall_elapsed=None if started is None else now-started, sim_time=sim_now,
                      topics=snapshot)
        if passed or failed or now-previous_report >= 5:
            path = Path(args.output)
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.tmp')
            temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            temporary.replace(path)
            rospy.set_param('/drone/startup_check_passed', passed)
            print('启动数据检查通过，持续正常 3 秒。' if passed else '等待：'+'；'.join(problems), flush=True)
            previous_report = now
        if passed:
            return 0
        if failed:
            print('启动检查超时；保留定位和控制程序，查看 JSON 与 stack.log。检查器不会自动飞行或中止飞行。', flush=True)
            return 1
        time.sleep(.2)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
