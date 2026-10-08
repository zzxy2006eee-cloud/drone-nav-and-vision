#!/usr/bin/env python3
"""Record launcher identity and stop only this workspace's simulation processes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parent.parent
REGISTRY = Path('/tmp') / ('drone_processes_%d_%s' % (
    os.getuid(), hashlib.sha256(str(ROOT).encode()).hexdigest()[:16]))


def identity(pid):
    try:
        path = Path('/proc') / str(pid)
        if path.stat().st_uid != os.getuid():
            return None
        fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return int(fields[1]), fields[19]  # parent PID, start time ticks
    except (OSError, ValueError, IndexError):
        return None


def excluded_pids():
    result = set()
    pid = os.getpid()
    while pid > 0 and pid not in result:
        result.add(pid)
        entry = identity(pid)
        pid = entry[0] if entry else 0
    return result


def discover(known=None):
    excluded = excluded_pids()
    processes, owned = {}, set()
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        pid = int(path.name)
        entry = identity(pid)
        if entry is None or pid in excluded:
            continue
        processes[pid] = entry
        try:
            environment = (path / 'environ').read_bytes().split(b'\0')
            if ('DRONE_PROCESS_ROOT='+str(ROOT)).encode() in environment:
                owned.add(pid)
                continue
            argv = [v.decode(errors='replace') for v in (path / 'cmdline').read_bytes().split(b'\0') if v]
            cwd = (path / 'cwd').resolve()
            # Also recognize manually launched project binaries and older launchers.
            if any(v.startswith(str(ROOT / 'devel/lib')+'/') for v in argv):
                owned.add(pid)
            elif argv and Path(argv[0]).name in ('gzserver', 'gzclient', 'px4') and (
                    cwd == ROOT or ROOT in cwd.parents or any(v.startswith(str(ROOT)+'/') for v in argv[1:])):
                owned.add(pid)
            else:
                scripts = {ROOT/'start/start_gazebo.sh', ROOT/'start/start_navigation.sh',
                           ROOT/'src/drone_stack/scripts/start_simulation.sh',
                           ROOT/'src/drone_stack/scripts/start_px4_sim.sh'}
                if any((cwd/v).resolve() in scripts for v in argv):
                    owned.add(pid)
                elif any(Path(arg).name == 'roslaunch' for arg in argv):
                    if 'drone_stack' in argv and 'stack.launch' in argv:
                        owned.add(pid)
        except (OSError, ValueError):
            pass
    if REGISTRY.exists():
        for file in REGISTRY.glob('*.json'):
            try:
                record = json.loads(file.read_text())
                pid = int(record['pid'])
                if pid not in excluded and processes.get(pid, (None, None))[1] == record['start']:
                    owned.add(pid)
                else:
                    file.unlink(missing_ok=True)
            except (OSError, ValueError, KeyError, TypeError):
                pass
    for pid, saved in (known or {}).items():
        if pid not in excluded and processes.get(pid, (None, None))[1] == saved[1]:
            owned.add(pid)
    # Include descendants even if they changed their environment.
    while True:
        children = {pid for pid, (parent, _) in processes.items() if parent in owned}
        if children <= owned:
            break
        owned |= children
    return {pid: processes[pid] for pid in owned}


def stop():
    known = discover()
    if not known:
        print('未发现本项目仿真进程。')
        return 0
    for sig, timeout in [(signal.SIGINT, 3.), (signal.SIGTERM, 3.), (signal.SIGKILL, 2.)]:
        targets = discover(known)
        known.update(targets)
        if not targets:
            break
        print('发送 %s，清理 %d 个项目进程。' % (sig.name, len(targets)), flush=True)
        for pid, saved in targets.items():
            current_identity = identity(pid)
            if current_identity is None or current_identity[1] != saved[1]:
                continue
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                print('无法停止 PID %d：权限不足。' % pid, flush=True)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            current = discover(known)
            known.update(current)
            if not current:
                break
            time.sleep(.2)
    remaining = discover(known)
    if remaining:
        print('仍有进程未退出：'+', '.join(map(str, sorted(remaining))))
        return 1
    print('本项目 Qt、ROS、PX4、Gazebo 及其后台子进程已停止。')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['register', 'unregister', 'stop'])
    parser.add_argument('pid', nargs='?', type=int)
    args = parser.parse_args()
    if args.action == 'stop':
        return stop()
    if args.pid is None:
        parser.error('register/unregister requires PID')
    if args.action == 'unregister':
        (REGISTRY/('%d.json' % args.pid)).unlink(missing_ok=True)
        return 0
    entry = identity(args.pid)
    if entry is None:
        return 1
    REGISTRY.mkdir(mode=0o700, parents=True, exist_ok=True)
    (REGISTRY/('%d.json' % args.pid)).write_text(json.dumps(dict(pid=args.pid, start=entry[1])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
