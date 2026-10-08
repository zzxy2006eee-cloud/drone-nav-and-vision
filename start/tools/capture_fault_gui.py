#!/usr/bin/env python3
"""Capture the operator screen when an actual flight protection state occurs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import rospy
from std_msgs.msg import String, Bool


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output');p.add_argument('--timeout-s',type=float,default=240);a=p.parse_args()
    output=Path(a.output).resolve();output.parent.mkdir(parents=True,exist_ok=True)
    rospy.init_node('gui_fault_capture',anonymous=True);state={}
    for topic,typ,key in [('/drone/flight_state',String,'phase'),('/drone/flight_error',String,'error'),('/drone/lio/valid',Bool,'lio_valid')]:
        rospy.Subscriber(topic,typ,lambda m,k=key:state.__setitem__(k,m.data),queue_size=1,tcp_nodelay=True)
    deadline=time.monotonic()+a.timeout_s
    while time.monotonic()<deadline and not rospy.is_shutdown():
        if state.get('phase') in ('LANDING','DESCENDING','FAILSAFE'):
            time.sleep(.6)
            subprocess.run(['gnome-screenshot','-f',str(output)],check=True,timeout=10)
            report={'captured_wall_time':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'sim_time':rospy.Time.now().to_sec(),
                    'observed':dict(state),'screenshot':str(output),
                    'observer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    'limits':'Screenshot captured; GUI button/text correctness needs visual review.'}
            output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return
        time.sleep(.05)
    raise SystemExit('No actual flight protection state observed before timeout')


if __name__=='__main__':main()
