#!/usr/bin/env python3
"""Capture visual and accessibility evidence of an actual fault condition."""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
import rospy
from std_msgs.msg import Bool,Header,String
from mavros_msgs.msg import State
from sensor_msgs.msg import Image


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trigger',choices=['lio_invalid','manager_stale','fcu_stale','camera_stale'])
    parser.add_argument('output');args=parser.parse_args()
    output=Path(args.output);data={};receipt={}
    rospy.init_node('live_gui_observer',anonymous=True)
    def callback(key,msg):
        data[key]=msg;receipt[key]=time.monotonic()
    subscriptions=[]
    for topic,typ,key in [('/drone/lio/valid',Bool,'lio'),('/drone/manager_heartbeat',Header,'manager'),
                         ('/mavros/state',State,'fcu'),('/drone/flight_state',String,'phase'),
                         ('/drone/front/image_processed',Image,'front'),('/drone/down/image_processed',Image,'down')]:
        subscriptions.append(rospy.Subscriber(topic,typ,lambda m,k=key:callback(k,m),queue_size=1,tcp_nodelay=True))
    deadline=time.monotonic()+80;triggered=False
    while time.monotonic()<deadline:
        now=time.monotonic()
        triggered=(args.trigger=='lio_invalid' and 'lio' in data and not data['lio'].data or
                   args.trigger=='manager_stale' and 'manager' in receipt and now-receipt['manager']>1.0 or
                   args.trigger=='fcu_stale' and 'fcu' in receipt and now-receipt['fcu']>2.3 or
                   args.trigger=='camera_stale' and all(k in receipt and now-receipt[k]>1.5 for k in ['front','down']))
        if triggered:break
        time.sleep(.05)
    report={'trigger':args.trigger,'trigger_observed':triggered,'captured_wall_time':time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    if triggered:
        time.sleep(.4)
        subprocess.run(['gnome-screenshot','-f',str(output.with_suffix('.png'))],check=True,timeout=10)
        check={'lio_invalid':'protection','manager_stale':'protection','fcu_stale':'fcu_stale','camera_stale':'camera_stale'}[args.trigger]
        code=subprocess.call([sys.executable,str(Path(__file__).with_name('gui_snapshot.py')),
            str(output.with_suffix('.accessibility.json')),'--check',check],timeout=15)
        report['accessibility_check_passed']=code==0
        report['limits']='Image needs visual review; accessibility check reports actual Qt widget flags.'
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if triggered and report.get('accessibility_check_passed') else 1)


if __name__=='__main__':main()
