#!/usr/bin/env python3
"""An experiment-only bidirectional UDP relay with explicit packet dropping."""
import argparse
import json
import selectors
import socket
import threading
import time
from pathlib import Path
import rospy
from std_srvs.srv import SetBool, SetBoolResponse


class Relay:
    def __init__(self, fcu_port=14540, client_port=14542, mavros_port=14541):
        self.fcu=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.client=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.fcu.bind(('127.0.0.1',fcu_port))
        self.client.bind(('127.0.0.1',client_port))
        self.fcu.setblocking(False);self.client.setblocking(False)
        self.mavros_address=('127.0.0.1',mavros_port)
        self.fcu_address=None
        self.drop=False;self.alive=True
        self.stats={'fcu_to_mavros_forwarded':0,'mavros_to_fcu_forwarded':0,
                    'fcu_to_mavros_dropped':0,'mavros_to_fcu_dropped':0}
        self.events=[]
        self.worker=threading.Thread(target=self.run,daemon=True)
        self.worker.start()

    def set_drop(self, drop):
        self.drop=bool(drop)
        self.events.append({'wall_monotonic':time.monotonic(),'drop':self.drop,'stats':dict(self.stats)})

    def run(self):
        with selectors.DefaultSelector() as selector:
            selector.register(self.fcu,selectors.EVENT_READ,'fcu')
            selector.register(self.client,selectors.EVENT_READ,'mavros')
            while self.alive:
                for key,_ in selector.select(timeout=.2):
                    try:packet,source=key.fileobj.recvfrom(65535)
                    except BlockingIOError:continue
                    if key.data=='fcu':
                        self.fcu_address=source
                        label='fcu_to_mavros'
                        if not self.drop:self.client.sendto(packet,self.mavros_address)
                    else:
                        label='mavros_to_fcu'
                        if not self.drop and self.fcu_address is not None:
                            self.fcu.sendto(packet,self.fcu_address)
                        elif not self.drop:
                            # PX4 has not yet announced its source address.
                            self.stats[label+'_dropped']+=1
                            continue
                    self.stats[label+('_dropped' if self.drop else '_forwarded')]+=1

    def close(self):
        self.alive=False;self.worker.join(timeout=2)
        self.fcu.close();self.client.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output');args=parser.parse_args()
    relay=Relay()
    rospy.init_node('mavlink_fault_relay',anonymous=False,disable_signals=True)
    def request(msg):
        relay.set_drop(msg.data)
        return SetBoolResponse(success=True,message='Packets dropped' if msg.data else 'Relay restored')
    rospy.Service('/drone/sim/faults/mavlink_drop',SetBool,request)
    try:
        while not rospy.is_shutdown():time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        relay.close()
        Path(args.output).write_text(json.dumps({'stats':relay.stats,'events':relay.events},indent=2)+'\n')


if __name__=='__main__':main()
