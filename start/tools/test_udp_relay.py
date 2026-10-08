#!/usr/bin/env python3
import socket
import time
import unittest
from mavlink_udp_relay import Relay


class UDPChecks(unittest.TestCase):
    def setUp(self):
        self.vehicle=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.vehicle.bind(('127.0.0.1',0));self.vehicle.settimeout(.4)
        self.client=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.client.bind(('127.0.0.1',0));self.client.settimeout(.4)
        self.relay=Relay(fcu_port=0,client_port=0,mavros_port=self.client.getsockname()[1])
        self.addCleanup(self.relay.close)
        self.addCleanup(self.client.close);self.addCleanup(self.vehicle.close)

    def test_round_trip_payload_and_address_preserved(self):
        self.vehicle.sendto(b'heartbeat',self.relay.fcu.getsockname())
        payload,source=self.client.recvfrom(100)
        self.assertEqual(payload,b'heartbeat')
        self.assertEqual(source,self.relay.client.getsockname())
        self.client.sendto(b'odometry_and_target',source)
        payload,source=self.vehicle.recvfrom(100)
        self.assertEqual(payload,b'odometry_and_target')
        self.assertEqual(source,self.relay.fcu.getsockname())

    def test_drops_are_drained_and_not_replayed_on_restore(self):
        self.relay.set_drop(True)
        self.vehicle.sendto(b'stale_feedback',self.relay.fcu.getsockname())
        self.client.sendto(b'stale_target',self.relay.client.getsockname())
        deadline=time.monotonic()+2
        while time.monotonic()<deadline and sum(self.relay.stats.values())<2:time.sleep(.01)
        self.assertEqual(self.relay.stats['fcu_to_mavros_dropped'],1)
        self.assertEqual(self.relay.stats['mavros_to_fcu_dropped'],1)
        self.relay.set_drop(False)
        self.vehicle.sendto(b'new_feedback',self.relay.fcu.getsockname())
        self.client.sendto(b'new_target',self.relay.client.getsockname())
        self.assertEqual(self.client.recvfrom(100)[0],b'new_feedback')
        self.assertEqual(self.vehicle.recvfrom(100)[0],b'new_target')
        with self.assertRaises(socket.timeout):self.client.recvfrom(100)
        with self.assertRaises(socket.timeout):self.vehicle.recvfrom(100)


if __name__=='__main__':unittest.main(verbosity=2)
