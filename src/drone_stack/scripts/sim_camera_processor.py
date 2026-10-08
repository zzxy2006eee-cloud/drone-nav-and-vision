#!/usr/bin/env python3
"""Colour-target processing for simulation pipeline validation only.

Publishes annotated images and pixel detections, without inventing 3D
coordinates or claiming to recognise real inspection defects.
"""
import json
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Processor:
    def __init__(self):
        self.bridge = CvBridge()
        self.min_area = rospy.get_param('~min_area_pixels', 150)
        self.publishers = {}
        for name in ('front', 'down'):
            base = '/drone/' + name
            self.publishers[name] = (
                rospy.Publisher(base + '/image_processed', Image, queue_size=1),
                rospy.Publisher(base + '/detections', String, queue_size=1))
            rospy.Subscriber(base + '/image_raw', Image, self.process,
                             callback_args=name, queue_size=1, buff_size=2**22)

    def process(self, msg, name):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = (cv2.inRange(hsv, np.array([0, 100, 60]), np.array([15, 255, 255])) |
                    cv2.inRange(hsv, np.array([165, 100, 60]), np.array([179, 255, 255])))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections = []
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({'label': 'sim_red_target', 'bbox_xywh': [x, y, w, h], 'area_pixels': area})
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 255), 2)
                cv2.putText(img, 'SIM target', (x, max(20, y-5)), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
            cv2.putText(img, 'SIM targets: %d' % len(detections), (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
            out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header = msg.header
            self.publishers[name][0].publish(out)
            self.publishers[name][1].publish(String(data=json.dumps({
                'stamp': msg.header.stamp.to_sec(), 'frame_id': msg.header.frame_id,
                'image_size': [msg.width, msg.height], 'detections': detections})))
        except (CvBridgeError, cv2.error) as exc:
            rospy.logerr_throttle(2, 'Camera processing failed: %s', exc)


if __name__ == '__main__':
    rospy.init_node('sim_camera_processor')
    Processor()
    rospy.spin()
