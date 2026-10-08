#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import os
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from PIL import ImageFont, ImageDraw
import PIL.Image as PILImage

# ==================== 用户可修改的参数 ====================
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_1000
YOLO_MODEL_PATH = "/home/d/robotproject/project0/src/controller/scripts/model.pt"
YOLO_CONF_THRESH = 0.5
OVERLAP_THRESH = 0.7
POSITION_DUP_THRESH = 1.0
OUTPUT_FILE = "obstacle_record.txt"
IMAGE_TOPIC = "/usb_cam/image_raw"
ODOM_TOPIC = "/mavros/local_position/odom"

# 中文字体路径
FONT_PATH = "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"
if not os.path.exists(FONT_PATH):
    FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
# =========================================================

class VisionMonitor:
    def __init__(self):
        rospy.init_node('vision_monitor', anonymous=True)

        # 初始化 ArUco
        cv_version = cv2.__version__
        major, minor = map(int, cv_version.split('.')[:2])
        if major >= 4 and minor >= 7:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_TYPE)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 加载YOLO
        if not os.path.exists(YOLO_MODEL_PATH):
            rospy.logwarn("YOLO模型不存在，使用默认模型")
            model_path = "yolov8n.pt"
        else:
            model_path = YOLO_MODEL_PATH
        self.yolo = YOLO(model_path)
        rospy.loginfo("YOLO模型加载完成")

        # 状态机
        self.state = "QR"
        self.qr_detected = False
        self.frozen_frame = None
        self.switch_time = None

        self.recorded_objects = []   # 已记录物体列表 (class, x, y, z)
        self.obstacle_counter = 0

        # 输出文件
        if not os.path.exists(OUTPUT_FILE):
            with open(OUTPUT_FILE, 'w') as f:
                f.write("id,class,confidence,x_uav,y_uav,z_uav,timestamp\n")

        # 订阅
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_cb)
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_cb)
        self.start_ok_pub = rospy.Publisher("/start_ok", Bool, queue_size=10)

        # 发布处理后的图像（压缩格式，减少带宽）
        self.debug_pub = rospy.Publisher("/vision/processed_image/compressed", CompressedImage, queue_size=10)

        self.latest_img = None
        self.latest_pose = None

        # 中文字体（字号20，红色）
        try:
            self.font = ImageFont.truetype(FONT_PATH, 20)
            rospy.loginfo("加载中文字体成功")
        except:
            self.font = ImageFont.load_default()
            rospy.logwarn("中文字体加载失败，使用默认字体")

        self.timer = rospy.Timer(rospy.Duration(0.05), self.process)
        rospy.loginfo("视觉监控节点启动，等待二维码...")

    def draw_chinese_text(self, img, text, position, text_color=(255,0,0)):   # 红色 (RGB)
        """在OpenCV图像上绘制中文，返回新图像"""
        img_pil = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        draw.text(position, text, font=self.font, fill=text_color)
        img = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        return img

    def image_cb(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("图像转换错误: %s", e)

    def odom_cb(self, msg):
        self.latest_pose = msg.pose.pose

    # 框合并辅助函数（不变，省略）
    def calculate_box_area(self, xyxy):
        x1, y1, x2, y2 = xyxy
        return (x2 - x1) * (y2 - y1)

    def calculate_overlap_area(self, box1, box2):
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        inter_x1 = max(x1_1, x1_2)
        inter_y1 = max(y1_1, y1_2)
        inter_x2 = min(x2_1, x2_2)
        inter_y2 = min(y2_1, y2_2)
        if inter_x1 >= inter_x2 or inter_y1 >= inter_y2:
            return 0
        return (inter_x2 - inter_x1) * (inter_y2 - inter_y1)

    def merge_overlapping_boxes(self, boxes_data, overlap_threshold):
        if not boxes_data:
            return []
        cls_groups = {}
        for box in boxes_data:
            cls = box['cls']
            if cls not in cls_groups:
                cls_groups[cls] = []
            cls_groups[cls].append(box)

        merged_boxes = []
        for cls, boxes in cls_groups.items():
            merged = [False] * len(boxes)
            for i in range(len(boxes)):
                if merged[i]:
                    continue
                current = boxes[i]
                current_xyxy = current['xyxy']
                current_area = current['area']
                to_merge = [current]
                for j in range(i+1, len(boxes)):
                    if merged[j]:
                        continue
                    compare = boxes[j]
                    compare_xyxy = compare['xyxy']
                    compare_area = compare['area']
                    overlap = self.calculate_overlap_area(current_xyxy, compare_xyxy)
                    min_area = min(current_area, compare_area)
                    overlap_ratio = overlap / min_area if min_area > 0 else 0
                    if overlap_ratio >= overlap_threshold:
                        to_merge.append(compare)
                        merged[j] = True
                if len(to_merge) > 1:
                    x1s = [b['xyxy'][0] for b in to_merge]
                    y1s = [b['xyxy'][1] for b in to_merge]
                    x2s = [b['xyxy'][2] for b in to_merge]
                    y2s = [b['xyxy'][3] for b in to_merge]
                    merged_xyxy = [min(x1s), min(y1s), max(x2s), max(y2s)]
                    max_conf = max(b['conf'] for b in to_merge)
                    merged_box = {
                        'cls': cls,
                        'xyxy': merged_xyxy,
                        'conf': max_conf,
                        'area': self.calculate_box_area(merged_xyxy)
                    }
                    merged_boxes.append(merged_box)
                else:
                    merged_boxes.append(current)
        return merged_boxes

    def is_duplicate(self, cls, x, y, z):
        for (c, cx, cy, cz) in self.recorded_objects:
            if c != cls:
                continue
            dist = np.hypot(x - cx, y - cy)
            if dist < POSITION_DUP_THRESH:
                return True
        return False

    def add_recorded_object(self, cls, x, y, z):
        self.recorded_objects.append((cls, x, y, z))
        if len(self.recorded_objects) > 200:
            self.recorded_objects = self.recorded_objects[-200:]

    def process(self, event):
        if self.latest_img is None:
            return

        img = self.latest_img.copy()

        if self.state == "QR":
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and not self.qr_detected:
                self.qr_detected = True
                self.frozen_frame = img.copy()
                aruco.drawDetectedMarkers(self.frozen_frame, corners, ids)
                qr_id = ids[0][0]
                corner_pts = corners[0][0]
                center_x = int(np.mean(corner_pts[:, 0]))
                center_y = int(np.mean(corner_pts[:, 1]))
                cv2.circle(self.frozen_frame, (center_x, center_y), 5, (0, 255, 0), -1)
                self.frozen_frame = self.draw_chinese_text(self.frozen_frame, f"识别到楼栋消息，为编号 {qr_id}", (50, 50))
                self.start_ok_pub.publish(Bool(data=True))
                rospy.loginfo("检测到二维码 ID=%d，发布启动信号，等待10秒", qr_id)
                self.switch_time = time.time()

            if self.qr_detected:
                display_img = self.frozen_frame.copy()
                if self.switch_time:
                    remaining = max(0, 10 - (time.time() - self.switch_time))
                    display_img = self.draw_chinese_text(display_img, f"即将切换至杂物检测模式，剩余 {int(remaining)} 秒", (50, 100))
                if time.time() - self.switch_time >= 10:
                    self.state = "OBSTACLE"
                    rospy.loginfo("10秒已到，切换至杂物检测模式")
            else:
                display_img = self.draw_chinese_text(img.copy(), "正在等待二维码...", (50, 50))

        elif self.state == "OBSTACLE":
            display_img = img.copy()
            # YOLO 检测
            results = self.yolo(img, conf=YOLO_CONF_THRESH, verbose=False)
            result = results[0]

            boxes_data = []
            for box in result.boxes:
                cls = int(box.cls)
                xyxy = box.xyxy[0].tolist()
                conf = float(box.conf)
                area = self.calculate_box_area(xyxy)
                boxes_data.append({
                    'cls': cls,
                    'xyxy': xyxy,
                    'conf': conf,
                    'area': area
                })

            merged_boxes = self.merge_overlapping_boxes(boxes_data, OVERLAP_THRESH)

            # 获取无人机位置
            if self.latest_pose is None:
                uav_x, uav_y, uav_z = 0.0, 0.0, 0.0
            else:
                uav_x = self.latest_pose.position.x
                uav_y = self.latest_pose.position.y
                uav_z = self.latest_pose.position.z

            # 先绘制所有检测框（不进行去重显示）
            for box in merged_boxes:
                xyxy = box['xyxy']
                x1, y1, x2, y2 = map(int, xyxy)
                cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{self.yolo.names[box['cls']]} {box['conf']:.2f}"
                cv2.putText(display_img, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # 然后处理记录（去重）
            for box in merged_boxes:
                cls = box['cls']
                conf = box['conf']
                class_name = self.yolo.names[cls]

                if self.is_duplicate(cls, uav_x, uav_y, uav_z):
                    continue

                self.obstacle_counter += 1
                timestamp = rospy.Time.now().to_sec()
                with open(OUTPUT_FILE, 'a') as f:
                    f.write(f"{self.obstacle_counter},{class_name},{conf:.3f},{uav_x:.1f},{uav_y:.1f},{uav_z:.1f},{timestamp}\n")
                rospy.loginfo("保存杂物 #%d: %s 位置(%.1f,%.1f,%.1f)", self.obstacle_counter, class_name,
                              uav_x, uav_y, uav_z)
                self.add_recorded_object(cls, uav_x, uav_y, uav_z)

            display_img = self.draw_chinese_text(display_img, "杂物检测模式", (50, 50))

        # 将处理后的图像转换为压缩消息并发布
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(display_img, dst_format='jpg')
            compressed_msg.header.stamp = rospy.Time.now()
            self.debug_pub.publish(compressed_msg)
        except Exception as e:
            rospy.logerr("发布处理图像失败: %s", e)

if __name__ == '__main__':
    try:
        node = VisionMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass