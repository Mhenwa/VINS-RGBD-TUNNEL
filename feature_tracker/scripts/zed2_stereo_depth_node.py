#!/usr/bin/env python
from __future__ import print_function

import threading

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Zed2StereoDepthNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame_count = 0
        self.last_log_time = rospy.Time.now()

        self.left_topic = rospy.get_param("~left_topic", "/zed_node/left/image_rect_color")
        self.right_topic = rospy.get_param("~right_topic", "/zed_node/right/image_rect_color")
        self.output_left_topic = rospy.get_param(
            "~output_left_topic", "/subsurface_georobo/zed2/left_resized"
        )
        self.depth_topic = rospy.get_param("~depth_topic", "/subsurface_georobo/zed2/depth")
        self.fx = float(rospy.get_param("~fx", 537.533))
        self.baseline = float(rospy.get_param("~baseline", 0.1190))
        self.min_depth = float(rospy.get_param("~min_depth", 0.35))
        self.max_depth = float(rospy.get_param("~max_depth", 20.0))
        self.queue_size = int(rospy.get_param("~queue_size", 8))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.03))
        self.resize_scale = float(rospy.get_param("~resize_scale", 1.0))
        self.profile = bool(rospy.get_param("~profile", False))

        block_size = int(rospy.get_param("~block_size", 5))
        if block_size % 2 == 0:
            block_size += 1
        num_disparities = int(rospy.get_param("~num_disparities", 192))
        num_disparities = max(16, (num_disparities // 16) * 16)
        min_disparity = int(rospy.get_param("~min_disparity", 0))

        self.matcher = cv2.StereoSGBM_create(
            minDisparity=min_disparity,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size * block_size,
            P2=32 * 3 * block_size * block_size,
            disp12MaxDiff=int(rospy.get_param("~disp12_max_diff", 1)),
            uniquenessRatio=int(rospy.get_param("~uniqueness_ratio", 8)),
            speckleWindowSize=int(rospy.get_param("~speckle_window_size", 80)),
            speckleRange=int(rospy.get_param("~speckle_range", 2)),
            preFilterCap=int(rospy.get_param("~pre_filter_cap", 31)),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self.pub_left = rospy.Publisher(self.output_left_topic, Image, queue_size=2)
        self.pub_depth = rospy.Publisher(self.depth_topic, Image, queue_size=2)
        left_sub = message_filters.Subscriber(self.left_topic, Image)
        right_sub = message_filters.Subscriber(self.right_topic, Image)
        sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], self.queue_size, self.sync_slop
        )
        sync.registerCallback(self.callback)
        self.sync = sync

        rospy.loginfo(
            "ZED2 stereo depth: left=%s right=%s output_left=%s depth=%s fx=%.3f baseline=%.4f scale=%.3f",
            self.left_topic,
            self.right_topic,
            self.output_left_topic,
            self.depth_topic,
            self.fx,
            self.baseline,
            self.resize_scale,
        )

    def _resize_if_needed(self, img, interpolation):
        if abs(self.resize_scale - 1.0) < 1e-6:
            return img
        return cv2.resize(
            img,
            (0, 0),
            fx=self.resize_scale,
            fy=self.resize_scale,
            interpolation=interpolation,
        )

    def _to_gray(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = self._resize_if_needed(img, cv2.INTER_AREA)
        if img.ndim == 2:
            return img
        if msg.encoding.lower() in ("bgra8", "rgba8"):
            code = cv2.COLOR_BGRA2GRAY if msg.encoding.lower() == "bgra8" else cv2.COLOR_RGBA2GRAY
            return cv2.cvtColor(img, code)
        if msg.encoding.lower() == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def callback(self, left_msg, right_msg):
        start = rospy.Time.now()
        try:
            left_img = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="passthrough")
            left_img = self._resize_if_needed(left_img, cv2.INTER_AREA)
            left_gray = self._to_gray(left_msg)
            right_gray = self._to_gray(right_msg)
            disparity = self.matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
            valid = disparity > 0.5
            depth_m = np.zeros(disparity.shape, dtype=np.float32)
            depth_m[valid] = (self.fx * self.baseline) / disparity[valid]
            valid &= (depth_m >= self.min_depth) & (depth_m <= self.max_depth)

            depth_mm = np.zeros(disparity.shape, dtype=np.uint16)
            depth_mm[valid] = np.clip(depth_m[valid] * 1000.0, 0, 65535).astype(np.uint16)

            depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding="mono16")
            depth_msg.header = left_msg.header
            left_out_msg = self.bridge.cv2_to_imgmsg(left_img, encoding=left_msg.encoding)
            left_out_msg.header = left_msg.header
            self.pub_left.publish(left_out_msg)
            self.pub_depth.publish(depth_msg)

            if self.profile:
                with self.lock:
                    self.frame_count += 1
                    now = rospy.Time.now()
                    elapsed = (now - self.last_log_time).to_sec()
                    if elapsed >= 2.0:
                        hz = self.frame_count / elapsed
                        latency_ms = (now - start).to_sec() * 1000.0
                        valid_ratio = float(np.count_nonzero(valid)) / float(valid.size)
                        rospy.loginfo(
                            "ZED2 stereo depth %.2f Hz, %.1f ms, valid %.1f%%",
                            hz,
                            latency_ms,
                            valid_ratio * 100.0,
                        )
                        self.frame_count = 0
                        self.last_log_time = now
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "ZED2 stereo depth failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("zed2_stereo_depth_node")
    Zed2StereoDepthNode()
    rospy.spin()
