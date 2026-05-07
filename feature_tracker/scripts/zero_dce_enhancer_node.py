#!/usr/bin/env python
from __future__ import print_function

import os
import time

import cv2
import numpy as np
import rospy
import torch
import torch.nn as nn
import torch.nn.functional as F
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class DepthwiseSeparableConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(DepthwiseSeparableConv, self).__init__()
        self.depth_conv = nn.Conv2d(
            in_channels=in_channels,
            out_channels=in_channels,
            kernel_size=3,
            stride=1,
            padding=1,
            groups=in_channels,
            bias=True,
        )
        self.point_conv = nn.Conv2d(
            in_channels=in_channels,
            out_channels=out_channels,
            kernel_size=1,
            stride=1,
            padding=0,
            groups=1,
            bias=True,
        )

    def forward(self, x):
        return self.point_conv(self.depth_conv(x))


class ZeroDcePlusNet(nn.Module):
    def __init__(self, scale_factor=12):
        super(ZeroDcePlusNet, self).__init__()
        number_f = 32
        self.relu = nn.ReLU(inplace=True)
        self.scale_factor = max(1, int(scale_factor))
        self.e_conv1 = DepthwiseSeparableConv(3, number_f)
        self.e_conv2 = DepthwiseSeparableConv(number_f, number_f)
        self.e_conv3 = DepthwiseSeparableConv(number_f, number_f)
        self.e_conv4 = DepthwiseSeparableConv(number_f, number_f)
        self.e_conv5 = DepthwiseSeparableConv(number_f * 2, number_f)
        self.e_conv6 = DepthwiseSeparableConv(number_f * 2, number_f)
        self.e_conv7 = DepthwiseSeparableConv(number_f * 2, 3)

    def forward(self, x):
        if self.scale_factor == 1:
            x_down = x
        else:
            height = max(1, int(x.size(2)) // self.scale_factor)
            width = max(1, int(x.size(3)) // self.scale_factor)
            x_down = F.interpolate(x, size=(height, width), mode="bilinear", align_corners=False)

        x1 = self.relu(self.e_conv1(x_down))
        x2 = self.relu(self.e_conv2(x1))
        x3 = self.relu(self.e_conv3(x2))
        x4 = self.relu(self.e_conv4(x3))
        x5 = self.relu(self.e_conv5(torch.cat([x3, x4], 1)))
        x6 = self.relu(self.e_conv6(torch.cat([x2, x5], 1)))
        x_r = torch.tanh(self.e_conv7(torch.cat([x1, x6], 1)))
        if self.scale_factor != 1:
            x_r = F.interpolate(x_r, size=(int(x.size(2)), int(x.size(3))), mode="bilinear", align_corners=False)

        for _ in range(8):
            x = x + x_r * (torch.pow(x, 2) - x)
        return x


# Keep the historical class name importable for small local tooling scripts.
EnhanceNetNoPool = ZeroDcePlusNet


class ZeroDceEnhancerNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.input_topic = rospy.get_param("~input_topic", "/camera/color/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/zero_dce/image_enhanced")
        default_model = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "models",
            "zero_dce_plus_epoch99.pth",
        )
        self.model_path = rospy.get_param("~model_path", default_model)
        self.scale_factor = int(rospy.get_param("~scale_factor", 12))
        self.torch_threads = max(1, int(rospy.get_param("~torch_threads", 1)))
        self.queue_size = int(rospy.get_param("~queue_size", 2))
        self.use_gpu = bool(rospy.get_param("~use_gpu", True))
        self.publish_original_on_error = bool(rospy.get_param("~publish_original_on_error", True))
        self.profile = bool(rospy.get_param("~profile", False))

        torch.set_num_threads(self.torch_threads)
        self.device = self._select_device()
        self.net = self._load_model()
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=self.queue_size)
        self.subscriber = rospy.Subscriber(
            self.input_topic, Image, self.image_callback, queue_size=self.queue_size, buff_size=2 ** 24
        )

        rospy.loginfo(
            "Zero-DCE++ enhancer subscribed to %s, publishing %s on %s, scale_factor=%d, torch_threads=%d",
            self.input_topic,
            self.output_topic,
            self.device,
            self.scale_factor,
            self.torch_threads,
        )

    def _select_device(self):
        if self.use_gpu and torch.cuda.is_available():
            return "cuda"
        if self.use_gpu:
            rospy.logwarn("Zero-DCE++ requested GPU, but CUDA is unavailable; using CPU")
        return "cpu"

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise rospy.ROSInitException("Zero-DCE++ model not found: %s" % self.model_path)

        net = ZeroDcePlusNet(self.scale_factor)
        map_location = None if self.device == "cuda" else "cpu"
        state_dict = torch.load(self.model_path, map_location=map_location)
        net.load_state_dict(state_dict)
        net.eval()
        if self.device == "cuda":
            net.cuda()
        return net

    def image_callback(self, msg):
        start = time.time()
        try:
            rgb = self._message_to_rgb(msg)
            enhanced_bgr = self._enhance(rgb)
            out_msg = self.bridge.cv2_to_imgmsg(enhanced_bgr, encoding="bgr8")
            out_msg.header = msg.header
            self.publisher.publish(out_msg)
            if self.profile:
                rospy.loginfo("Zero-DCE++ inference %.3f ms", (time.time() - start) * 1000.0)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Zero-DCE++ enhancement failed: %s", exc)
            if self.publish_original_on_error:
                self.publisher.publish(msg)

    def _message_to_rgb(self, msg):
        try:
            if msg.encoding == "rgb8":
                rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            elif msg.encoding == "bgr8":
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            elif msg.encoding in ("mono8", "8UC1"):
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError:
            raise
        return np.ascontiguousarray(rgb)

    def _enhance(self, rgb):
        data = rgb.astype(np.float32) / 255.0
        tensor = torch.from_numpy(data).permute(2, 0, 1).unsqueeze(0)
        if self.device == "cuda":
            tensor = tensor.cuda(non_blocking=True)

        with torch.no_grad():
            enhanced = self.net(tensor)

        enhanced = enhanced.detach().clamp(0.0, 1.0).cpu().numpy()[0]
        enhanced = np.transpose(enhanced, (1, 2, 0))
        enhanced_rgb = (enhanced * 255.0 + 0.5).astype(np.uint8)
        return cv2.cvtColor(enhanced_rgb, cv2.COLOR_RGB2BGR)


if __name__ == "__main__":
    rospy.init_node("zero_dce_enhancer")
    node = ZeroDceEnhancerNode()
    rospy.spin()
