#!/usr/bin/env python
"""Record ROS topic rates and simple wall-clock latencies for VINS-RGBD runs."""

from __future__ import print_function

import json
import math
import os
import signal
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud

try:
    from voxblox_msgs.msg import Mesh
except Exception:
    Mesh = None


def stamp_key(stamp):
    return int(stamp.to_nsec())


def is_finite(value):
    return not (math.isnan(value) or math.isinf(value))


def summarize(values):
    values = [float(v) for v in values if is_finite(float(v))]
    if not values:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "median": None,
            "p95": None,
        }
    values.sort()
    count = len(values)
    p95_index = int(math.ceil(0.95 * count)) - 1
    p95_index = max(0, min(count - 1, p95_index))
    mid = count // 2
    if count % 2:
        median = values[mid]
    else:
        median = 0.5 * (values[mid - 1] + values[mid])
    return {
        "count": count,
        "min": values[0],
        "max": values[-1],
        "mean": sum(values) / count,
        "median": median,
        "p95": values[p95_index],
    }


class TopicStats(object):
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.first_wall = None
        self.last_wall = None
        self.first_stamp = None
        self.last_stamp = None
        self.intervals = []

    def record(self, stamp, wall_time):
        if self.last_wall is not None:
            interval = wall_time - self.last_wall
            if interval >= 0:
                self.intervals.append(interval)
        if self.first_wall is None:
            self.first_wall = wall_time
        self.last_wall = wall_time
        stamp_sec = stamp.to_sec() if stamp is not None else None
        if self.first_stamp is None:
            self.first_stamp = stamp_sec
        self.last_stamp = stamp_sec
        self.count += 1

    def to_dict(self):
        duration = None
        hz = None
        if self.first_wall is not None and self.last_wall is not None:
            duration = self.last_wall - self.first_wall
            if duration > 0 and self.count > 1:
                hz = float(self.count - 1) / duration
        return {
            "topic": self.name,
            "count": self.count,
            "wall_duration_sec": duration,
            "hz": hz,
            "first_stamp_sec": self.first_stamp,
            "last_stamp_sec": self.last_stamp,
            "interval_sec": summarize(self.intervals),
        }


class PerfMonitor(object):
    def __init__(self):
        self.output_path = rospy.get_param("~output_path", "/tmp/perf_monitor.json")
        self.raw_topic = rospy.get_param("~raw_topic", "/camera/color/image_raw")
        self.enhanced_topic = rospy.get_param("~enhanced_topic", "/zero_dce/image_enhanced")
        self.feature_topic = rospy.get_param("~feature_topic", "/feature_tracker/feature")
        self.odom_topic = rospy.get_param("~odom_topic", "/vins_estimator/odometry")
        self.mesh_topic = rospy.get_param("~mesh_topic", "/pose_graph/voxblox/mesh")
        self.max_cache_size = int(rospy.get_param("~max_cache_size", 5000))

        self.start_wall = time.time()
        self.end_wall = None
        self.saved = False

        self.stats = {
            "raw": TopicStats(self.raw_topic),
            "enhanced": TopicStats(self.enhanced_topic),
            "feature": TopicStats(self.feature_topic),
            "odom": TopicStats(self.odom_topic),
            "mesh": TopicStats(self.mesh_topic),
        }
        self.raw_wall_by_stamp = {}
        self.enhanced_wall_by_stamp = {}
        self.latencies = {
            "raw_to_enhanced_ms": [],
            "image_to_feature_ms": [],
            "image_to_odom_ms": [],
        }

        self.subscribers = [
            rospy.Subscriber(self.raw_topic, Image, self.raw_callback, queue_size=200),
            rospy.Subscriber(self.enhanced_topic, Image, self.enhanced_callback, queue_size=200),
            rospy.Subscriber(self.feature_topic, PointCloud, self.feature_callback, queue_size=200),
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=200),
        ]
        if Mesh is not None:
            self.subscribers.append(rospy.Subscriber(self.mesh_topic, Mesh, self.mesh_callback, queue_size=50))

        rospy.on_shutdown(self.save)
        rospy.loginfo("perf_monitor writing %s", self.output_path)

    def prune(self, cache):
        if len(cache) <= self.max_cache_size:
            return
        for key in sorted(cache.keys())[: len(cache) - self.max_cache_size]:
            cache.pop(key, None)

    def raw_callback(self, msg):
        wall = time.time()
        self.stats["raw"].record(msg.header.stamp, wall)
        self.raw_wall_by_stamp[stamp_key(msg.header.stamp)] = wall
        self.prune(self.raw_wall_by_stamp)

    def enhanced_callback(self, msg):
        wall = time.time()
        self.stats["enhanced"].record(msg.header.stamp, wall)
        key = stamp_key(msg.header.stamp)
        self.enhanced_wall_by_stamp[key] = wall
        if key in self.raw_wall_by_stamp:
            self.latencies["raw_to_enhanced_ms"].append((wall - self.raw_wall_by_stamp[key]) * 1000.0)
        self.prune(self.enhanced_wall_by_stamp)

    def feature_callback(self, msg):
        wall = time.time()
        self.stats["feature"].record(msg.header.stamp, wall)
        key = stamp_key(msg.header.stamp)
        source_wall = self.enhanced_wall_by_stamp.get(key, self.raw_wall_by_stamp.get(key))
        if source_wall is not None:
            self.latencies["image_to_feature_ms"].append((wall - source_wall) * 1000.0)

    def odom_callback(self, msg):
        wall = time.time()
        self.stats["odom"].record(msg.header.stamp, wall)
        key = stamp_key(msg.header.stamp)
        source_wall = self.enhanced_wall_by_stamp.get(key, self.raw_wall_by_stamp.get(key))
        if source_wall is not None:
            self.latencies["image_to_odom_ms"].append((wall - source_wall) * 1000.0)

    def mesh_callback(self, msg):
        wall = time.time()
        self.stats["mesh"].record(msg.header.stamp, wall)

    def save(self):
        if self.saved:
            return
        self.saved = True
        self.end_wall = time.time()
        data = {
            "start_wall_sec": self.start_wall,
            "end_wall_sec": self.end_wall,
            "wall_duration_sec": self.end_wall - self.start_wall,
            "topics": dict((name, stat.to_dict()) for name, stat in self.stats.items()),
            "latencies_ms": dict((name, summarize(values)) for name, values in self.latencies.items()),
        }
        directory = os.path.dirname(self.output_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(self.output_path, "w") as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
            handle.write("\n")
        rospy.loginfo("perf_monitor saved %s", self.output_path)


def main():
    rospy.init_node("perf_monitor", anonymous=False)
    monitor = PerfMonitor()

    def handle_signal(signum, frame):
        monitor.save()
        rospy.signal_shutdown("signal %s" % signum)

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    rospy.spin()
    monitor.save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
