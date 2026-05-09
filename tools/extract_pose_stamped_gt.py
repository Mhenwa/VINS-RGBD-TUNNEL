#!/usr/bin/env python
"""Extract a geometry_msgs/PoseStamped trajectory from a rosbag."""

from __future__ import print_function

import argparse
import os
import sys

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Input rosbag path.")
    parser.add_argument(
        "--topic",
        default="/vrpn_client_node/jackal/pose",
        help="PoseStamped topic to extract.",
    )
    parser.add_argument("--out", required=True, help="Output txt path: time tx ty tz.")
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    count = 0
    with rosbag.Bag(args.bag, "r") as bag, open(args.out, "w") as handle:
        for _, msg, bag_time in bag.read_messages(topics=[args.topic]):
            stamp = msg.header.stamp
            timestamp = stamp.to_sec() if stamp and not stamp.is_zero() else bag_time.to_sec()
            pose = msg.pose.position
            handle.write("%.9f %.9f %.9f %.9f\n" % (timestamp, pose.x, pose.y, pose.z))
            count += 1

    if count == 0:
        raise RuntimeError("No messages found on %s in %s" % (args.topic, args.bag))
    print("wrote %d poses to %s" % (count, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
