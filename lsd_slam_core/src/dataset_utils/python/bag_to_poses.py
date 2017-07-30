#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract pose messages from a rosbag.
"""

import os
import argparse

import cv2

import rospy
import rosbag

def main():
    """Extract pose messages from a rosbag into a text file in TUM format.

    The poses will be written to a text file with the following fields per line:
      timestamp tx ty tz qx qy qz qw
    """
    parser = argparse.ArgumentParser(description="Extract pose messages from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_file", help="Output file.")
    parser.add_argument("pose_topic", help="Pose topic.")
    args = parser.parse_args()

    topics = [args.pose_topic]

    print "Extract poses from %s on topic %s into %s" % (args.bag_file,
                                                         args.pose_topic,
                                                         args.output_file)

    tum_file = open(args.output_file, 'w')

    bag = rosbag.Bag(args.bag_file, "r")
    for topic, msg, t in bag.read_messages(topics=topics):
        # Write TUM info file.
        time = msg.header.stamp.to_sec()
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        tum_file.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" %
                       (time, tx, ty, tz, qx, qy, qz, qw))

    tum_file.close()
    bag.close()

    return

if __name__ == '__main__':
    main()
