#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import numpy as np

import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from camera_info_manager import saveCalibrationFile

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("cinfo_topic", help="Camera Info topic.")
    parser.add_argument("--cname", default="camera", help="The camera name.")
    parser.add_argument("--tum", action="store_true", help="Generate TUM text file.")

    args = parser.parse_args()

    topics = [args.image_topic, args.cinfo_topic]

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    if args.tum:
        tum_file = open(os.path.join(args.output_dir, "rgb.txt"), 'w')

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    wrote_cinfo = False
    for topic, msg, t in bag.read_messages(topics=topics):
        if (topic == args.image_topic):
            # Parse the image and write to disk.
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # FLIP FOR UPSIDE DOWN FLEA3
            # cv_img = np.flipud(cv_img)
            # cv_img = np.fliplr(cv_img)

            fname = "%06i.png" % count
            cv2.imwrite(os.path.join(args.output_dir, fname), cv_img)
            print "Wrote image %i" % count

            if args.tum:
                # Write TUM info file.
                time = msg.header.stamp.to_sec()
                tum_file.write("%.6f rgb/%s\n" % (time, fname))

            count += 1
        elif (topic == args.cinfo_topic) and not wrote_cinfo:
            # Write the camera info to a yml file.
            cinfo_file = os.path.join(args.output_dir, args.cname + ".yml")
            saveCalibrationFile(msg, cinfo_file, args.cname)
            wrote_cinfo = True

    if args.tum:
        tum_file.close()

    bag.close()

    return

if __name__ == '__main__':
    main()
