#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from matplotlib import pyplot as plt
import rospy
import tf
from sensor_msgs.msg import PointCloud2 #Image, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from matplotlib import pyplot as plt
from subprocess import Popen
import sys
import os
HOME = os.path.abspath(os.path.expanduser("~"))
sys.path.insert(0, os.path.join(HOME, "Documents/pyniel")) # Pyniel as available on github
from pyniel.ros_tools.pointclouds import pointcloud2_to_numpy
plt.ion()

class Listener(object):
    def __init__(self, max_frames=10):
        self.fixed_frame = 'SurroundingFrontLaser_frame'
        self.depth_cloud_topic = "/pepper_robot/camera/depth_registered/points"
        self.frames = []
        self.tfs = []
        self.MAX_FRAMES = max_frames
        self.tf_listener = tf.TransformListener()
        self.finished = False

        rospy.init_node('listener', anonymous=False)
        self.depth_sub = rospy.Subscriber(self.depth_cloud_topic, PointCloud2, self._depth_cloud_callback)

    def spin(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def _depth_cloud_callback(self, data):
        rospy.loginfo("Frame: %s " +  rospy.get_caller_id() + "I heard %s", len(self.frames), data.width)
        try:
            tf_fixed_to_sensor = self.tf_listener.lookupTransform(self.fixed_frame, data.header.frame_id, rospy.Time(0))
            self.tfs.append(tf_fixed_to_sensor)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf not found")
        self.frames.append(data)
        if len(self.frames) > self.MAX_FRAMES:
          self.finished = True
          rospy.loginfo("Stopping ros listen")
          rospy.signal_shutdown("Message received")

    def export_frames(self, directory=os.path.join(HOME, "pepper_data/depth_clouds/")):
        if not self.finished:
            print("Frames are not yet recorded.")
            return None
        frame1 = self.frames[0]
        frame2 = self.frames[-1]
        tf1 = self.tfs[0]
        tf2 = self.tfs[-1]
        f1 = pointcloud2_to_numpy(frame1)
        f2 = pointcloud2_to_numpy(frame2)
        # TODO save tfs if useful
        if directory is not None:
            directory = os.path.abspath(directory)
            if not os.path.exists(directory):
              os.makedirs(directory)
            print("Saving scans to {}".format(directory))
            np.save(os.path.join(directory, "scan1.npy"), f1)
            np.save(os.path.join(directory, "scan2.npy"), f2)
        return f1, f2

if __name__ == '__main__':
    l = Listener()
    try:
      l.spin()
    except KeyboardInterrupt:
        print("Ended")
    #frames = l.frames
