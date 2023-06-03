#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from message_filters import TimeSynchronizer, Subscriber

from scripts.evaluator import Evaluator
from typing import List
from scripts.methods.base import BaseMethod

def getMethods() -> List[BaseMethod]:
    """
    Get the methods to be evaluated.

    :return: List[BaseMethod], the list of methods to be evaluated
    """
    # Create methods
    methods = []
    method_name = rospy.get_param('method', None)

    if method_name is not None:
        threshold = float(rospy.get_param('threshold'))

        if method_name == "logg3d":
            from scripts.methods.LoGG3D import LoGG3D
            methods.append(LoGG3D(threshold))
        elif method_name == "superglue":
            from scripts.methods.SuperGlue import SuperGlue
            methods.append(SuperGlue(threshold))
        elif method_name == "mix_vpr":
            from scripts.methods.MixVPR_ import MixVPR
            methods.append(MixVPR(threshold))
        else:
            rospy.logerr(
                "Invalid method parameter. Use 'logg3d' or 'superglue'.")
            exit(-1)
    else:
        # from scripts.methods.LoGG3D import LoGG3D
        # methods.append(LoGG3D(0.08))

        # from scripts.methods.SuperGlue import SuperGlue
        # methods.append(SuperGlue(240))

        # from scripts.methods.MixVPR_ import MixVPR
        # methods.append(MixVPR(0.13))

        # Example for finding best threshold for LoGG3D
        from scripts.methods.LoGG3D import LoGG3D
        for threshold in range(5, 17):
            threshold = threshold / 100.0  # Convert to float equivalent
            methods.append(LoGG3D(threshold))

        # Example for finding best threshold for SuperGlue
        # from scripts.methods.SuperGlue import SuperGlue
        # for threshold in range(150, 301, 30):
        #     methods.append(SuperGlue(threshold))

        # Example for finding best threshold for MixVPR
        # from scripts.methods.MixVPR_ import MixVPR
        # for threshold in range(9, 21):
        #     threshold = threshold / 100.0  # Convert to float equivalent
        #     methods.append(MixVPR(threshold))

    return methods


def process_bag(evaluator, input_bag_path, lidar_topic, odom_topic, camera_topic):
    cloud_msg = None
    odom_msg = None
    img_msg = None

    with rosbag.Bag(input_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[lidar_topic, odom_topic, camera_topic]):
            if topic == lidar_topic:
                cloud_msg = msg
            elif topic == odom_topic:
                odom_msg = msg
            elif topic == camera_topic:
                img_msg = msg

            if cloud_msg is not None and odom_msg is not None and img_msg is not None:
                evaluator.synced_callback(cloud_msg, odom_msg, img_msg)
                cloud_msg = None
                odom_msg = None
                img_msg = None


rospy.init_node('evaluation_node')

# Retrieve parameters from the ROS parameter server
keyframe_type = int(rospy.get_param('keyframe_type'))
cloud_size = int(rospy.get_param('cloud_size'))

# Common params
record_size = int(rospy.get_param('record_size'))
min_idx_diff = int(rospy.get_param('min_idx_diff'))
angle_consideration = bool(rospy.get_param('angle_consideration'))
max_angle_deg = int(rospy.get_param('max_angle_deg'))
max_dist = float(rospy.get_param('max_dist'))
save_candidates = bool(rospy.get_param('save_candidates'))

if keyframe_type == 1:
    record_size = int(record_size / cloud_size)

# Get methods
methods = getMethods()

# Create evaluator
evaluator = Evaluator(methods, record_size, angle_consideration,
                      max_angle_deg, max_dist, min_idx_diff, save_candidates)


# Get merged topics
lidar_topic = rospy.get_param('merged_lidar_topic')
odom_topic = rospy.get_param('merged_odom_topic')
camera_topic = rospy.get_param('merged_camera_topic')

use_rosbag = bool(rospy.get_param('use_rosbag'))
input_bag_path = rospy.get_param('merge_rosbag_output_path')

if use_rosbag:
    process_bag(evaluator, input_bag_path,
                lidar_topic, odom_topic, camera_topic)
else:
    # Create subscribers
    cloud_sub = Subscriber(lidar_topic, PointCloud2, queue_size=5)
    odom_sub = Subscriber(odom_topic, Odometry, queue_size=20)
    img_sub = Subscriber(camera_topic, Image, queue_size=5)

    # Synchronize messages from different topics
    sync = TimeSynchronizer([cloud_sub, odom_sub, img_sub], queue_size=10)

    # Register the synchronized callback
    sync.registerCallback(evaluator.synced_callback)

    # Process callbacks and wait for new messages
    rospy.spin()
