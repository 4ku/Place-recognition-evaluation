import abc
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import numpy as np
import math
import matplotlib.pyplot as plt
from scripts.methods import BaseMethod
from typing import Optional, Tuple, List
import time
import os
from pathlib import Path


def angular_difference(quat_1, quat_2):
    """
    Calculate the angular difference between two quaternions.

    sql

    :param quat_1: Quaternion, the first quaternion
    :param quat_2: Quaternion, the second quaternion
    :return: float, the angle between the two quaternions in radians
    """
    cos_angle = quat_1.x * quat_2.x + quat_1.y * \
        quat_2.y + quat_1.z * quat_2.z + quat_1.w * quat_2.w
    angle = 2 * math.acos(abs(cos_angle))
    return angle


def euclidean_difference(point_1, point_2):
    """
    Calculate the Euclidean distance between two 3D points.

    sql

    :param point_1: Point, the first 3D point
    :param point_2: Point, the second 3D point
    :return: float, the Euclidean distance between the two points
    """
    dx = point_1.x - point_2.x
    dy = point_1.y - point_2.y
    dz = point_1.z - point_2.z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    return dist


class Evaluator(abc.ABC):
    """
    Evaluator class for evaluating loop closure detection methods.

    This class collects data, calculates real loop closures and predicted loop closures
    based on the provided methods, and calculates evaluation metrics (precision, recall, and F1 score).
    """

    def __init__(self, methods: List[BaseMethod], record_size: int, angle_consideration: bool, max_angle_deg: int, max_dist: float, min_idx_diff: int, save_candidates: bool):
        """
        Initialize the Evaluator object.

        :param methods: List[BaseMethod], list of loop closure detection methods to evaluate
        :param record_size: int, number of records to collect before evaluating
        :param angle_consideration: bool, flag to consider angular difference between poses for loop closure
        :param max_angle_deg: int, maximum angular difference in degrees for considering loop closure
        :param max_dist: float, maximum Euclidean distance for considering loop closure
        :param min_idx_diff: int, minimum index difference between two poses to consider them as potential loop closures
        """
        self.methods = methods
        self.RECORD_SIZE = record_size
        self.ANGLE_CONSIDERATION = angle_consideration
        self.MAX_ANGLE_DEG = max_angle_deg
        self.MIN_IDX_DIFF = min_idx_diff

        self.MAX_DIST = max_dist
        self.MAX_ANGLE_RAD = (self.MAX_ANGLE_DEG/180)*3.14

        self.SAVE_CANDIDATES = save_candidates

        self.odom_vec = []
        self.cloud_vec = []
        self.img_vec = []

        self.real_loop_candidates = np.full(
            (record_size, record_size), False)

        self.counter = 0
        # ros->numpy converter
        self.bridge = CvBridge()

    def synced_callback(self, cloud_msg: PointCloud2, odom_msg: Odometry, img_msg: Optional[Image]) -> None:
        """
        ROS callback function to receive synchronized sensor data and evaluate loop closure detection methods.

        :param cloud_msg: PointCloud2, point cloud message
        :param odom_msg: Odometry, odometry message
        :param img_msg: Optional[Image], image message (optional)
        :return: None
        """
        log_frequency = 50
        if self.counter % log_frequency == 0:
            rospy.loginfo("Amount of data recorded: %d", self.counter)

        # Store data to lists
        self.cloud_vec.append(cloud_msg)
        self.odom_vec.append(odom_msg.pose.pose)
        if img_msg is not None:
            image = self.bridge.imgmsg_to_cv2(
                img_msg, desired_encoding='mono8')
            self.img_vec.append(image)

        self.counter += 1

        if self.counter == self.RECORD_SIZE:

            self.get_real_loop_candidates()
            self.get_model_loop_candidates()

            # Plot odometry path
            self.plot_path()

            rospy.signal_shutdown("Evaluation completed.")

    def save_candidates(self, candidates, filename):
        if self.SAVE_CANDIDATES:
            path = Path(os.path.dirname(__file__)).parent / "results"
            if self.ANGLE_CONSIDERATION:
                path = os.path.join(path, "with_angle")
            else:
                path = os.path.join(path, "no_angle")
            
            # Check if the directory exists and create it if it doesn't
            if not os.path.exists(path):
                os.makedirs(path)
            
            path = os.path.join(path, filename)
            
            with open(path, 'w') as f:
                for i in range(1, self.RECORD_SIZE):
                    for j in range(i - self.MIN_IDX_DIFF + 1):
                        f.write(f"{int(candidates[i][j])} ")
                    f.write("\n")

    def get_real_loop_candidates(self) -> None:
        """
        Calculate real loop candidates based on Euclidean distance and angular difference between odometry poses.
        Save real loop candidates to a file if SAVE_CANDIDATES is True.

        :return: None
        """
        for i in range(1, self.RECORD_SIZE):
            for j in range(i - self.MIN_IDX_DIFF + 1):
                odom_diff = euclidean_difference(
                    self.odom_vec[i].position, self.odom_vec[j].position)
                condition = odom_diff < self.MAX_DIST

                if self.ANGLE_CONSIDERATION:
                    ang_diff = angular_difference(
                        self.odom_vec[i].orientation, self.odom_vec[j].orientation)
                    condition = condition and ang_diff < self.MAX_ANGLE_RAD

                if condition:
                    self.real_loop_candidates[i][j] = True

        method_name = str(self.methods[0]).split("_")[0]
        self.save_candidates(self.real_loop_candidates, f"real_{method_name}.txt")

    def get_model_loop_candidates(self) -> None:
        """
        Calculate predicted loop candidates using the provided loop closure detection methods.

        :return: None
        """

        best_f1_score = 0
        best_method = None
        
        for method in self.methods:
            start = time.time()
            predicted_loop_candidates = method.predict_loop_candidates(
                self.cloud_vec, self.odom_vec, self.img_vec, self.MIN_IDX_DIFF)
            duration = time.time() - start
            rospy.loginfo(f"Results for method: {method}")
            rospy.loginfo(
                f"It took {duration :.5f} seconds to predict loop candidates ({len(self.cloud_vec)} frames in total).")
            rospy.loginfo(
                f"It took {duration / len(self.cloud_vec) :.5f} seconds per frame.")


            precision, recall, f1_score, accuracy = self.calculate_metrics(predicted_loop_candidates)

            rospy.loginfo("----------------------------------------------")
            rospy.loginfo(f"Precision: {precision:.3f}")
            rospy.loginfo(f"Recall   : {recall:.3f}")
            rospy.loginfo(f"F1 Score : {f1_score:.3f}")
            rospy.loginfo(f"Accuracy : {accuracy:.3f}")
            rospy.loginfo("----------------------------------------------")
            rospy.loginfo("----------------------------------------------")
            rospy.loginfo(" ")

            if f1_score > best_f1_score:
                best_f1_score = f1_score
                best_method = method

            self.save_candidates(predicted_loop_candidates, f"{method}.txt")
        
        rospy.loginfo(f"Best method with f1_score {best_f1_score:.3f}: {best_method}")

    def calculate_metrics(self, predicted_loop_candidates: List[List[bool]]) -> Tuple[float, float, float, float]:
        """
        Calculate evaluation metrics (precision, recall, F1 score, and accuracy) based on the real and predicted loop candidates.

        :return: Tuple[float, float, float, float], evaluation metrics (precision, recall, F1 score, and accuracy)
        """
        TP = 0
        FP = 0
        FN = 0
        TN = 0

        for i in range(1, self.RECORD_SIZE):
            for j in range(i - self.MIN_IDX_DIFF + 1):
                if self.real_loop_candidates[i][j] and predicted_loop_candidates[i][j]:
                    TP += 1
                elif not self.real_loop_candidates[i][j] and predicted_loop_candidates[i][j]:
                    FP += 1
                elif self.real_loop_candidates[i][j] and not predicted_loop_candidates[i][j]:
                    FN += 1
                else:
                    TN += 1

        if TP + FP == 0:
            precision = 0
        else:
            precision = TP / (TP + FP)

        if TP + FN == 0:
            recall = 0
        else:
            recall = TP / (TP + FN)

        if precision + recall == 0:
            f1_score = 0
        else:
            f1_score = 2 * (precision * recall) / (precision + recall)

        if TP + FP + TN + FN == 0:
            accuracy = 0
        else:
            accuracy = (TP + TN) / (TP + FP + TN + FN)

        return precision, recall, f1_score, accuracy

    def plot_path(self):

        # Extract x and y coordinates from odom_vec
        x_coords = [pose.position.x for pose in self.odom_vec]
        y_coords = [pose.position.y for pose in self.odom_vec]

        # Plot the data
        plt.plot(x_coords, y_coords)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Odometry Plot')
        plt.savefig("../trajectory.png")
