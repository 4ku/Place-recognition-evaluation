from abc import ABC, abstractmethod
from typing import List
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Pose


class BaseMethod(ABC):
    @abstractmethod
    def predict_loop_candidates(self, cloud_vec: List[PointCloud2], odom_vec: List[Pose],
                                img_vec: List[Image], min_idx_diff: int) -> List[List[bool]]:
        pass
