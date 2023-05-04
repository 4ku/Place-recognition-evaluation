from scripts.methods import BaseMethod
from scripts.methods.superglue.matching import Matching
from .superglue.utils import frame2tensor

import cv2
import torch
from tqdm import tqdm
import numpy as np


class SuperGlue(BaseMethod):

    def __init__(self, threshold):
        self.THRESHOLD = threshold

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        config = {
            'superpoint':
            {
                'nms_radius': 4,
                'keypoint_threshold': 0.005,
                'max_keypoints': 1024
            },
            'superglue':
            {
                'weights': 'indoor',
                'sinkhorn_iterations': 20,
                'match_threshold': 0.2
            }
        }

        self.matching = Matching(config).eval().to(self.device)

        self.N_ROWS = 480
        self.N_COLS = 640

    def __str__(self) -> str:
        return self.__class__.__name__ + f'_threshold({self.THRESHOLD})'
    
    def predict_loop_candidates(self, cloud_vec, odom_vec, img_vec, min_idx_diff=1) -> np.ndarray:
        size = len(img_vec)
        predicted_loop_candidates = np.full((size, size), False)

        for i in tqdm(range(size)):

            resized1 = cv2.resize(img_vec[i], (self.N_COLS, self.N_ROWS))
            tensor1 = frame2tensor(resized1, self.device)

            for j in range(i - min_idx_diff + 1):
                tensor2 = frame2tensor(cv2.resize(
                    img_vec[j], (self.N_COLS, self.N_ROWS)), self.device)

                pred = self.matching({'image0': tensor1, 'image1': tensor2})
                pred = {k: v[0].cpu().detach().numpy()
                        for k, v in pred.items()}

                kpts0, kpts1 = pred['keypoints0'], pred['keypoints1']
                matches, conf = pred['matches0'], pred['matching_scores0']
                valid = matches > -1
                mkpts0 = kpts0[valid]

                del tensor2
                del pred

                desc_dist = mkpts0.shape[0]
                if desc_dist > self.THRESHOLD:
                    predicted_loop_candidates[i][j] = True

            del tensor1

        return np.array(predicted_loop_candidates)
