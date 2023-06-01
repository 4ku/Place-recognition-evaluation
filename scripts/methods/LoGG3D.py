from scripts.methods.base import BaseMethod
from sensor_msgs import point_cloud2

import numpy as np
from tqdm import tqdm
import os
import torch

from .LoGG3D_Net.models.pipelines.pipeline_utils import make_sparse_tensor
from .LoGG3D_Net.models.pipeline_factory import get_pipeline
from scipy.spatial.distance import cdist
import numpy as np

def descriptor_difference(desc_1, desc_2):
    # Calculate the dot product between desc_1 and desc_2
    dot_product = np.dot(desc_1, desc_2)

    # Calculate the product of the magnitudes of desc_1 and desc_2
    magnitude_product = np.linalg.norm(desc_1) * np.linalg.norm(desc_2)

    # Calculate the cosine similarity using the dot product and magnitude product
    cosine_similarity = 1 - (dot_product / magnitude_product)

    # Return the cosine similarity
    return cosine_similarity


class LoGG3D(BaseMethod):

    def __init__(self, threshold):
        self.THRESHOLD = threshold

        model = get_pipeline('LOGG3D')
        save_path = os.path.join(os.path.dirname(__file__), 'LoGG3D_Net/checkpoints')
        save_path = str(save_path) +\
            '/mulran_10cm/2021-09-14_08-59-00_3n24h_MulRan_v10_q29_4s_263039.pth'

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        checkpoint = torch.load(
            save_path, map_location=torch.device(self.device))
        model.load_state_dict(checkpoint['model_state_dict'])
        # model = model.cuda()
        model.eval()
        self.model = model
        self.seen_descriptors = []

    def __str__(self) -> str:
        return self.__class__.__name__ + f'_threshold({self.THRESHOLD})'

    def predict_loop_candidates(self, cloud_vec, odom_vec, img_vec, min_idx_diff=1)  -> np.ndarray:
        desc_list = []
        for cloud in tqdm(cloud_vec):

            # convert cloud into numpy
            cloud_current = []
            points = point_cloud2.read_points(
                cloud, field_names=['x', 'y', 'z', 'intensity'], skip_nans=True)

            for point in points:
                cloud_current.append(point[:4])

            cloud = np.array(cloud_current).astype('float32')
            out = self.predict_simple(cloud)
            desc_list.append(out)
        
        size = len(desc_list)
        predicted_loop_candidates = np.full((size, size), False)
        for i in range(1, size):
            for j in range(i - min_idx_diff + 1):
                desc_dist = descriptor_difference(desc_list[i], desc_list[j])
                if desc_dist < self.THRESHOLD:
                    predicted_loop_candidates[i][j] = True
        
        return np.array(predicted_loop_candidates)
    
    def predict_simple(self, point_cloud):
        input = make_sparse_tensor(point_cloud, 0.1).to(self.device)
        output_desc, output_feats = self.model(input)
        global_descriptor = output_desc.detach().to('cpu').numpy()
        return global_descriptor

    def predict(self, point_cloud):
        input = make_sparse_tensor(point_cloud, 0.1).to(self.device)
        output_desc, output_feats = self.model(input)  # .squeeze()
        output_feats = output_feats[0]
        global_descriptor = output_desc.cpu().detach().to('cpu').numpy()

        global_descriptor = np.reshape(global_descriptor, (1, -1))

        if len(global_descriptor) < 1:
            return None

        self.seen_descriptors.append(global_descriptor)

        if len(self.seen_descriptors) < 15:
            print("Not enough descriptors")
            print("len(seen_descriptors)", len(self.seen_descriptors))
            return None

        db_seen_descriptors = np.copy(self.seen_descriptors)
        db_seen_descriptors = db_seen_descriptors[:-14]
        db_seen_descriptors = db_seen_descriptors.reshape(
            -1, np.shape(global_descriptor)[1])

        feat_dists = cdist(global_descriptor, db_seen_descriptors,
                           metric='cosine').reshape(-1)
        min_dist, nearest_idx = np.min(feat_dists), np.argmin(feat_dists)
        return min_dist, nearest_idx
