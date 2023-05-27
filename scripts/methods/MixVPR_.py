import sys
import os

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
mixVPR_path = os.path.join(current_directory, 'MixVPR')
sys.path.insert(0, mixVPR_path)

import utils
import numpy as np
import faiss.contrib.torch_utils
import faiss
from .MixVPR.models import helper

import rospy
import os
import torch
import cv2

import sys
import os

import pytorch_lightning as pl
import torch
from torch.optim import lr_scheduler

from scripts.methods.base import BaseMethod
from tqdm import tqdm



class VPRModel(pl.LightningModule):
    """This is the main model for Visual Place Recognition
    we use Pytorch Lightning for modularity purposes.

    Args:
        pl (_type_): _description_
    """

    def __init__(self,
                 # ---- Backbone
                 backbone_arch='resnet50',
                 pretrained=True,
                 layers_to_freeze=1,
                 layers_to_crop=[],

                 # ---- Aggregator
                 agg_arch='ConvAP',  # CosPlace, NetVLAD, GeM
                 agg_config={},

                 # ---- Train hyperparameters
                 lr=0.03,
                 optimizer='sgd',
                 weight_decay=1e-3,
                 momentum=0.9,
                 warmpup_steps=500,
                 milestones=[5, 10, 15],
                 lr_mult=0.3,

                 # ----- Loss
                 loss_name='MultiSimilarityLoss',
                 miner_name='MultiSimilarityMiner',
                 miner_margin=0.1,
                 faiss_gpu=True
                 ):
        super().__init__()
        self.encoder_arch = backbone_arch
        self.pretrained = pretrained
        self.layers_to_freeze = layers_to_freeze
        self.layers_to_crop = layers_to_crop

        self.agg_arch = agg_arch
        self.agg_config = agg_config

        self.lr = lr
        self.optimizer = optimizer
        self.weight_decay = weight_decay
        self.momentum = momentum
        self.warmpup_steps = warmpup_steps
        self.milestones = milestones
        self.lr_mult = lr_mult

        self.loss_name = loss_name
        self.miner_name = miner_name
        self.miner_margin = miner_margin

        self.save_hyperparameters()  # write hyperparams into a file

        self.loss_fn = utils.get_loss(loss_name)
        self.miner = utils.get_miner(miner_name, miner_margin)
        # we will keep track of the % of trivial pairs/triplets at the loss level
        self.batch_acc = []

        self.faiss_gpu = faiss_gpu

        # ----------------------------------
        # get the backbone and the aggregator
        self.backbone = helper.get_backbone(
            backbone_arch, pretrained, layers_to_freeze, layers_to_crop)
        self.aggregator = helper.get_aggregator(agg_arch, agg_config)

    # the forward pass of the lightning model
    def forward(self, x):
        x = self.backbone(x)
        x = self.aggregator(x)
        return x

    # configure the optimizer
    def configure_optimizers(self):
        if self.optimizer.lower() == 'sgd':
            optimizer = torch.optim.SGD(self.parameters(),
                                        lr=self.lr,
                                        weight_decay=self.weight_decay,
                                        momentum=self.momentum)
        elif self.optimizer.lower() == 'adamw':
            optimizer = torch.optim.AdamW(self.parameters(),
                                          lr=self.lr,
                                          weight_decay=self.weight_decay)
        elif self.optimizer.lower() == 'adam':
            optimizer = torch.optim.AdamW(self.parameters(),
                                          lr=self.lr,
                                          weight_decay=self.weight_decay)
        else:
            raise ValueError(
                f'Optimizer {self.optimizer} has not been added to "configure_optimizers()"')
        scheduler = lr_scheduler.MultiStepLR(
            optimizer, milestones=self.milestones, gamma=self.lr_mult)
        return [optimizer], [scheduler]

    # configure the optizer step, takes into account the warmup stage
    def optimizer_step(self,  epoch, batch_idx,
                       optimizer, optimizer_idx, optimizer_closure,
                       on_tpu, using_native_amp, using_lbfgs):
        # warm up lr
        if self.trainer.global_step < self.warmpup_steps:
            lr_scale = min(
                1., float(self.trainer.global_step + 1) / self.warmpup_steps)
            for pg in optimizer.param_groups:
                pg['lr'] = lr_scale * self.lr
        optimizer.step(closure=optimizer_closure)

    #  The loss function call (this method will be called at each training iteration)
    def loss_function(self, descriptors, labels):
        # we mine the pairs/triplets if there is an online mining strategy
        if self.miner is not None:
            miner_outputs = self.miner(descriptors, labels)
            loss = self.loss_fn(descriptors, labels, miner_outputs)

            # calculate the % of trivial pairs/triplets
            # which do not contribute in the loss value
            nb_samples = descriptors.shape[0]
            nb_mined = len(set(miner_outputs[0].detach().cpu().numpy()))
            batch_acc = 1.0 - (nb_mined/nb_samples)

        else:  # no online mining
            loss = self.loss_fn(descriptors, labels)
            batch_acc = 0.0
            if type(loss) == tuple:
                # somes losses do the online mining inside (they don't need a miner objet),
                # so they return the loss and the batch accuracy
                # for example, if you are developping a new loss function, you might be better
                # doing the online mining strategy inside the forward function of the loss class,
                # and return a tuple containing the loss value and the batch_accuracy (the % of valid pairs or triplets)
                loss, batch_acc = loss

        # keep accuracy of every batch and later reset it at epoch start
        self.batch_acc.append(batch_acc)
        # log it
        self.log('b_acc', sum(self.batch_acc) /
                 len(self.batch_acc), prog_bar=True, logger=True)
        return loss

    # This is the training step that's executed at each iteration
    def training_step(self, batch, batch_idx):
        places, labels = batch

        # Note that GSVCities yields places (each containing N images)
        # which means the dataloader will return a batch containing BS places
        BS, N, ch, h, w = places.shape

        # reshape places and labels
        images = places.view(BS*N, ch, h, w)
        labels = labels.view(-1)

        # Feed forward the batch to the model
        # Here we are calling the method forward that we defined above
        descriptors = self(images)
        # Call the loss_function we defined above
        loss = self.loss_function(descriptors, labels)

        self.log('loss', loss.item(), logger=True)
        return {'loss': loss}

    # This is called at the end of eatch training epoch
    def training_epoch_end(self, training_step_outputs):
        # we empty the batch_acc list for next epoch
        self.batch_acc = []

    # For validation, we will also iterate step by step over the validation set
    # this is the way Pytorch Lghtning is made. All about modularity, folks.
    def validation_step(self, batch, batch_idx, dataloader_idx=None):
        places, _ = batch
        # calculate descriptors
        descriptors = self(places)
        return descriptors.detach().cpu()

    def validation_epoch_end(self, val_step_outputs):
        """this return descriptors in their order
        depending on how the validation dataset is implemented 
        for this project (MSLS val, Pittburg val), it is always references then queries
        [R1, R2, ..., Rn, Q1, Q2, ...]
        """
        dm = self.trainer.datamodule
        # The following line is a hack: if we have only one validation set, then
        # we need to put the outputs in a list (Pytorch Lightning does not do it presently)
        if len(dm.val_datasets) == 1:  # we need to put the outputs in a list
            val_step_outputs = [val_step_outputs]

        for i, (val_set_name, val_dataset) in enumerate(zip(dm.val_set_names, dm.val_datasets)):
            feats = torch.concat(val_step_outputs[i], dim=0)

            if 'pitts' in val_set_name:
                # split to ref and queries
                num_references = val_dataset.dbStruct.numDb
                num_queries = len(val_dataset)-num_references
                positives = val_dataset.getPositives()
            elif 'msls' in val_set_name:
                # split to ref and queries
                num_references = val_dataset.num_references
                num_queries = len(val_dataset)-num_references
                positives = val_dataset.pIdx
            else:
                print(
                    f'Please implement validation_epoch_end for {val_set_name}')
                raise NotImplemented

            r_list = feats[: num_references]
            q_list = feats[num_references:]
            pitts_dict = utils.get_validation_recalls(r_list=r_list,
                                                      q_list=q_list,
                                                      k_values=[
                                                          1, 5, 10, 15, 20, 50, 100],
                                                      gt=positives,
                                                      print_results=True,
                                                      dataset_name=val_set_name,
                                                      faiss_gpu=self.faiss_gpu
                                                      )
            del r_list, q_list, feats, num_references, positives

            self.log(f'{val_set_name}/R1',
                     pitts_dict[1], prog_bar=False, logger=True)
            self.log(f'{val_set_name}/R5',
                     pitts_dict[5], prog_bar=False, logger=True)
            self.log(f'{val_set_name}/R10',
                     pitts_dict[10], prog_bar=False, logger=True)
        print('\n\n')


class MixVPR(BaseMethod):

    def __init__(self, threshold):
        super().__init__()
        self.threshold = threshold

        use_cuda = torch.cuda.is_available()
        # Note that images must be resized to 320x320
        model = VPRModel(backbone_arch='resnet50',
                         layers_to_crop=[4],
                         agg_arch='MixVPR',
                         agg_config={'in_channels': 1024,
                                     'in_h': 20,
                                     'in_w': 20,
                                     'out_channels': 1024,
                                     'mix_depth': 4,
                                     'mlp_ratio': 1,
                                     'out_rows': 4},
                            faiss_gpu=use_cuda
                         )

        current_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(
            current_dir, 'MixVPR', 'resnet50_MixVPR_4096_channels(1024)_rows(4).ckpt')
        state_dict = torch.load(model_path)
        model.load_state_dict(state_dict)
        model.eval()

        self.device = torch.device(
            "cuda" if use_cuda else "cpu")
        model.to(self.device)
        self.model = model

        rospy.loginfo(f'Loading model from {model_path}')
        if use_cuda:
            res = faiss.StandardGpuResources()
            flat_config = faiss.GpuIndexFlatConfig()
            flat_config.useFloat16 = True
            flat_config.device = 0
            self.faiss_index = faiss.GpuIndexFlatL2(res, 4096, flat_config)
        else:
            self.faiss_index = faiss.IndexFlatL2(4096)
        rospy.loginfo(f'Model is loaded')


    def __str__(self) -> str:
        return self.__class__.__name__ + f'_threshold({self.threshold})'

    def predict_loop_candidates(self,cloud_vec, odom_vec, img_vec, min_idx_diff=1) -> np.ndarray:
        size = len(img_vec)
        predicted_loop_candidates = np.full((size, size), False)

        descriptors = []
        for i in tqdm(range(size)):
            image = img_vec[i]

            # Convert grayscale image to RGB
            if len(image.shape) == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            elif len(image.shape) == 3 and image.shape[2] == 1:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            elif len(image.shape) == 3 and image.shape[2] > 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)

            # Resize the image. Final shape is (320, 320)
            resized = cv2.resize(image, (320, 320))

            with torch.no_grad():
                img_tensor = torch.from_numpy(resized).permute(
                    2, 0, 1).unsqueeze(0).float().to(self.device)
                descriptor = self.model(img_tensor).cpu()

            descriptors.append(descriptor)

            if i < min_idx_diff:
                continue
            
            self.faiss_index.reset()
            self.faiss_index.add(
                np.vstack(descriptors[:-min_idx_diff]))

            distances, indices = self.faiss_index.search(descriptor, len(descriptors[:-min_idx_diff]))
            
            for idx, desc_dist in zip(indices[0], distances[0]):
                if desc_dist < self.threshold:
                    predicted_loop_candidates[i][idx] = True

        return np.array(predicted_loop_candidates)
