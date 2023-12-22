#
# This file is part of https://github.com/JingwenWang95/DSP-SLAM
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#

import warnings
import cv2
import torch
import numpy as np
import mmcv
from mmcv.runner import load_checkpoint
from mmdet.models import build_detector
from mmdet.core import get_classes
from mmdet.apis import inference_detector


object_class_table = {
    "cars": [2],
    "benches": [13], # 板凳
    "backpack": [24], # 背包
    "chairs": [56, 57], # 椅子，沙发
    "bottles": [39], # 瓶子
    "wine_glasses": [40], # 酒杯
    "cups": [41], # 杯子
    "bowls": [45], # 碗
    "bananas": [46], "apples": [47], "oranges": [49],
    "potted_plants": [58], # 盆栽植物
    "beds": [59],
    "dining_tables": [60],
    "tv_monitor": [62],
    "laptop": [63],
    "mouse": [64],
    "keyboard": [66],
    "microwave": [68], "oven":[69], "toaster": [70], "refrigerator": [72],
    "book": [73], "clock": [74], "vase": [75], "teddy_bears": [77]
}


def get_detector2d(configs):
    return Detector2D(configs)


class Detector2D(object):
    def __init__(self, configs):
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        config = configs.Detector2D.config_path
        checkpoint = configs.Detector2D.weight_path
        if isinstance(config, str):
            config = mmcv.Config.fromfile(config)
        elif not isinstance(config, mmcv.Config):
            raise TypeError('config must be a filename or Config object, '
                            f'but got {type(config)}')
        config.model.pretrained = None
        config.model.train_cfg = None
        self.model = build_detector(config.model, test_cfg=config.get('test_cfg'))
        if checkpoint is not None:
            checkpoint = load_checkpoint(self.model, checkpoint, map_location='cpu')
            if 'CLASSES' in checkpoint.get('meta', {}):
                self.model.CLASSES = checkpoint['meta']['CLASSES']
            else:
                warnings.simplefilter('once')
                warnings.warn('Class names are not saved in the checkpoint\'s '
                              'meta data, use COCO classes by default.')
                self.model.CLASSES = get_classes('coco')
        self.model.cfg = config  # save the config in the model for convenience
        self.model.to(device)
        self.model.eval()
        self.min_bb_area = configs.min_bb_area
        self.predictions = None

    def make_prediction(self, image, object_class="cars"):
        
        # assert object_class == "chairs" or object_class == "cars"
        assert object_class in object_class_table

        self.predictions = inference_detector(self.model, image)

        # print(f"type(self.predictions) = {self.predictions}")

        """
            structure of predictions:
            - bboxs: self.predictions[0]
                - bboxs[class]: self.predictions[0][class_index]
                    - bbox: self.predictions[0][class_index][instance_index]
                        - x1, y1, x2, y2, prob
            - masks: self.predictions[1]
                - masks[class]: self.predictions[1][class_index]
        """

        bboxes = []
        masks = []
        labels = []

        n_det = 0

        for o in object_class_table[object_class]:
            # print(f"o = {o}")
            n_det_bbox = len(self.predictions[0][o])
            n_det_mask = len(self.predictions[1][o])
            assert n_det_bbox == n_det_mask,  f"len(bbox[{o}]) != len(mask[{o}])"
            bboxes_o = self.predictions[0][o]
            bboxes.append(bboxes_o)
            masks += self.predictions[1][o]
            labels.extend([o for i in range(n_det_bbox)])
            n_det += n_det_bbox

        bboxes = np.concatenate(bboxes, axis=0)
        labels = np.array(labels)
        probs = bboxes[:,4]
        
        # In case there is no detections
        if n_det == 0:
            masks = np.zeros((0, 0, 0))
        else:
            masks = np.stack(masks, axis=0)

        return self.get_valid_detections(bboxes, masks, labels, probs)

    def visualize_result(self, image, filename):
        self.model.show_result(image, self.predictions, out_file=filename)


    def get_valid_detections(self, boxes, masks, labels, probs):
        # Remove those on the margin

        cond1 = (boxes[:, 0] >= 30) & (boxes[:, 1] > 10) & (boxes[:, 2] < 1211) & (boxes[:, 3] < 366)
        boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        # Remove those with too small bounding boxes

        cond2 = (boxes_area > self.min_bb_area)
        scores = boxes[:, -1]
        cond3 = (scores >= 0.70)

        # valid_mask = (cond2 & cond3)
        valid_mask = (cond2 & cond3)

        valid_instances = {"pred_boxes": boxes[valid_mask, :4],
                           "pred_masks": masks[valid_mask, ...],
                           "pred_labels": labels[valid_mask],
                           "pred_probs": probs[valid_mask],
                           }

        return valid_instances

    @staticmethod
    def save_masks(masks):
        mask_imgs = masks.cpu().numpy()
        n = mask_imgs.shape[0]
        for i in range(n):
            cv2.imwrite("mask_%d.png" % i, mask_imgs[i, ...].astype(np.float32) * 255.)