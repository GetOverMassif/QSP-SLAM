import os,sys
import os.path as osp
import cv2
import matplotlib.pyplot as plt

sys.path.append("./")

import reconstruct.utils as io_utils
from reconstruct.detector2d import Detector2D
from mmdet.apis import show_result_pyplot

import warnings

warnings.filterwarnings("ignore")

# img_dir = f"assets/nmb_objects"
img_dir = f"/home/lj/Documents/codes/QSP-SLAM/assets/nmb_objects"

pyCfgPath = f"configs/config_NMD_mobile_phone.json"
pyCfg = io_utils.get_configs(pyCfgPath)

detector = Detector2D(configs=pyCfg)

img_list = list(os.listdir(img_dir))
# img_list = [img_list[0]]

for img_name in img_list:
    img_path = osp.join(img_dir, img_name)
    # print(f"img_path = {img_path}")
    img_bgr = cv2.imread(img_path)
    img_rgb = img_bgr[:,:,(2,1,0)]

    predictions = detector.make_prediction(img_bgr)
    img_labeled_bgr = show_result_pyplot(detector.model, img_bgr, detector.predictions, score_thr=0.2)
    # img_labeled_rgb = img_labeled_bgr[:,:,(2,1,0)]

    
    # plt.imshow(img_labeled_rgb)
    # plt.show()

    # print(f"{img_bgr.shape[0]}, {img_bgr.shape[1]}")
