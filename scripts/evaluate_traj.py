import os
import os.path as osp
# import sys

ICL_NUIM_PATH = "/media/lj/TOSHIBA/dataset/ICL-NUIM"
# scene_type = 'home'
scene_type = 'office'
data_num = 1


if scene_type == 'home':
    dataset_path = f"{ICL_NUIM_PATH}/home/living_room_traj{data_num}_frei_png"
    gt_traj = osp.join(dataset_path, f"livingRoom{data_num}_adjusted.gt.freiburg")
elif scene_type == 'office':
    dataset_path = f"{ICL_NUIM_PATH}/office/traj{data_num}_frei_png"
    gt_traj = osp.join(dataset_path, f"traj{data_num}_adjusted.gt.freiburg")


my_kf_traj = osp.join(dataset_path, "KeyFrameTrajectory_QSP.txt")
my_traj = osp.join(dataset_path, "CameraTrajectory_QSP.txt")

ORB_SLAM_kf_traj = osp.join(dataset_path, "KeyFrameTrajectory_ORB-SLAM.txt")
ORB_SLAM_traj = osp.join(dataset_path, "CameraTrajectory_ORB-SLAM.txt")

# cmd = f"evo_rpe tum {gt_traj} {my_traj} -a -s -p"

cmd = f"evo_traj tum --ref {gt_traj} {my_traj} {ORB_SLAM_traj} -a -s -p"
# cmd = f"evo_traj tum --ref {gt_traj} {my_traj} {my_kf_traj} {ORB_SLAM_traj} {ORB_SLAM_kf_traj} -a -s -p"
# cmd = f"evo_rpe tum {gt_traj} {gt_traj} -a -s -p"
os.system(cmd)