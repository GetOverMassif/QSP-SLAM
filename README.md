# QSP-SLAM
Quadric Object SLAM with Deep Shape Prior


数据准备过程：
- associate
- build
- 标注
- 目标检测过程

实验测试


首先，数据包括：
- 图像文件
  - **depth**文件夹：数据以 `<index>.png` 或  `<index>.jpg` 命名，由 0 ~ n-1, 共 n 张
  - **rgb**文件夹：同样形式，与depth文件夹中的文件一一对应

- 轨迹真值文件：
  - **groundtruth.txt**: `timestamp` `x` `y` `x` `qx` `qy` `qz` `qw`, timestamp 从 0.000000 开始, 共

- 关联文件：
  - **associations_adjusted.txt**: `timestamp` `path/to/rgb_img` `timestamp` `path/to/depth_img`，timestamp 
  - **associate.txt**: `timestamp` `path/to/rgb_img` `index` `path/to/depth_img`, index 0~n-1,共 n 个
  - **associateGroundtruth.txt**: `index` `path/to/rgb_img` `index` `path/to/depth_img`, index 从 0~n-1，共n个
  - 

此过程中要确保不同文件中对应的 timestamp 相同，因为timestamp可能作为map的键使用

CameraTrajectoryVO.txt
associateOdom.txt

点云文件


物体标注结果：
`index` `x` `y` `z` `roll` `pitch` `yaw` `length / 2` `width / 2` `height / 2` `label`


保存下来的地图结果： `save_map_dir`


有待做的：
手动设置是否根据prob_3D排除观测结果
手动设置是否进行聚类

提取除了支撑平面之外的曼哈顿平面
调试点模型

全局优化

实现便捷、精准的标注

真正建立评测