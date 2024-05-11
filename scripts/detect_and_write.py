import os.path as osp
import os
import cv2


dataset_path = f"/media/lj/TOSHIBA/dataset/EllipsoidSLAM/living_room_traj2n_frei_png"

bbox_path = osp.join(dataset_path, "bbox")
rgb_path = osp.join(dataset_path, "rgb")

indexs = [int(f.split('.')[0]) for f in os.listdir(bbox_path)]

indexs = sorted(indexs)

for index in indexs:
    # print(f"index = {index}")
    bbox_file = osp.join(bbox_path, f'{index}.txt')
    rgb_file = osp.join(rgb_path, f'{index}.png')
    f = open(bbox_file, 'r')
    img = cv2.imread(rgb_file)
    matrix = []
    for line in f.readlines():
        row = line.strip().split()
        # print(f"row[1:5] = {row[1:5]}")
        x1, y1, x2, y2 = [int(x) for x in row[1:5]]
        start_point = (x1, y1)
        end_point = (x2, y2)

        # 定义矩形的颜色（BGR 格式）
        color = (0, 255, 0)  # 这里使用绿色

        # 绘制矩形
        cv2.rectangle(img, start_point, end_point, color, thickness=2)

    cv2.imshow("img", img)
    cv2.waitKey()