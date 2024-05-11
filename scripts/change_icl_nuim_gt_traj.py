import os.path as osp
import sys

def process_file(input_file, output_file, divide_value):
    with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
        for line in f_in:
            # 按空格分割每行数据
            data = line.strip().split()
            # 提取FrameID和对应的图像路径
            frame_id, x, y, z, qx, qy, qz, qw = data
            # 计算新的时间戳
            time = (int(frame_id)-1) / divide_value
            decimal_places = 8
            time_formatted = "{:.{}f}".format(time, decimal_places)
            # 写入新文件
            # f_out.write(f"{time_formatted} {x} {-float(y)} {z} {qx} {qy} {qz} {qw}\n")
            f_out.write(f"{time_formatted} {x} {y} {z} {qx} {qy} {qz} {qw}\n")


ICL_NUIM_PATH = "/media/lj/TOSHIBA/dataset/ICL-NUIM"
# scene_type = 'home'
scene_type = 'office'
data_num = 1

# 指定除数值
divide_value = 30  # 你可以根据需求调整这个值

if scene_type == 'home':
    dir_path = f"{ICL_NUIM_PATH}/home/living_room_traj{data_num}_frei_png"
    # 指定输入文件路径
    input_file_path = osp.join(dir_path, f"livingRoom{data_num}.gt.freiburg")
    # 指定输出文件路径
    output_file_path = osp.join(dir_path, f"livingRoom{data_num}_adjusted.gt.freiburg")
elif scene_type == 'office':
    dir_path = f"{ICL_NUIM_PATH}/office/traj{data_num}_frei_png"
    # 指定输入文件路径
    input_file_path = osp.join(dir_path, f"traj{data_num}.gt.freiburg")
    # 指定输出文件路径
    output_file_path = osp.join(dir_path, f"traj{data_num}_adjusted.gt.freiburg")
else:
    sys.exit(-1)


process_file(input_file_path, output_file_path, divide_value)

