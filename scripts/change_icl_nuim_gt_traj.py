import os.path as osp
import sys

def my_format(num_str):
    return format(float(num_str), '.6f')

def process_file(input_file, output_file, divide_value, use_index=False):
    with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
        cnt = 0
        for line in f_in:
            # 按空格分割每行数据
            data = line.strip().split()
            # 提取FrameID和对应的图像路径
            frame_id, x, y, z, qx, qy, qz, qw = data
            # 计算新的时间戳
            time = (int(frame_id)-1) / divide_value
            decimal_places = 6

            # time_formatted = "{:.{}f}".format(time, decimal_places)

            # y_adjusted = float(y)
            # z_adjusted = float(z)

            # z_adjusted += 2.25

            # 写入新文件
            if use_index:
                f_out.write(f"{cnt} {my_format(x)} {my_format(y)} {my_format(z)} {my_format(qx)} {my_format(qy)} {my_format(qz)} {my_format(qw)}\n")
            else:
                f_out.write(f"{my_format(time)} {my_format(x)} {my_format(y)} {my_format(z)} {my_format(qx)} {my_format(qy)} {my_format(qz)} {my_format(qw)}\n")
            
            cnt += 1


ICL_NUIM_PATH = "/media/lj/TOSHIBA/dataset/ICL-NUIM"
# scene_type = 'home'
scene_type = 'office'
data_num = 2

# 指定除数值
divide_value = 30  # 你可以根据需求调整这个值

suffix = ".gt.freiburg"

if scene_type == 'home':
    dir_path = f"{ICL_NUIM_PATH}/home/living_room_traj{data_num}_frei_png"
    # 指定输入文件路径
    input_file_path = osp.join(dir_path, f"livingRoom{data_num}{suffix}")
    # 指定输出文件路径
    output_file_path = osp.join(dir_path, f"livingRoom{data_num}")
elif scene_type == 'office':
    dir_path = f"{ICL_NUIM_PATH}/office/traj{data_num}_frei_png"
    # 指定输入文件路径
    input_file_path = osp.join(dir_path, f"traj{data_num}{suffix}")
    # 指定输出文件路径
    output_file_path = osp.join(dir_path, f"traj{data_num}")
else:
    sys.exit(-1)


# process_file(input_file_path, f"{output_file_path}_index_invY{suffix}", divide_value, use_index=True, inverse_y=True)
# process_file(input_file_path, f"{output_file_path}_index_noInv{suffix}", divide_value, use_index=True, inverse_y=False)
# process_file(input_file_path, f"{output_file_path}_time_invY{suffix}", divide_value, use_index=False, inverse_y=True)
# process_file(input_file_path, f"{output_file_path}_time_noInv{suffix}", divide_value, use_index=False, inverse_y=False)

process_file(input_file_path, f"{dir_path}/groundtruth.txt", divide_value, use_index=False)

