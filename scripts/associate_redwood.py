import argparse
import os
import os.path as osp
import subprocess

def get_img_txt(rgb_img_path, rgb_txt_path, prefix):
    rgb_img_list = sorted(os.listdir(rgb_img_path))
    with open(rgb_txt_path, 'w') as f:
        f.write(f"# color images\n")
        f.write(f"# file: 'redwood'\n")
        f.write(f"# timestamp filename")
        for rgb_filename in rgb_img_list:
            timestamp = float(rgb_filename.split('.')[0].split('-')[1]) / 1000000
            f.write(f"\n{'%.6f'%timestamp} {prefix}/{rgb_filename}")

if __name__ == "__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('-r','--redwood_dir', help='first text file (format: timestamp data)')
    # parser.add_argument('second_file', help='second text file (format: timestamp data)')
    # parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    # parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    # parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)

    arg = parser.parse_args()
    rgb_img_path = osp.join(arg.redwood_dir, "rgb")
    depth_img_path = osp.join(arg.redwood_dir, "depth")
    rgb_txt_path = osp.join(arg.redwood_dir, "rgb.txt")
    depth_txt_path = osp.join(arg.redwood_dir, "depth.txt")

    get_img_txt(rgb_img_path, rgb_txt_path, "rgb")
    get_img_txt(depth_img_path, depth_txt_path, "depth")

    cmd = f"python /home/lj/Documents/QSP-SLAM/scripts/associate.py {rgb_txt_path} {depth_txt_path}"
    subprocess.check_call(cmd, shell=True)

