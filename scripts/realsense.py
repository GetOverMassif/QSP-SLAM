# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import time
# import os

# pipeline = rs.pipeline()

# #Create a config并配置要流​​式传输的管道
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# profile = pipeline.start(config)

# depth_sensor = profile.get_device().first_depth_sensor()
# depth_scale = depth_sensor.get_depth_scale()
# print("Depth Scale is: " , depth_scale)

# align_to = rs.stream.color
# align = rs.align(align_to)

# # 按照日期创建文件夹
# save_path = os.path.join(os.getcwd(), "out", time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()))
# os.mkdir(save_path)
# os.mkdir(os.path.join(save_path, "color"))
# os.mkdir(os.path.join(save_path, "depth"))

# # 保存的图片和实时的图片界面
# cv2.namedWindow("live", cv2.WINDOW_AUTOSIZE)
# cv2.namedWindow("save", cv2.WINDOW_AUTOSIZE)
# saved_color_image = None # 保存的临时图片
# saved_depth_mapped_image = None
# saved_count = 0

# # 主循环
# try:
#     while True:
#         frames = pipeline.wait_for_frames()

#         aligned_frames = align.process(frames)

#         aligned_depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()

#         if not aligned_depth_frame or not color_frame:
#             continue
        
#         depth_data = np.asanyarray(aligned_depth_frame.get_data(), dtype="float16")
#         depth_image = np.asanyarray(aligned_depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())
#         depth_mapped_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
#         cv2.imshow("live", np.hstack((color_image, depth_mapped_image)))
#         key = cv2.waitKey(30)

#         # s 保存图片
#         if key & 0xFF == ord('s'):
#             saved_color_image = color_image
#             saved_depth_mapped_image = depth_mapped_image

#             # 彩色图片保存为png格式
#             cv2.imwrite(os.path.join((save_path), "color", "{}.png".format(saved_count)), saved_color_image)
#             # 深度信息由采集到的float16直接保存为npy格式
#             np.save(os.path.join((save_path), "depth", "{}".format(saved_count)), depth_data)
#             saved_count+=1
#             cv2.imshow("save", np.hstack((saved_color_image, saved_depth_mapped_image)))

#         # q 退出
#         if key & 0xFF == ord('q') or key == 27:
#             cv2.destroyAllWindows()
#             break    
# finally:
#     pipeline.stop()

# import pyrealsense2 as rs

# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
# cfg = pipeline.start(config)
# device = cfg.get_device()
# name = device.get_info(rs.camera_info.name)
# print(name)
# profile = cfg.get_stream(rs.stream.depth)
# profile1 = cfg.get_stream(rs.stream.color)
# intr = profile.as_video_stream_profile().get_intrinsics()
# intr1 = profile1.as_video_stream_profile().get_intrinsics()
# extrinsics = profile1.get_extrinsics_to(profile)
# print(extrinsics)
# print("深度传感器内参：", intr)
# print("RGB相机内参:", intr1)


import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        cv2.imshow('color_image', color_image)

        # the size of color_frame is (720,1280,3)
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        # depth image is 1 channel, color is 3 channels
        # depth_image_3d shape is (720,1280,3)
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        # the size of bg_removed is (720,1280,3)
        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()