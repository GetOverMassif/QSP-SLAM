# import vispy

import numpy as np
import vispy.scene
from vispy.scene import visuals

import os.path as osp
import open3d as o3d
# import pcl
# import numpy as np


def read_pcd(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    # pcd_np_points = np.asarray(pcd.points)

    pcd_data = {}

    pcd_data['pos'] = np.asarray(pcd.points)
    pcd_data['color'] = np.asarray(pcd.colors)

    return pcd_data

pcd_file = "/home/lj/Documents/codes/QSP-SLAM/map/icl_nuim/office/1/map.pcd"

#
# Make a canvas and add simple view
#
canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
view = canvas.central_widget.add_view()

pcd_data = read_pcd(pcd_file)


# generate data
pos = np.random.normal(size=(100000, 3), scale=0.2)

# # one could stop here for the data generation, the rest is just to make the
# # data look more interesting. Copied over from magnify.py
# centers = np.random.normal(size=(50, 3))
# indexes = np.random.normal(size=100000, loc=centers.shape[0] / 2,
#                            scale=centers.shape[0] / 3)
# indexes = np.clip(indexes, 0, centers.shape[0] - 1).astype(int)

# scales = 10**(np.linspace(-2, 0.5, centers.shape[0]))[indexes][:, np.newaxis]
# pos *= scales
# pos += centers[indexes]

# pos = pcd_data[:,:3]

# print(type(pos))
# print(pos.shape)

pos = pcd_data['pos']
face_colors = pcd_data['color']

symbols = np.random.choice(['o', '^'], len(pos))


# face_colors = np.random.rand(len(pos), 4)

# print(f"face_color = {face_colors}")

# create scatter object and fill in the data
scatter = visuals.Markers()
# scatter.set_data(pos, edge_width=0, face_color=(0, 1, 1, .5), size=5, symbol=symbols)
scatter.set_data(pos, edge_width=0, face_color=face_colors, size=5, symbol=symbols)

view.add(scatter)

view.camera = 'turntable'  # or try 'arcball'

# add a colored 3D axis for orientation
axis = visuals.XYZAxis(parent=view.scene)

if __name__ == '__main__':
    import sys
    if sys.flags.interactive != 1:
        vispy.app.run()