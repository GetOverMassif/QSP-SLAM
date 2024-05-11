import open3d as o3d
import numpy as np

T = np.eye(4)

T[:3,3] = np.array([0.2,0.1,-0.3])

pcd = o3d.io.read_point_cloud("/home/lj/Documents/codes/QSP-SLAM/map/redwood/01053/map.pcd")
pcd.transform(T)
o3d.io.write_point_cloud("/home/lj/Documents/codes/QSP-SLAM/map/redwood/01053/map2.pcd", pcd)