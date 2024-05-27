
import open3d as o3d
import numpy as np
import os

def save_view_point(vis, filename = "viewpoint.json"):
    cam_param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(filename, cam_param)


def load_view_point(vis, filename = "viewpoint.json"):
    ctr = vis.get_view_control()
    cam_param = o3d.io.read_pinhole_camera_parameters(filename)
    ctr.convert_from_pinhole_camera_parameters(cam_param)

def getCoordinateAxis(T = np.eye(4), frameSize = 0.5):
    gCoordinateAxis = o3d.geometry.TriangleMesh.create_coordinate_frame().\
            scale(frameSize, np.array([0., 0., 0.]))
    return gCoordinateAxis.transform(T)

def get_cube_lineset(xyz_max, xyz_min, color, xyz_centroid = [0,0,0]):
    [x1, y1, z1] = xyz_max
    [x2, y2, z2] = xyz_min
    d = xyz_centroid
    points = [[x1,y1,z2],[x1,y2,z2],[x2,y2,z2],[x2,y1,z2],[x1,y1,z1],[x1,y2,z1],[x2,y2,z1],[x2,y1,z1]]
    points = [[(p[i] - d[i]) for i in range(3)] for p in points]
    pointPairs = [[0,1],[1,2],[2,3],[3,0],[0,4],[1,5],[2,6],[3,7],[4,5],[5,6],[6,7],[7,4]]
    cube_lineset = o3d.geometry.LineSet()
    cube_lineset.lines = o3d.utility.Vector2iVector(pointPairs)
    cube_lineset.paint_uniform_color(color)
    cube_lineset.points = o3d.utility.Vector3dVector(points)
    return cube_lineset

def set_view(vis, dist = 100., theta = np.pi/6.):
    vis_ctr = vis.get_view_control()
    cam = vis_ctr.convert_to_pinhole_camera_parameters()
    # world to eye
    cam.extrinsic = np.array([[1., 0., 0., 0.],
                              [0., np.cos(theta), -np.sin(theta), 0.],
                              [0., np.sin(theta), np.cos(theta), dist],
                              [0., 0., 0., 1.]])
    vis_ctr.convert_from_pinhole_camera_parameters(cam)

def set_view_by_qt(vis, Rq = [0,0,0,1], t = [0,0,2]):
    vis_ctr = vis.get_view_control()
    cam = vis_ctr.convert_to_pinhole_camera_parameters()
    # world to eye
    T = np.eye(4)
    rotation_matrix = R.from_quat(Rq).as_matrix()  # qx qy qz qw
    T[:3, :4] = np.concatenate((rotation_matrix,np.array(t).reshape(3,1)),axis=1)
    cam.extrinsic = np.linalg.inv(T)
    vis_ctr.convert_from_pinhole_camera_parameters(cam)