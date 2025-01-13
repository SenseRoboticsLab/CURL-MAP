import os
import numpy as np
import open3d as o3d
# Assuming the provided code is saved in a file named line_mesh_module.py
from line_mesh_module import LineMesh

def self_define_rotation(pcd):
    center = np.mean(pcd.points, 0)
    iteration_times = 1000
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    # vis.run()
    while True:
        R = pcd.get_rotation_matrix_from_xyz((0, 0, np.pi * 2 / iteration_times))
        pcd.rotate(R, center=(center[0], center[1], center[2]))
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

if __name__ == '__main__':
    # Get the list of all files in the directory
    folder_path = "/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/KITTI_trajectory_06/"
    files = [f for f in os.listdir(folder_path) if os.path.isfile(
        os.path.join(folder_path, f))]

    # Filter files based on the pattern
    files = [f for f in files if f.endswith("_T_w_j.txt")]

    # Sort the files based on the starting number
    files.sort(key=lambda x: int(x.split('_')[0]))

    all_points = []

    for file in files:
        file_path = os.path.join(folder_path, file)

        # Read the file using numpy
        data = np.loadtxt(file_path)

        # Get the 1-3 rows and 4th column
        points = data[0:3, 3].reshape(-1, 3)
        all_points.extend(points)

    # Create the trajectory as a wide line using LineMesh
    # Adjust the radius value for desired line width
    trajectory_line = LineMesh(all_points, radius=0.5)

    # Read point cloud
    pcd = o3d.io.read_point_cloud(
        "/media/zkc/EAGET/icra_24/KITTI_map_06/float_degree_10_rso_5/whole_map_points.txt", format='xyz')
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Visualization
    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd)
    # trajectory_line.add_line(vis)
    # vis.run()
    # vis.destroy_window()
    self_define_rotation(pcd)
