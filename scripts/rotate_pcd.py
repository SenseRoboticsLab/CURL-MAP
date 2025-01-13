import os
import numpy as np
import open3d as o3d
# Assuming the provided code is saved in a file named line_mesh_module.py
from line_mesh_module import LineMesh


def custom_draw_geometry_with_rotation(pcd):
	def rotate_view(vis):
		ctr = vis.GetViewControl()
		ctr.Rotate(10.0, 0.0)
		return False
	vis = Visualizer()
	vis.CreateWindow()
	vis.AddGeometry(pcd)
	vis.RegisterAnimationCallback(rotate_view)
	vis.Run()
	vis.DestroyWindow()

def custom_draw_geometry_with_custom_fov(pcd, fov_step):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    ctr = vis.get_view_control()
    para = vis.get_view_control().convert_to_pinhole_camera_parameters()
    print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
    ctr.change_field_of_view(step=fov_step)
    print("Field of view (after changing) %.2f" % ctr.get_field_of_view())
    vis.run()
    vis.destroy_window()
    
if __name__ == '__main__':
    # Read point cloud
    pcd = o3d.io.read_point_cloud(
        "/media/zkc/EAGET/icra_24/KITTI_map_06/float_degree_10_rso_2/whole_map_points.txt", format='xyz')
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Visualization
    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd)
    # vis.run()
    # vis.destroy_window()
    custom_draw_geometry_with_custom_fov(pcd,180)
