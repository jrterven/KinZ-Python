import numpy as np
import open3d as o3d

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("/datasets2/Dropbox/Projects/KinZ/KinZ-Python/mypointcloud.ply")
print(pcd)
pcd_np = np.asarray(pcd.points)
colors_np = np.asarray(pcd.colors)
print("pcd_np:", pcd_np.shape, pcd_np.dtype, np.amin(pcd_np), np.amax(pcd_np))
print("colors_np:", colors_np.shape, colors_np.dtype, np.amin(colors_np), np.amax(colors_np))

vis = o3d.visualization.Visualizer()
vis.create_window("PointCloud")

while True:
    vis.add_geometry(pcd)
    geometry_added = True

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
