"""
  Kinect for Azure Pointcloud demo

    This demo uses the Open3D library to visualize the pointcloud.
    It saves a Pointcloud in a PLY file.
"""
from datetime import datetime
import numpy as np 
import os
import kinz
import open3d as o3d

# Output directory to save the pointcloud
POINTCLOUD_OUTPUT_DIR = "."

# Create Kinect object and initialize
kin = kinz.Kinect(resolution=1080, wfov=True, binned=True, framerate=30)

try:
    count = 0
    geometry_added = False
    vis = o3d.visualization.Visualizer()
    vis.create_window("PointCloud")
    pcd = o3d.geometry.PointCloud()

    while True:
        dt0 = datetime.now()

        # read kinect frames. If frames available return 1
        if kin.get_frames(get_color=True, get_depth=True, get_ir=False):
            count += 1

            # Get the pointcloud data and convert to Numpy (512 x 512 x 3, int16)
            pointcloud_data = kin.get_pointcloud()
            pointcloud_np = np.array(pointcloud_data, copy = True)

            # Get the pointcloud color and conver to Numpy (512 x 512 x 3, uint8)
            pointcloudcolor_data = kin.get_pointcloud_color()
            pointcloudcolor_np = np.array(pointcloudcolor_data, copy = True)

            # Every 10 pointclouds, print the statistics
            if count % 10 == 0:
                print('Pointcloud shape and type:', pointcloud_np.shape, pointcloud_np.dtype)
                print('PC min:{:d}, max:{:d}'.format(np.amin(pointcloud_np), np.amax(pointcloud_np)))

                print('Pointcloudcolor shape and type:', pointcloudcolor_np.shape, pointcloudcolor_np.dtype)

            # Prepare the Pointcloud data for Open3D pointcloud data
            xyz = np.zeros((pointcloud_np.shape[0]*pointcloud_np.shape[1], 3), dtype=np.float32)
            x = pointcloud_np[:, :, 0]  # extract the X, Y, Z matrices
            y = pointcloud_np[:, :, 1]
            z = pointcloud_np[:, :, 2]
            xyz[:, 0] = np.reshape(x, -1) # convert to vectors
            xyz[:, 1] = np.reshape(y, -1)
            xyz[:, 2] = np.reshape(z, -1)
            pcd.points = o3d.utility.Vector3dVector(xyz)

            # Prepare the Pointcloud colors for Open3D pointcloud colors
            colors = np.zeros((pointcloudcolor_np.shape[0]*pointcloudcolor_np.shape[1], 3), dtype=np.float32)
            blue = pointcloudcolor_np[:, :, 0] # extract the blue, green , and red components
            green = pointcloudcolor_np[:, :, 1]
            red = pointcloudcolor_np[:, :, 2]
            colors[:, 0] = np.reshape(red, -1) / 255.0 # convert to vectors and scale to 0-1
            colors[:, 1] = np.reshape(green, -1) / 255.0
            colors[:, 2] = np.reshape(blue, -1) / 255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # Render the pointcloud
            if not geometry_added:
                vis.add_geometry(pcd)
                geometry_added = True
            
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            process_time = datetime.now() - dt0
            if count % 10 == 0:
                print("FPS = {0}".format(1/process_time.total_seconds()))

            if count == 30:
                pointcloud_ouput_path = os.path.join(POINTCLOUD_OUTPUT_DIR, "pointcloud_{:d}.ply".format(count))
                print("Saving Pointcloud in {}".format(pointcloud_ouput_path))
                
                kin.savePointCloud(pointcloud_ouput_path)
                break

finally:
    print("Closing Kinect")
    kin.close()

