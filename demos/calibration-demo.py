"""
    Calibration demo

    Shows how to extract the instrinsic and extrinsic parameters.
"""
import numpy as np 
import cv2
import kinz

# Create Kinect object and initialize
kin = kinz.Kinect(resolution=1080, wfov=True, binned=True)

# get calibration objects
depth_calib = kin.get_depth_calibration()
color_calib = kin.get_color_calibration()

# extract calibration parameters
depth_size = depth_calib.get_size() # image size
color_size = color_calib.get_size()

# Intrinsics is a 3x3 if extended=False or 4x4 if extended=True
depth_intrinsics = depth_calib.get_intrinsics_matrix(extended=False)
color_intrinsics = color_calib.get_intrinsics_matrix(extended=False)

# Distortion is a 8x1 numpy vector: k1,k2,p1,p2,k3,k4,k5,k6
depth_dist = depth_calib.get_distortion_params()
color_dist = color_calib.get_distortion_params()

# Rotation is a 3x3 Rotation matrix wrt depth camera
depth_rot = depth_calib.get_rotation_matrix()
color_rot = color_calib.get_rotation_matrix()

# Translation is a 3x1 translation vector wrt depth camera
depth_trans = depth_calib.get_translation_vector()
color_trans = color_calib.get_translation_vector()

# Pose is a 4x4 pose matrix wrt depth camera
depth_pose = depth_calib.get_camera_pose()
color_pose = color_calib.get_camera_pose()

print("----- DEPTH CALIBRATION ------")
print("depth_size:", type(depth_size), depth_size)
print("depth_intrinsics:", type(depth_intrinsics))
print(depth_intrinsics)
print("depth_dist:", type(depth_dist), depth_dist)
print("depth_rot:", type(depth_rot))
print(depth_rot)
print("depth_trans:", type(depth_trans))
print(depth_trans)
print("depth_pose:", type(depth_pose))
print(depth_pose)
print("--------------------------------")

print("----- COLOR CALIBRATION ------")
print("color_size:", type(color_size), color_size)
print("color_intrinsics:", type(color_intrinsics))
print(color_intrinsics)
print("color_dist:", type(color_dist), color_dist)
print("color_rot:", type(color_rot))
print(color_rot)
print("color_trans:", type(color_trans))
print(color_trans)
print("color_pose:", type(color_pose))
print(color_pose)
print("--------------------------------")

kin.close() 