"""
    Calibration demo

    Shows how to extract the instrinsic and extrinsic parameters.
"""
import numpy as np 
import cv2
import pyk4

# Create Kinect object and initialize
kin = pyk4.Kinect(resolution=2160, wfov=True, binned=True)

# get calibration objects
depth_calib = kin.getDepthCalibration()
color_calib = kin.getColorCalibration()

# extract calibration parameters
depth_size = depth_calib.getSize() # image size
color_size = color_calib.getSize()

# Intrinsics is a 3x3 if extended=False or 4x4 if extended=True
depth_intrinsics = depth_calib.getIntrinsicsMatrix(extended=False)
color_intrinsics = color_calib.getIntrinsicsMatrix(extended=False)

# Distortion is a 8x1 numpy vector: k1,k2,p1,p2,k3,k4,k5,k6
depth_dist = depth_calib.getDistortionParams()
color_dist = color_calib.getDistortionParams()

# Rotation is a 3x3 Rotation matrix wrt depth camera
depth_rot = depth_calib.getRotationMatrix()
color_rot = color_calib.getRotationMatrix()

# Translation is a 3x1 translation vector wrt depth camera
depth_trans = depth_calib.getTranslationVector()
color_trans = color_calib.getTranslationVector()

# Pose is a 4x4 pose matrix wrt depth camera
depth_pose = depth_calib.getCameraPose()
color_pose = color_calib.getCameraPose()

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