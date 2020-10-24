"""
  Kinect for Azure Color, Depth and Infrared streaming in Python

  Supported resolutions:
  720:  1280 x 720 @ 30 FPS  binned depth
  1080: 1920 x 1080 @ 30 FPS binned depth
  1440: 2560 x 1440 @ 30 FPS binned depth
  1535: 2048 x 1536 @ 30 FPS binned depth
  2160: 3840 x 2160 @ 30 FPS binned depth
  3072: 4096 x 3072 @ 15 FPS binned depth
"""
import numpy as np 
import cv2
import kinz

# Create Kinect object and initialize
kin = kinz.Kinect(resolution=4096, wfov=True, binned=False, framerate=30, imu_sensors=False)

# Get depth aligned with color?
align_frames = False

# initialize fps counter
t = cv2.getTickCount()
fps_count = 0
fps = 0

while True:
    if fps_count==0:
      t = cv2.getTickCount()

    # read kinect frames. If frames available return 1
    if kin.get_frames(get_color=True, get_depth=True, get_ir=True, get_sensors=True):
        color_data = kin.get_color_data()
        depth_data = kin.get_depth_data(align=align_frames)
        ir_data = kin.get_ir_data()

        # extract frames to np arrays
        depth_image = np.array(depth_data.buffer, copy = True)
        color_image = np.array(color_data.buffer, copy = True) # image is BGRA
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR) # to BGR
        ir_image = np.array(ir_data.buffer, copy = True)


    # increment frame counter and calculate FPS
    fps_count = fps_count + 1
    if (fps_count == 30):
      t = (cv2.getTickCount() - t)/cv2.getTickFrequency()
      fps = 30.0/t
      fps_count = 0
      print(f"FPS:{fps:.2f}")

kin.close()  # close Kinect
cv2.destroyAllWindows()
