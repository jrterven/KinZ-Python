import numpy as np 
import cv2
import kinz

# Create Kinect object and initialize
kin = kinz.Kinect(resolution=720, wfov=True, binned=False, framerate=30)

# initialize fps counter
t = cv2.getTickCount()
fps_count = 0
fps = 0

try:
    count = 0
    while True:
        if fps_count==0:
            t = cv2.getTickCount()

        # read kinect frames. If frames available return 1
        if kin.get_frames(get_color=True, get_depth=True, get_ir=False):
            count += 1

            # Get the pointcloud data and convert to Numpy (512 x 512 x 3, int16)
            pointcloud_data = kin.get_pointcloud()
            pointcloud_np = np.array(pointcloud_data, copy = True)

            # Get the pointcloud color and conver to Numpy (512 x 512 x 3, uint8)
            pointcloudcolor_data = kin.get_pointcloud_color()
            pointcloudcolor_np = np.array(pointcloudcolor_data, copy = True)

        # increment frame counter and calculate FPS
        fps_count = fps_count + 1
        if (fps_count == 30):
            t = (cv2.getTickCount() - t)/cv2.getTickFrequency()
            fps = 30.0/t
            fps_count = 0
            print(f"FPS:{fps:.2f}")

finally:
    print("Closing Kinect")
    kin.close()

