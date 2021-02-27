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
kin = kinz.Kinect(resolution=720, wfov=True, binned=True, framerate=30, imu_sensors=True)

# Get depth aligned with color?
align_frames = False
image_scale = 0.5    # visualized image scale

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
        sensor_data = kin.get_sensor_data()

        # extract frames to np arrays
        depth_image = np.array(depth_data.buffer, copy = True)
        color_image = np.array(color_data.buffer, copy = True) # image is BGRA
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR) # to BGR
        ir_image = np.array(ir_data.buffer, copy = True)

        print('Depth shape, type, timestamp:', depth_image.shape, depth_image.dtype, depth_data.timestamp_nsec)
        print('Color shape, type, timestamp:', color_image.shape, color_image.dtype, color_data.timestamp_nsec)
        print('IR shape, type, timestamp:', ir_image.shape, ir_image.dtype, ir_data.timestamp_nsec)
        print('Depth range values:', np.amin(depth_image), np.amax(depth_image))
        print('IR range values:', np.amin(ir_image), np.amax(ir_image))
        print('Temperature: {:.1f} Celsius'.format(sensor_data.temperature))
        print('AccX, AccY, AccZ, AccTime = {:.3f}, {:.3f}, {:.3f}, {:d}'.format(sensor_data.acc_x,
                                                                                sensor_data.acc_y,
                                                                                sensor_data.acc_z,
                                                                                sensor_data.acc_timestamp_usec))
        print('GyroX, GyroY, GyroZ. GyroTime = {:.3f}, {:.3f}, {:.3f}, {:d}'.format(sensor_data.gyro_x,
                                                                                    sensor_data.gyro_y,
                                                                                    sensor_data.gyro_z,
                                                                                    sensor_data.gyro_timestamp_usec))

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Reescale IR image values
        ir_image = cv2.convertScaleAbs(ir_image, alpha=0.04)

        # Resize images
        if align_frames:
          depth_colormap = cv2.resize(depth_colormap, None, fx=image_scale, fy=image_scale)

        color_small = cv2.resize(color_image, None, fx=image_scale, fy=image_scale)
        size = color_small.shape[0:2]
        cv2.putText(color_small, "{0:.2f}-FPS".format(fps), (20, size[0]-20), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow('Depth', depth_colormap)
        cv2.imshow('Color', color_small)
        cv2.imshow('IR', ir_image)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    elif k == ord('s'):
        cv2.imwrite("color.jpg", color_image)
        cv2.imwrite("depth.png", depth_image)
        print("Image saved")

    # increment frame counter and calculate FPS
    fps_count = fps_count + 1
    if (fps_count == 30):
      t = (cv2.getTickCount() - t)/cv2.getTickFrequency()
      fps = 30.0/t
      fps_count = 0

kin.close()  # close Kinect
cv2.destroyAllWindows()
