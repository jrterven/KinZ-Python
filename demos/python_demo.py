import numpy as np 
import cv2 as cv
import cmapy
import kinz

def main():
  # Create Kinect object and initialize
  kz = kinz.Kinect(resolution=720, wfov=True, binned=True,
                    framerate=30, imuSensors=True,
                    bodyTracking=True)

  while True:
    # Capture kinect frames
    if kz.getFrames(getColor=True, getDepth=True,
                     getIR=True, getSensors=True,
                     getBody=True, getBodyIndex=True):
      # Read cameras data
      color_data = kz.getColorData()
      depth_data = kz.getDepthData(align=False)
      ir_data = kz.getIRData()
      
      # Read sensor data
      sensor_data = kz.getSensorData()
      num_bodies = kz.getNumBodies()
      
      # Read body tracking data
      bodies = kz.getBodies()
      body_idx_data = kz.getBodyIndexMap(returnId=True)

      # Read pointcloud data
      pointcloud_data = kz.getPointCloud()
      pointcloudcolor_data = kz.getPointCloudColor()

      # Copy pointcloud data to numpy arrays
      pointcloud_np = np.array(pointcloud_data)
      pointcloudcolor_np = np.array(pointcloudcolor_data)

      # Copy images data to np arrays
      depth_image = np.array(depth_data.buffer)
      color_image = np.array(color_data.buffer)
      ir_image = np.array(ir_data.buffer)
      body_idx_img = np.array(body_idx_data.buffer)

      # Draw bodies on the RGB image
      draw_keypoints(color_image, bodies, img_type='rgb')

      # Draw bodies on the depth image
      depth_cmap = cv.applyColorMap(cv.convertScaleAbs(
                                      depth_image,
                                      alpha=0.03),
                                      cv.COLORMAP_JET)
      draw_keypoints(depth_cmap, bodies, img_type='depth')

      # Display sensor data
      print(f"Temp: {sensor_data.temperature:.1f} Celsius")
      print(f"AccX:{sensor_data.acc_x:.2f}")
      print(f"AccY:{sensor_data.acc_y:.2f}")
      print(f"AccZ:{sensor_data.acc_z:.2f}")
      print(f"AccT:{sensor_data.acc_timestamp_usec:.2f}")
      print(f"GyroX:{sensor_data.gyro_x:.2f}")
      print(f"GyroY:{sensor_data.gyro_y:.2f}")
      print(f"GyroZ:{sensor_data.gyro_z:.2f}")
      print(f"GyroT:{sensor_data.gyro_timestamp_usec:.2f}")

      # Display images
      color_image = cv.cvtColor(color_image,
                                cv.COLOR_BGRA2BGR)
      body_index_image = cv.applyColorMap(body_idx_img*10,
                                  cmapy.cmap('tab20'))
      cv.imshow('Color', color_image)
      cv.imshow('Depth', depth_cmap)
      cv.imshow('IR', ir_image)
      cv.imshow('Body index', body_index_image)

    # Exit with ESC
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

  kz.close()  # close Kinect
  cv.destroyAllWindows()

if __name__ == "__main__":
    main()