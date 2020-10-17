import numpy as np 
import cv2
import pyk4

# Create Kinect object and initialize
kin = pyk4.Kinect(resolution=720, wfov=True, binned=True,
                  framerate=30, imuSensors=True)

while True:
    # Capture kinect frames
    if kin.getFrames(getColor=True, getDepth=True,
                     getIR=True, getSensors=True):
        color_data = kin.getColorData()
        depth_data = kin.getDepthData(align=False)
        ir_data = kin.getIRData()
        s_data = kin.getSensorData()

        # Extract data to np arrays
        depth_image = np.array(depth_data.buffer, copy=True)
        color_image = np.array(color_data.buffer, copy=True)
        color_image = cv2.cvtColor(color_image,
                                   cv2.COLOR_BGRA2BGR) # to BGR
        ir_image = np.array(ir_data.buffer, copy = True)

        # Display sensor data
        print(f"Temperature: {s_data.temperature:.1f} Celsius")
        print(f"AccX, AccY, AccZ, AccTime = {:.3f}, {:.3f}, {:.3f}, {:d}")".format(sensor_data.acc_x,
                                                                                s_data.acc_y,
                                                                                s_data.acc_z,
                                                                                s_data.acc_timestamp_usec))
        print('GyroX, GyroY, GyroZ. GyroTime = {:.3f}, {:.3f}, {:.3f}, {:d}'.format(s_data.gyro_x,
                                                                                    s_data.gyro_y,
                                                                                    s_data.gyro_z,
                                                                                    s_data.gyro_timestamp_usec))

        # Display images
        cv2.imshow('Depth', depth_image)
        cv2.imshow('Color', color_image)
        cv2.imshow('IR', ir_image)

    # Exit with ESC
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

kin.close()  # close Kinect
cv2.destroyAllWindows()
