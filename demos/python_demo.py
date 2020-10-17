import numpy as np 
import cv2 as cv
import cmapy
import kinz


def main():
    # Create Kinect object and initialize
    kin = kinz.Kinect(resolution=720, wfov=True, binned=True,
                    framerate=30, imuSensors=True,
                    bodyTracking=True)

    while True:
        # Capture kinect frames
        if kin.getFrames(getColor=True, getDepth=True,
                        getIR=True, getSensors=True,
                        getBody=True, getBodyIndex=True):
            # Read cameras data
            color_data = kin.getColorData()
            depth_data = kin.getDepthData(align=False)
            ir_data = kin.getIRData()
            
            # Read sensor data
            sensor_data = kin.getSensorData()
            num_bodies = kin.getNumBodies()
            
            # Read body tracking data
            bodies = kin.getBodies()
            body_index_data = kin.getBodyIndexMap(returnId=True)

            # Read pointcloud data
            pointcloud_data = kin.getPointCloud()
            pointcloudcolor_data = kin.getPointCloudColor()

            # Copy pointcloud data to numpy arrays
            pointcloud_np = np.array(pointcloud_data)
            pointcloudcolor_np = np.array(pointcloudcolor_data)

            # Copy images data to np arrays
            depth_image = np.array(depth_data.buffer)
            color_image = np.array(color_data.buffer)
            ir_image = np.array(ir_data.buffer)
            body_index_image = np.array(body_index_data.buffer)

            # Draw bodies on the RGB image
            draw_keypoints(color_image, bodies, img_type='rgb')

            # Draw bodies on the depth image
            depth_colormap = cv.applyColorMap(cv.convertScaleAbs(
                                            depth_image,
                                            alpha=0.03),
                                            cv.COLORMAP_JET)
            draw_keypoints(depth_colormap, bodies, img_type='depth')

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
            body_index_image = cv.applyColorMap(body_index_image*10,
                                        cmapy.cmap('tab20'))
            cv.imshow('Color', color_image)
            cv.imshow('Depth', depth_colormap)
            cv.imshow('IR', ir_image)
            cv.imshow('Body index', body_index_image)

        # Exit with ESC
        k = cv.waitKey(1) & 0xFF
        if k == 27:
            break

    kin.close()  # close Kinect
    cv.destroyAllWindows()

def draw_keypoints(img, bodies, img_type='rgb', size=5):
    colors = [
        [255, 10, 0], [255, 85, 0], [255, 170, 0], [255, 255, 0], [170, 255, 0],
        [85, 255, 0], [0, 255, 0], [0, 255, 85], [0, 255, 170], [0, 255, 255],
        [0, 170, 255], [0, 85, 255], [0, 0, 255], [85, 0, 255], [170, 0, 255],
        [255, 0, 255], [255, 0, 170], [255, 0, 85], [255, 0, 0], [170, 0, 255],
        [0, 170, 255], [0, 85, 255], [0, 0, 255], [85, 0, 255], [170, 0, 255],
        [0, 170, 255], [0, 85, 255], [0, 0, 255], [85, 0, 255], [170, 0, 255]]

    body_parts = ["Pelvis", "Spine_navel", "Spine_chest", 
                "Neck", "Clavicle_left", "Shoulder_left",
                "Elbow_left", "Wrist_left", "Hand_left",
                "Handtip_left", "Thumb_left", "Clavicle_right",
                "Shoulder_right", "Elbow_right", "Wrist_right",
                "Hand_right", "Handtip_right", "Thumb_right",
                "Hip_left", "Knee_left", "Ankle_left",
                "Foot_left", "Hip_right", "Knee_right",
                "Ankle_right", "Foot_right", "Head",
                "Nose", "Eye_left", "Ear_left",
                "Eye_right", "Ear_right"]
    body_limbs = [("Head", "Neck"), ("Neck", "Spine_chest"),
                  ("Spine_chest", "Spine_navel"), ("Spine_navel", "Pelvis"),
                  ("Neck", "Clavicle_left"), ("Neck", "Clavicle_right"), 
                  ("Clavicle_left", "Shoulder_left"), ("Clavicle_right", "Shoulder_right"),
                  ("Shoulder_left", "Elbow_left"), ("Shoulder_right", "Elbow_right"),
                  ("Elbow_left", "Wrist_left"), ("Elbow_right", "Wrist_right"),
                  ("Wrist_left", "Thumb_left"), ("Wrist_right", "Thumb_right"),
                  ("Wrist_left", "Hand_left"), ("Wrist_right", "Hand_right"),
                  ("Hand_left", "Handtip_left"), ("Hand_right", "Handtip_right"),
                  ("Pelvis", "Hip_left"), ("Pelvis", "Hip_right"),
                  ("Hip_left", "Knee_left"), ("Hip_right", "Knee_right"),
                  ("Knee_left", "Ankle_left"), ("Knee_right", "Ankle_right"),
                  ("Ankle_left", "Foot_left"), ("Ankle_right", "Foot_right")]
    #color = [255, 255, 255]
    if img_type == 'rgb':
        p_type = 'position2d-rgb'
    else:
        p_type = 'position2d-depth'
    for body in bodies:
        # Get the body ID to select a unique color
        body_id = body['id']
        color = colors[body_id]
        
        # for each body part
        for part in body_parts:
            # Get the prediction confidence
            confidence = body[part]['confidence']

            # if the confidence is not None or Low, draw the body keypoint
            if confidence > 1:
                x = body[part][p_type]['x']
                y = body[part][p_type]['y']
                cv.circle(img, (x, y), size, color, -1)

        # Now draw the body limbs
        for limb in body_limbs:
            conf1 = body[limb[0]]['confidence']
            conf2 = body[limb[1]]['confidence']
            # if the confidence is not None or Low, draw the body keypoint
            if conf1 >1  and conf2 > 1 :
                xy1 = (body[limb[0]][p_type]['x'],body[limb[0]][p_type]['y'])
                xy2 = (body[limb[1]][p_type]['x'],body[limb[1]][p_type]['y'])
                cv.line(img, xy1, xy2, color, size-1)

if __name__ == "__main__":
    main()