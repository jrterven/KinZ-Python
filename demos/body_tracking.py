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
import cmapy


def main():
    # Create Kinect object and initialize
    kin = kinz.Kinect(resolution=720, wfov=False, binned=True, framerate=30,
                    imu_sensors=False, body_tracking=True)

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
        if kin.get_frames(get_color=True, get_depth=True, get_ir=False,
                        get_sensors=False, get_body=True, get_body_index=True):
            color_data = kin.get_color_data()
            depth_data = kin.get_depth_data(align=align_frames)
            num_bodies = kin.get_num_bodies()
            bodies = kin.get_bodies()
            body_index_data = kin.get_body_index_map(returnId=True)

            print("{:d} bodies detected.".format(num_bodies))
            print("bodies:", bodies)

            # extract frames to np arrays
            depth_image = np.array(depth_data.buffer, copy=True)
            color_image = np.array(color_data.buffer, copy=True) # image is BGRA
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR) # to BGR
            body_index_image = np.array(body_index_data.buffer, copy=True)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Apply colormap on body index image
            body_index_image = cv2.applyColorMap(body_index_image*10, cmapy.cmap('tab20'))

            # Draw bodies on the RGB image
            draw_keypoints(color_image, bodies, img_type='rgb')

            # Draw bodies on the depth image
            draw_keypoints(depth_colormap, bodies, img_type='depth')

            # Resize images
            if align_frames:
                depth_colormap = cv2.resize(depth_colormap, None, fx=image_scale, fy=image_scale)
                body_index_image = cv2.resize(body_index_image, None, fx=image_scale, fy=image_scale)

            color_small = cv2.resize(color_image, None, fx=image_scale, fy=image_scale)
            size = color_small.shape[0:2]
            cv2.putText(color_small, "{0:.2f}-FPS".format(fps), (20, size[0]-20), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow('Depth', depth_colormap)
            cv2.imshow('Color', color_small)
            cv2.imshow('Body index', body_index_image)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite("color.jpg", color_image)
            print("Image saved")

        # increment frame counter and calculate FPS
        fps_count = fps_count + 1
        if (fps_count == 30):
            t = (cv2.getTickCount() - t)/cv2.getTickFrequency()
            fps = 30.0/t
            fps_count = 0

    kin.close()  # close Kinect
    cv2.destroyAllWindows()


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
                cv2.circle(img, (x, y), size, color, -1)

        # Now draw the body limbs
        for limb in body_limbs:
            conf1 = body[limb[0]]['confidence']
            conf2 = body[limb[1]]['confidence']
            # if the confidence is not None or Low, draw the body keypoint
            if conf1 >1  and conf2 > 1 :
                xy1 = (body[limb[0]][p_type]['x'],body[limb[0]][p_type]['y'])
                xy2 = (body[limb[1]][p_type]['x'],body[limb[1]][p_type]['y'])
                cv2.line(img, xy1, xy2, color, size-1)


if __name__ == "__main__":
    main()