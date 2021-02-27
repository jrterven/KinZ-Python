"""
    Mapping from depth to color demo

    How to use:
    - Clic on the depth image to show the corresponding point on color image
      and the 3D coordinates
"""
import numpy as np 
import cv2
import kinz

depth_points = []

def mouse_event(event, x, y, flags, param):
    global depth_points
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_points.append((x, y))

def main():
    global depth_points
    # Create Kinect object and initialize
    kin = kinz.Kinect(resolution=720, wfov=True, binned=True)
    
    depth_window_name = 'Click on the Depth image'
    color_window_name = 'Mapped coordinates in Color'
    cv2.namedWindow(color_window_name, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(depth_window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(depth_window_name, mouse_event)

    points_3d = []
    prev_points_3d = []

    while True:
        if kin.get_frames(get_color=True, get_depth=True, get_ir=False, align_depth=True):
            color_data = kin.get_color_data()
            depth_data = kin.get_depth_data()

            depth_image = np.array(depth_data.buffer, copy = True)
            color_image = np.array(color_data.buffer, copy = True)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)

            # Project depth image points to color image points
            # depth_points is a list of value pairs [[xd1,yd1], [xd2,yd2], ...]
            # color_points have the same format as depth_points
            # map_coords_depth_to_color return [-1, -1] if there is no depth information
            # at the desire location, or the transformation failed.
            color_points = kin.map_coords_depth_to_color(depth_points)

            # Backproject depth image points to 3D depth camera space
            # points_3d is a list of triplets of X, Y, Z points in
            # the depth camera reference
            # if depth_coords is [[x1,y1], [x2,y2], ...]
            # points_3d is [[X1, Y1, Z1], [X2, Y2, Z2], ...]
            # points_3d contains [0, 0, 0] if for desired depth coordinates
            # there is not depth available. 
            # E.g. if at [x2, y2] the depth is 0, points_3d = [[X1, Y1, Z1], [0, 0, 0]]
            # we select 0 because depth = 0 means no depth data.
            points_3d = kin.map_coords_depth_to_3D(depth_points)

            if points_3d != prev_points_3d:
                print("3D point:", points_3d)

            # Draw mapped points in color image
            for p in color_points:                                          
                cv2.circle(color_image,(p[0], p[1]), 8, (0,0,255), -1)

            # Apply colormap on depth image for visualization
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Draw depth points
            for p in depth_points:
                cv2.circle(depth_colormap,(p[0], p[1]), 5, (0,0,255), -1)

            # Resize color image for visualization purposes
            color_small = cv2.resize(color_image, None, fx=0.5, fy=0.5)

            cv2.imshow(depth_window_name, depth_colormap)
            cv2.imshow(color_window_name, color_small)
        prev_points_3d = points_3d

        k = cv2.waitKey(1) & 0xFF
        if k == ord('c'):
            depth_points = []
        if k ==27:
            break

    kin.close()  # close Kinect
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()