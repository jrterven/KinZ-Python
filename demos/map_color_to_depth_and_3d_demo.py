"""
    Mapping from color to depth and 3D demo

    How to use:
    - Clic on the color image to show the corresponding point on depth image
      and the 3D coordinates
"""
import numpy as np 
import cv2
import kinz

color_coords = []

def mouse_event(event, x, y, flags, param):
    global color_coords
    if event == cv2.EVENT_LBUTTONDOWN:
        color_coords.append([x, y])

def main():
    global color_coords
    # Create Kinect object and initialize
    kin = kinz.Kinect(resolution=720, wfov=True, binned=True)

    depth_window_name = 'Mapped coordinates to Depth image'
    color_window_name = 'Click on the Color image'
    cv2.namedWindow(color_window_name, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(depth_window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(color_window_name, mouse_event)

    points_3d = []
    prev_points_3d = []
    
    while True:
        if kin.get_frames(get_color=True, get_depth=True, get_ir=False, align_depth=True):
            # get buffer data from Kinect
            color_data = kin.get_color_data()
            depth_data = kin.get_depth_data()
            ir_data = kin.get_ir_data()
            sensor_data = kin.get_sensor_data()

            # convert buffers to numpy arrays
            depth_image = np.array(depth_data.buffer, copy = True)
            color_image = np.array(color_data.buffer, copy = True)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)

            # Apply colormap on depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                            depth_image, alpha=0.03), cv2.COLORMAP_JET)

            print("Color coords:", color_coords)
            # Project color image points to depth image
            # color_coords is a list of value pairs [[xc1,yc1], [xc2,yc2], ...]
            # depth_coords have the same format as color_coords
            # depth_coords return -1 if there is no depth information
            # at the desire location, 
            # e.g. if at [x2, y2] the depth is 0 depth_coords = [[xd1, yd1], [-1, -1]]
            # we select -1 because there can not be negative pixel coordinates
            depth_coords = kin.map_coords_color_to_depth(color_coords)
            print("Depth coords:", depth_coords)

            # Backproject color image points to 3D depth camera space
            # points_3d is a list of triplets of X, Y, Z points in
            # the depth camera reference or color camera reference
            # if color_coords is [[xc1,yc1], [xc2,yc2], ...]
            # points_3d is [[X1, Y1, Z1], [X2, Y2, Z2], ...]
            # points_3d contains [0, 0, 0] if for desired color coordinates
            # there is not depth available. 
            # E.g. if at [x2, y2] the depth is 0, points_3d = [[X1, Y1, Z1], [0, 0, 0]]
            # we select 0 because depth = 0 means no depth data.
            points_3d = kin.map_coords_color_to_3D(color_coords,
                                                      depth_reference=False)

            #if points_3d != prev_points_3d:
            print("3D points:", points_3d)

            # Project 3D points image points to depth image and color image
            # points_3d is a list of value triplets [[X1, Y1, Z1], [X2, Y2, Z2], ...]
            # depth_coords2 and color_coords2 have the same format: [[x1, y1], [x2, y2], ...]
            # depth_coords return -1 if there is no depth information
            # at the desire location,
            # e.g. if at [x2, y2] the depth is 0 depth_coords = [[xd1, yd1], [-1, -1]]
            # we select -1 because there can not be negative pixel coordinates
            depth_coords2 = kin.map_coords_3d_to_depth(points_3d, depth_reference=False)
            color_coords2 = kin.map_coords_3d_to_color(points_3d, depth_reference=False)
            print("Color coords2:", color_coords2)
            print("Depth coords2:", depth_coords2)

            ## Visualization
            # Draw color image points
            for p in color_coords:                                          
                cv2.circle(color_image,(p[0], p[1]), 8, (0,0,255), -1)

            # Draw depth points
            for p in depth_coords:
                cv2.circle(depth_colormap,(p[0], p[1]), 5, (0,0,255), -1)
            
            cv2.imshow(depth_window_name, depth_colormap)
            cv2.imshow(color_window_name, color_image)

        prev_points_3d = points_3d

        k = cv2.waitKey(10) & 0xFF
        if k == ord('c'):
            color_coords = []
        if k ==27:
            break

    kin.close()  # close Kinect
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()