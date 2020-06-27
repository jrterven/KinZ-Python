"""
    Mapping from color to depth and 3D demo

    How to use:
    - Clic on the color image to show the corresponding point on depth image
"""
import numpy as np 
import cv2
import pyk4

color_coords = []

def mouse_event(event, x, y, flags, param):
    global color_coords
    if event == cv2.EVENT_LBUTTONDOWN:
        color_coords.append([x, y])

def main():
    global color_coords
    # Create Kinect object and initialize
    kin = pyk4.Kinect(resolution=720, wide_fov=True, binned=True)

    cv2.namedWindow('Color', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Color', mouse_event)

    while True:
        if kin.getFrames(getColor=True, getDepth=True, getIR=False):
            # get buffer data from Kinect
            color_data = kin.getColorData()
            depth_data = kin.getDepthData(align=False)

            # convert buffers to numpy arrays
            depth_image = np.array(depth_data, copy = True)
            color_image = np.array(color_data, copy = True)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)

            # Apply colormap on depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                            depth_image, alpha=0.03), cv2.COLORMAP_JET)

            ## Here is where the magic happens ...
            # Project color image points to depth image
            # color_coords is a list of value pairs [[xc1,yc1], [xc2,yc2], ...]
            # depth_coords have the same format as color_coords
            # depth_coords return -1 if there is no depth information
            # at the desire location, 
            # e.g. if at [x2, y2] the depth is 0 depth_coords = [[xd1, yd1], [-1, -1]]
            # we select -1 because there can not be negative pixel coordinates
            depth_coords = kin.map_coords_color_2d_to_depth_2d(color_coords)

            # Backproject color image points to 3D depth camera space
            # points_3d is a list of triplets of X, Y, Z points in
            # the depth camera reference or color camera reference
            # if color_coords is [[xc1,yc1], [xc2,yc2], ...]
            # points_3d is [[X1, Y1, Z1], [X2, Y2, Z2], ...]
            # points_3d contains [0, 0, 0] if for desired color coordinates
            # there is not depth available. 
            # E.g. if at [x2, y2] the depth is 0, points_3d = [[X1, Y1, Z1], [0, 0, 0]]
            # we select 0 because depth = 0 means no depth data.
            points_3d = kin.map_coords_color_2d_to_3D(color_coords,
                                                      depth_reference=False)

            ## Visualization
            # Draw color image points
            for p in color_coords:                                          
                cv2.circle(color_image,(p[0], p[1]), 8, (0,0,255), -1)

            # Draw and print depth points
            print("Depths: ", end="")
            for p in depth_coords:
                cv2.circle(depth_colormap,(p[0], p[1]), 5, (0,0,255), -1)
                print("[%2d] " % depth_image[p[1], p[0]], end="")
            
            cv2.imshow('Depth', depth_colormap)
            cv2.imshow('Color', color_image)

        k = cv2.waitKey(33) & 0xFF
        if k == ord('c'):
            color_coords = []
        if k ==27:
            break

    kin.close()  # close Kinect
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()