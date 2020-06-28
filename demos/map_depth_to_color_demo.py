"""
    Mapping from depth to color demo

    How to use:
    - Clic on the depth image to show the corresponding point on color image
"""
import numpy as np 
import cv2
import pyk4

depth_points = []

def mouse_event(event, x, y, flags, param):
    global depth_points
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_points.append((x, y))

def main():
    global depth_points
    # Create Kinect object and initialize
    kin = pyk4.Kinect(resolution=1080, wfov=True, binned=True)
    
    depth_window_name = 'Click on the Depth image'
    color_window_name = 'Mapped coordinates in Color'
    cv2.namedWindow(color_window_name, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(depth_window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(depth_window_name, mouse_event)

    while True:
        if kin.getFrames(getColor=True, getDepth=True, getIR=False):
            color_data = kin.getColorData()
            depth_data = kin.getDepthData(align=False)

            depth_image = np.array(depth_data, copy = True)
            color_image = np.array(color_data, copy = True)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)

            # Project depth image points to color image points
            # depth_points is a list of value pairs [[xd1,yd1], [xd2,yd2], ...]
            # color_points have the same format as depth_points
            # map_coords_depth_to_color return [-1, -1] if there is no depth information
            # at the desire location, or the transformation failed.
            color_points = kin.map_coords_depth_to_color(depth_points)

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

        k = cv2.waitKey(1) & 0xFF
        if k == ord('c'):
            depth_points = []
        if k ==27:
            break

    kin.close()  # close Kinect
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()