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
    
    # get calibration objects
    depth_calib = kin.getDepthCalibration()
    color_calib = kin.getColorCalibration()

    # extract calibration parameters
    depth_size = depth_calib.getSize()
    depth_K = depth_calib.getIntrinsicsMatrix(extended=False)
    depth_dist = depth_calib.getDistortionParams()
    depth_pose = depth_calib.getCameraPose()
    color_K = color_calib.getIntrinsicsMatrix(extended=True)
    color_dist = color_calib.getDistortionParams()
    color_pose = color_calib.getCameraPose()

    cv2.namedWindow('Color', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Depth', mouse_event)

    while True:
        if kin.getFrames(getColor=True, getDepth=True, getIR=False):
            color_data = kin.getColorData()
            depth_data = kin.getDepthData(align=False)

            depth_image = np.array(depth_data, copy = True)
            color_image = np.array(color_data, copy = True)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)

            # Project X, Y, Z computed with depth to color image
            color_points = project_xyZ_to_camera(depth_points, depth_image,
                                          depth_K, depth_dist, color_K, color_pose)
            for p in color_points:                                          
                cv2.circle(color_image,(p[0], p[1]), 8, (0,0,255), -1)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            for p in depth_points:
                cv2.circle(depth_colormap,(p[0], p[1]), 5, (0,0,255), -1)

            # Resize images
            color_small = cv2.resize(color_image, None, fx=0.5, fy=0.5)

            cv2.imshow('Depth', depth_colormap)
            cv2.imshow('Color', color_small)

        k = cv2.waitKey(33) & 0xFF
        if k == ord('c'):
            depth_points = []
        if k ==27:
            break

    kin.close()  # close Kinect
    cv2.destroyAllWindows()

def project_xyZ_to_camera(points, depth, d_k, d_dist, c_k, c2d_T):
    ''' Project a point from depth to color camera
    :param points: list of (x, y) pairs
    :param depth: depth image 
    :param d_k: 3x3 depth camera intrinsic matrix
    :param d_dist: 8x1 distortion vector (k1, k2, p1, p2, k3, k4, k5, k6)
    :param c_k: 4x4 color camera intrinsic matrix
    :param c2d_T: 4x4 color to depth pose matrix
    :return color_points: list of (x, y) coordinates in color image
    '''
    color_points = []

    if len(points) == 0:
        return color_points

    # Extract intrisic parameters
    fx = d_k[0, 0]
    fy = d_k[1, 1]
    cx = d_k[0, 2]
    cy = d_k[1, 2]

    # undistort depth coordinates (results are normalized by fx,fy)
    pts = np.array(points)
    undist_points = cv2.undistortPoints(pts.reshape(-1,1,2).astype(np.float32), d_k, d_dist).squeeze(axis=1)

    # map each depth point to color camera
    for idx in range(undist_points.shape[0]):
        Z = np.array(depth[points[idx][1], points[idx][0]]) # get depth
        x = undist_points[idx][0] * fx + cx # denormalize
        y = undist_points[idx][1] * fy + cy

        # get the 3D points
        X = (Z * (x - cx)) / fx
        Y = (Z * (y - cy)) / fy
        XYZ = np.array([X, Y, Z, 1], dtype=np.float32).reshape(4, 1)
    
        # Project X, Y, Z in color camera
        xyh = c_k @ c2d_T @ XYZ

        # dehomogenization
        xc = xyh[0]/xyh[2]
        yc = xyh[1]/xyh[2]
        color_points.append((xc, yc))

    return color_points

if __name__ == '__main__':
    main()