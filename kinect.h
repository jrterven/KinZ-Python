/**
 * @file kinect.h
 *
 * @brief Class of kinect K4 funcionality
 *
 * @author Juan Terven
 * Contact: juan@aifi.io
 *
 */

#ifndef KINECT_H
#define KINECT_H

#include <string>
#include <k4a/k4a.h>
#include "utils.h"
#include "calibration.h"

/**
 * Implementation of K4a wrapper for python bindings
 * 
 */
class Kinect {
private:
    int res;
    // Kinect device
    k4a_device_t m_device = NULL;

    // Kinect configuration
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    // Maximum timeout
    const int32_t TIMEOUT_IN_MS = 1000;

    // capture device
    k4a_capture_t m_capture = NULL;

    // color, depth, and IR images
    k4a_image_t m_image_c = nullptr;
    k4a_image_t m_image_d = nullptr;
    k4a_image_t m_image_ir = nullptr;

    // Sensor data
    Imu_sample m_imu_data;
    bool m_imu_sensors_available;

    // calibration and transformation object
    k4a_calibration_t m_calibration;
    Calibration m_depth_calib;
    Calibration m_color_calib;
    k4a_transformation_t m_transformation = NULL;

    int initialize(uint8_t deviceIndex, int resolution, bool wideFOV, bool binned, 
        uint8_t framerate, bool sensorColor, bool sensorDepth, bool sensorIR, bool imuSensors);
    bool align_depth_to_color(int width, int height,
                        k4a_image_t &transformed_depth_image);
    bool align_color_to_depth(k4a_image_t &transformed_color_image);
    void updateCalibration(Calibration&, bool);
    bool depth_image_to_point_cloud(int width, int height, k4a_image_t &xyz_image);
    std::string serial_number;

public:
    Kinect(uint8_t deviceIndex = 0, int resolution = 1080, bool wfov = false,
           bool binned = true, uint8_t framerate = 30, bool sensorColor = true,
           bool sensorDepth = true, bool sensorIR = true, bool imuSensors = false);
    ~Kinect();

    void close();
    const int getFrames(bool getColor = true, bool getDepth = true, bool getIR = true, bool getSensors = false);
    Imu_sample getSensorData();
    ColorData getColorData();
    DepthData getDepthData(bool align=false);
    DepthData getIRData();
    BufferPointCloud getPointCloud();
    BufferColor getPointCloudColor();
    void savePointCloud(const char *file_name);
    Calibration getDepthCalibration();
    Calibration getColorCalibration();
    std::string getSerialNumber();
    void setExposure(int);
    const int getExposure();
    void setGain(int);
    std::vector<std::vector<int> > map_coords_color_to_depth(std::vector<std::vector<int> > &color_coords);
    std::vector<std::vector<int> > map_coords_color_to_3D(std::vector<std::vector<int> > &color_coords, bool depth_reference);
    std::vector<std::vector<int> > map_coords_depth_to_color(std::vector<std::vector<int> > &depth_coords);
    std::vector<std::vector<int> > map_coords_depth_to_3D(std::vector<std::vector<int> > &depth_coords);
};

#endif
