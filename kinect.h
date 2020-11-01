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

#ifdef BODY
#include <k4abt.h>
#endif

/**
 * Implementation of K4a wrapper for python bindings
 * 
 */
class Kinect {
private:
    int res;
    // Kinect device
    k4a_device_t m_device = NULL;
    std::string serial_number;

    // Kinect configuration
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    // Maximum timeout
    const int32_t TIMEOUT_IN_MS = 1000;

    // capture device
    k4a_capture_t m_capture = NULL;

    // color, depth, IR, and body index images
    k4a_image_t m_image_c = nullptr;
    k4a_image_t m_image_d = nullptr;
    k4a_image_t m_image_ir = nullptr;
    k4a_image_t m_body_index = nullptr;

    // Sensor data
    Imu_sample m_imu_data;
    bool m_imu_sensors_available;

    // Body tracking
    #ifdef BODY
    k4abt_tracker_t m_tracker = NULL;
    k4abt_frame_t m_body_frame = NULL;
    bool m_body_tracking_available;
    uint32_t m_num_bodies;
    py::list m_bodies;
    #endif

    // calibration and transformation object
    k4a_calibration_t m_calibration;
    Calibration m_depth_calib;
    Calibration m_color_calib;
    k4a_transformation_t m_transformation = NULL;

    int initialize(uint8_t deviceIndex, int resolution, bool wfov,
                   bool binned, uint8_t framerate, bool sensor_color,
                   bool sensor_depth, bool sensor_ir, bool imu_sensors,
                   bool body_tracking, bool body_index);
    bool align_depth_to_color(int width, int height,
                        k4a_image_t &transformed_depth_image);
    bool align_color_to_depth(k4a_image_t &transformed_color_image);
    void update_calibration(Calibration&, bool);
    bool depth_image_to_point_cloud(int width, int height, k4a_image_t &xyz_image);
    
    #ifdef BODY
    py::dict get_body_data(k4abt_body_t body);
    void change_body_index_to_body_id(uint8_t* image_data, int width, int height);
    #endif

public:
    Kinect(uint8_t deviceIndex = 0, int resolution = 1080, bool wfov = false,
           bool binned = true, uint8_t framerate = 30, bool sensor_color = true,
           bool sensor_depth = true, bool sensor_ir = true,
           bool imu_sensors = false, bool body_tracking = false,
           bool body_index = false);
    ~Kinect();

    void close();
    const int get_frames(bool get_color = true, bool get_depth = true, bool get_ir = true, 
                        bool get_sensors = false, bool get_body = false,
                        bool get_body_index = false);
    Imu_sample get_sensor_data();
    ColorData get_color_data();
    DepthData get_depth_data(bool align=false);
    DepthData get_ir_data();
    BufferPointCloud get_pointcloud();
    BufferColor get_pointcloud_color();
    void save_pointcloud(const char *file_name);
    Calibration get_depth_calibration();
    Calibration get_color_calibration();
    std::string get_serial_number();
    void set_exposure(int);
    const int get_exposure();
    void set_gain(int);
    std::vector<std::vector<int> > map_coords_color_to_depth(std::vector<std::vector<int> > &color_coords);
    std::vector<std::vector<int> > map_coords_color_to_3D(std::vector<std::vector<int> > &color_coords, bool depth_reference);
    std::vector<std::vector<int> > map_coords_depth_to_color(std::vector<std::vector<int> > &depth_coords);
    std::vector<std::vector<int> > map_coords_depth_to_3D(std::vector<std::vector<int> > &depth_coords);

    // Body tracking functions
    #ifdef BODY
    int get_num_bodies();
    py::list get_bodies();
    BodyIndexData get_body_index_map(bool returnId=false);
    #endif
};

#endif
