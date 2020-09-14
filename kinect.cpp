/**
 * @file kinect.cpp
 *
 * @brief Implementation of kinect K4 wrapper for python bindings
 *
 * @author Juan Terven
 * Contact: juan@aifi.io
 *
 */

#include <string>
#include <memory>
#include <iostream>
#include <math.h>
#include "kinect.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

Kinect::Kinect(uint8_t deviceIndex, int resolution, bool wfov, bool binned, uint8_t framerate,
    bool sensorColor, bool sensorDepth, bool sensorIR, bool imuSensors) {
    initialize(deviceIndex, resolution, wfov, binned, framerate, sensorColor, sensorDepth, sensorIR, imuSensors);
}

Kinect::~Kinect()
{
    if (m_device != NULL) {
        k4a_device_close(m_device);
        m_device = NULL;
    }
    if (m_image_c != NULL) {
        k4a_image_release(m_image_c);
        m_image_c = NULL;
    }
    if (m_image_d != NULL) {
        k4a_image_release(m_image_d);
        m_image_d = NULL;
    }
    if (m_capture != NULL) {
        k4a_capture_release(m_capture);
        m_capture = NULL;
    }
}

/**
*    Available resolutions are:
*    720: 1280 x 720 @ 25 FPS, Aligned 22 FPS
*    1080: 1920 x 1080 @ 24 FPS, Aligned 18 FPS
*    1440: 2560 x 1440 @ 22 FPS, Aligned 14 FPS
*    1536: 2048 x 1536 @ 23 FPS, Aligned 16 FPS
*    2160: 3840 x 2160 @ 20 FPS, Aligned 10 FPS
*    3072: 4096 x 3072 @ 12 FPS, Aligned 7 FPS
*/
int Kinect::initialize(uint8_t deviceIndex, int resolution, bool wideFOV, bool binned, uint8_t framerate,
    bool sensorColor, bool sensorDepth, bool sensorIR, bool imuSensors) 
{
    // Values initialization
    // Color resolution
    this->res = resolution;
    // General configuration of the device
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // Sensors sync
    m_config.synchronized_images_only = false;
    // Color OFF
    m_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    // depth mode
    m_config.depth_mode = K4A_DEPTH_MODE_OFF;


    // Devices available
    uint32_t m_device_count = k4a_device_get_installed_count();
    
    if (m_device_count == 0) 
    {
        printf("No K4A m_devices found\n");
        return 1;
    }

    // Open Kinect device
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &m_device)) {
        printf("Failed to open m_device\n");
        if (m_device != NULL) {
            k4a_device_close(m_device);
            m_device = NULL;
            return 1;
        }
    }

    // Get serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(m_device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char *serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(m_device,serial,&serial_size);
    // printf("Opened device SN: %s\n",serial);
    this->serial_number = serial;
    free(serial);



    // Turn image sync if RGB or IR is present
    if (m_config.color_resolution != K4A_COLOR_RESOLUTION_OFF && m_config.depth_mode != K4A_DEPTH_MODE_OFF )
        m_config.synchronized_images_only = true;

    // Color Configuration
    if (sensorColor) 
    {
        // Consider K4A_IMAGE_FORMAT_COLOR_MJPG. It is less CPU expensive
        m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        // m_config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        
        switch(resolution) {
            case 720:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
                break;
            case 1080:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
                break;
            case 1440:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
                break;
            case 2160:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
                break;
            case 1536:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
                break;
            case 3072:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
                break;
            default:
                m_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
                break;
        }
        
    }


    // Configure Depth sensor
    if (!sensorIR && !sensorDepth) 
    {
        m_config.depth_mode = K4A_DEPTH_MODE_OFF;
    }
    else if (sensorIR && !sensorDepth)
    {
        m_config.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    }
    else if ((sensorIR && sensorDepth) || (!sensorIR && sensorDepth))
    {
        if (wideFOV)
        {
            if (binned)
            {
                m_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
            } else {
                m_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
            }
        } else {
            if (binned)
            {
                m_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
            } else {
                m_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
            }
        }
    }


    // Final FPS configuration
    // FPS
    m_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    // For resolution 4096x3072 only 0,5,15 FPS
    int maxFPS = 30;
    if (m_config.color_resolution==K4A_COLOR_RESOLUTION_3072P)
        //m_config.camera_fps = framerate<=5?K4A_FRAMES_PER_SECOND_5:K4A_FRAMES_PER_SECOND_15;
        maxFPS = 15;

    if (m_config.depth_mode==K4A_DEPTH_MODE_WFOV_UNBINNED)
        maxFPS = 15;

    if (framerate>maxFPS)
    {
        if (maxFPS==15)
            m_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    } else {
        if (framerate<=5)
            m_config.camera_fps = K4A_FRAMES_PER_SECOND_5;
        // else if (framerate<=10)
        //     m_config.camera_fps = K4A_FRAMES_PER_SECOND_10;
        else if (framerate<=15)
            m_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    }

    // Get calibration
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(m_device, m_config.depth_mode, m_config.color_resolution, &m_calibration)) {
        printf("Failed to get calibration\n");
        if (m_device) {
            k4a_device_close(m_device);
            m_device = NULL;
            return 1;
        }
    }
    updateCalibration(m_depth_calib, true);
    updateCalibration(m_color_calib, false);

    // get transformation to map from depth to color
    m_transformation = k4a_transformation_create(&m_calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &m_config)) {
        printf("Failed to start m_device\n");
        if (m_device) {
            k4a_device_close(m_device);
            m_device = NULL;
            return 1;
        }
    }
    else
        printf("Kinect started successfully!!\n");

    if (K4A_RESULT_SUCCEEDED != k4a_device_set_color_control(m_device,
                                    K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
                                    K4A_COLOR_CONTROL_MODE_MANUAL, 2)) {
            printf("Failed to change power frequency\n");
    }

    m_imu_sensors_available = false;
    if(imuSensors) {
        if(k4a_device_start_imu(m_device) == K4A_RESULT_SUCCEEDED) {
            printf("IMU sensors started succesfully.");
            m_imu_sensors_available = true;
        }
        else {
            printf("IMU SENSORES FAILED INITIALIZATION");
            m_imu_sensors_available = false;
        }
    }
    
    return 0;
} // initialize


void Kinect::updateCalibration(Calibration &calib_struct, bool depth) {
    k4a_calibration_camera_t  calib;

    if(depth)
        calib = m_calibration.depth_camera_calibration;
    else
        calib = m_calibration.color_camera_calibration;

    calib_struct.width = calib.resolution_width;
    calib_struct.height = calib.resolution_height;
    calib_struct.cx = calib.intrinsics.parameters.param.cx;
    calib_struct.cy = calib.intrinsics.parameters.param.cy;
    calib_struct.fx = calib.intrinsics.parameters.param.fx;
    calib_struct.fy = calib.intrinsics.parameters.param.fy;
    calib_struct.k1 = calib.intrinsics.parameters.param.k1;
    calib_struct.k2 = calib.intrinsics.parameters.param.k2;
    calib_struct.k3 = calib.intrinsics.parameters.param.k3;
    calib_struct.k4 = calib.intrinsics.parameters.param.k4;
    calib_struct.k5 = calib.intrinsics.parameters.param.k5;
    calib_struct.k6 = calib.intrinsics.parameters.param.k6;
    calib_struct.p1 = calib.intrinsics.parameters.param.p1;
    calib_struct.p2 = calib.intrinsics.parameters.param.p2;
    calib_struct.codx = calib.intrinsics.parameters.param.codx;
    calib_struct.cody = calib.intrinsics.parameters.param.cody;
    for(int i=0; i<9; i++)
        calib_struct.rotation[i] = calib.extrinsics.rotation[i];
    for(int i=0; i<3; i++)        
        calib_struct.translation[i] = calib.extrinsics.translation[i];
    
}

Calibration Kinect::getDepthCalibration() {
    return m_depth_calib;
}

Calibration Kinect::getColorCalibration() {
    return m_color_calib;
}


const int Kinect::getFrames(bool getColor, bool getDepth, bool getIR, bool getSensors) {
    bool goodColor = true, goodDepth = true, goodIR = true;
    if (this->res==0)
        getColor = false;

    // Release images before next acquisition
    if (m_capture) {
        k4a_capture_release(m_capture);
        m_capture = NULL;
    }
    if (m_image_c) {
        k4a_image_release(m_image_c);
        m_image_c = NULL;
    }
    if (m_image_d) {
        k4a_image_release(m_image_d);
        m_image_d = NULL;
    }
    if (m_image_ir) {
        k4a_image_release(m_image_ir);
        m_image_ir = NULL;
    }

    // Get a m_capture
    switch (k4a_device_get_capture(m_device, &m_capture, TIMEOUT_IN_MS)) {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a m_capture\n");
            return 0;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a m_capture\n");
            printf("Restarting streaming ...");
            k4a_device_stop_cameras	(m_device);	
            if(K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &m_config)) {
                printf("Failed to restart streaming\n");
                k4a_device_stop_cameras	(m_device);	
                return 0;
            }
    }

    // Get color image
    if(getColor) {
        m_image_c = k4a_capture_get_color_image(m_capture);
        if (m_image_c == NULL) {
            goodColor = false;
            printf("Could not read color image\n");
        }
    }
    // Get depth16 image
    if(getDepth) {
        m_image_d = k4a_capture_get_depth_image(m_capture);
        if (m_image_d == NULL) {
            goodDepth = false;
            printf("Could not read depth image\n");
        }
    }
    // Get IR image
    if(getIR) {
        m_image_ir = k4a_capture_get_ir_image(m_capture);
        if (m_image_ir == NULL) {
            goodIR = false;
            printf("Could not read IR image");
        }
    }

    if(getSensors && m_imu_sensors_available) {
        k4a_imu_sample_t imu_sample;

        // Capture a imu sample
        k4a_wait_result_t imu_status;
        imu_status = k4a_device_get_imu_sample(m_device, &imu_sample, TIMEOUT_IN_MS);
        switch (imu_status)
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a imu sample\n");
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a imu sample\n");
            break;
        }

        // Access the accelerometer readings
        if (imu_status == K4A_WAIT_RESULT_SUCCEEDED)
        {
            m_imu_data.temperature = imu_sample.temperature;
            m_imu_data.acc_x = imu_sample.acc_sample.xyz.x;
            m_imu_data.acc_y = imu_sample.acc_sample.xyz.y;
            m_imu_data.acc_z = imu_sample.acc_sample.xyz.z;
            m_imu_data.acc_timestamp_usec = imu_sample.acc_timestamp_usec;
            m_imu_data.gyro_x = imu_sample.gyro_sample.xyz.x;
            m_imu_data.gyro_y = imu_sample.gyro_sample.xyz.y;
            m_imu_data.gyro_z = imu_sample.gyro_sample.xyz.z;
            m_imu_data.gyro_timestamp_usec = imu_sample.gyro_timestamp_usec;
        }
    }

    if(goodColor && goodDepth && goodIR)
        return 1;
    else
        return 0;
} // getFrames

Imu_sample Kinect::getSensorData() {
    return m_imu_data;
}

ColorData Kinect::getColorData() {
    ColorData colorData;

    if(m_image_c) {
        int w = k4a_image_get_width_pixels(m_image_c);
        int h = k4a_image_get_height_pixels(m_image_c);
        int stride = k4a_image_get_stride_bytes(m_image_c);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_image_c);
        auto sz = k4a_image_get_size(m_image_c);
        void* data = malloc(sz);
        memcpy(data, dataBuffer, sz);
        BufferColor m((uint8_t *)data, h, w, stride);
        
        colorData.buffer = m;
        colorData.timestamp_nsec = k4a_image_get_system_timestamp_nsec(m_image_c);
        return colorData;
    }
    else {
        BufferColor m(NULL, 0, 0, 0);
        colorData.buffer = m;
        colorData.timestamp_nsec = 0;
        return colorData;
    }
}

DepthData Kinect::getDepthData(bool align) {
    DepthData depthData;

    if(m_image_d) {
        // align images
        if(align) {
            k4a_image_t image_dc;
            if(align_depth_to_color(k4a_image_get_width_pixels(m_image_c),
                k4a_image_get_height_pixels(m_image_c), image_dc)) {
                k4a_image_release(m_image_d);
                m_image_d = NULL;
                m_image_d = image_dc;
            }
            else 
                printf("Failed to align\n");
        }

        int w = k4a_image_get_width_pixels(m_image_d);
        int h = k4a_image_get_height_pixels(m_image_d);
        int stride = k4a_image_get_stride_bytes(m_image_d);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_image_d);
        auto sz = k4a_image_get_size(m_image_d);
        void* data = malloc(sz);
        memcpy(data, dataBuffer, sz);
        BufferDepth m((uint16_t *)data, h, w, stride);

        depthData.buffer = m;
        depthData.timestamp_nsec = k4a_image_get_system_timestamp_nsec(m_image_d);
        return depthData;
    }
    else {
        BufferDepth m(NULL, 0, 0, 0);
        depthData.buffer = m;
        depthData.timestamp_nsec = 0;
        return depthData;
    }
}

DepthData Kinect::getIRData() {
    DepthData irData;

    if(m_image_ir) {
        int w = k4a_image_get_width_pixels(m_image_ir);
        int h = k4a_image_get_height_pixels(m_image_ir);
        int stride = k4a_image_get_stride_bytes(m_image_ir);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_image_ir);
        auto sz = k4a_image_get_size(m_image_ir);
        void* data = malloc(sz);
        memcpy(data, dataBuffer, sz);
        BufferDepth m((uint16_t *)data, h, w, stride);
        
        irData.buffer = m;
        irData.timestamp_nsec = k4a_image_get_system_timestamp_nsec(m_image_ir);
        return irData;
    }
    else {
        BufferDepth m(NULL, 0, 0, 0);
        irData.buffer = m;
        irData.timestamp_nsec = 0;
        return irData;
    }
}

bool Kinect::align_depth_to_color(int width, int height, k4a_image_t &transformed_depth_image){
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                width, height, width * (int)sizeof(uint16_t),
                                                &transformed_depth_image)) {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(m_transformation,
                                                                            m_image_d,
                                                                            transformed_depth_image)) {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    return true;
}


bool Kinect::align_color_to_depth(k4a_image_t &transformed_color_image){
    int depth_image_width_pixels = k4a_image_get_width_pixels(m_image_d);
    int depth_image_height_pixels = k4a_image_get_height_pixels(m_image_d);
    
    transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(m_transformation,
                                                                              m_image_d,
                                                                              m_image_c,
                                                                              transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    return true;
}

std::string Kinect::getSerialNumber()
{
    return this->serial_number;
}

void Kinect::close(){
    if (m_device) {
        k4a_device_close(m_device);
        m_device = NULL;
    }
}

void Kinect::setExposure(int exposure) {
    if (m_device) {
        if (K4A_RESULT_SUCCEEDED != k4a_device_set_color_control(m_device,
                                    K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
                                    K4A_COLOR_CONTROL_MODE_MANUAL,
                                    (int32_t)exposure)) {
            printf("Failed to change exposure time\n");
        }
    }
    else {
        printf("No valid Kinect device. Failed to change exposure time\n");
    }
}

const int Kinect::getExposure() {
    int exposure = 0;
    if(m_image_c) {
        exposure = k4a_image_get_exposure_usec(m_image_c);
    }
    else
        printf("Could not get exposure value. Image not valid.");

    return exposure;
}

void Kinect::setGain(int gain) {
    if (m_device) {
        if (K4A_RESULT_SUCCEEDED != k4a_device_set_color_control(m_device,
                                    K4A_COLOR_CONTROL_GAIN,
                                    K4A_COLOR_CONTROL_MODE_MANUAL,
                                    gain)) {
            printf("Failed to change gain\n");
        }
    }
    else {
        printf("No valid Kinect device. Failed to change gain\n");
    }
}


std::vector<std::vector<int> > Kinect::map_coords_color_to_depth(std::vector<std::vector<int> > &color_coords) {
    k4a_float2_t init_coords, depth_coords;
    std::vector<std::vector<int> > final_coords;
    int val;
    k4a_result_t res;

    for(unsigned int i=0; i < color_coords.size(); i++) {
        std::vector<int> d_coords;
        init_coords.xy.x = color_coords[i][0];
        init_coords.xy.y = color_coords[i][1];
        res = k4a_calibration_color_2d_to_depth_2d(&m_calibration, &init_coords, m_image_d, &depth_coords, &val);
        if(res == K4A_RESULT_SUCCEEDED) {
            d_coords.push_back(depth_coords.xy.x);
            d_coords.push_back(depth_coords.xy.y);
        }
        else {
            d_coords.push_back(-1);
            d_coords.push_back(-1);
        }
        final_coords.push_back(d_coords);
    }
    return final_coords;
}

std::vector<std::vector<int> > Kinect::map_coords_color_to_3D(
                            std::vector<std::vector<int> > &color_coords,
                            bool depth_reference) {
    std::vector<std::vector<int> > depth_coords;
    std::vector<std::vector<int> > final_coords;

    k4a_calibration_type_t reference = K4A_CALIBRATION_TYPE_DEPTH;
    if (!depth_reference)
        reference = K4A_CALIBRATION_TYPE_COLOR;


    // get the depth data
    int h = k4a_image_get_height_pixels(m_image_d);
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(m_image_d);

    // from color to depth
    depth_coords = this->map_coords_color_to_depth(color_coords);

    // from depth to 3D
    for(unsigned int i=0; i < depth_coords.size(); i++) {
        std::vector<int> coords3d_vect;
        k4a_float2_t coordsdepth;
        k4a_float3_t coords3d;
        int valid;
        if(depth_coords[i][0] != -1 && depth_coords[i][1] != -1) {
            coordsdepth.xy.x = depth_coords[i][0];
            coordsdepth.xy.y = depth_coords[i][1];
            // get depth value
            float depth = (float)depth_data[h*depth_coords[i][1] + depth_coords[i][0]];
            k4a_calibration_2d_to_3d(&m_calibration, &coordsdepth, depth, 
                                     K4A_CALIBRATION_TYPE_DEPTH, 
                                     reference,
                                     &coords3d, &valid);
            if(valid == 1) {
                coords3d_vect.push_back(coords3d.xyz.x);
                coords3d_vect.push_back(coords3d.xyz.y);
                coords3d_vect.push_back(coords3d.xyz.z);
            }
            else {
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
            }
        }
        else {
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
        }
        final_coords.push_back(coords3d_vect);
    }

    return final_coords;
}

std::vector<std::vector<int> > Kinect::map_coords_depth_to_color(std::vector<std::vector<int> > &depth_coords) {
    k4a_float2_t init_coords, color_coords;
    std::vector<std::vector<int> > final_coords;
    int val;
    k4a_result_t res;

    int h = k4a_image_get_height_pixels(m_image_d);
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(m_image_d);

    for(unsigned int i=0; i < depth_coords.size(); i++) {
        std::vector<int> c_coords;
        init_coords.xy.x = depth_coords[i][0];
        init_coords.xy.y = depth_coords[i][1];
        float depth = (float)depth_data[h*depth_coords[i][1] + depth_coords[i][0]];
        res = k4a_calibration_2d_to_2d(&m_calibration, &init_coords, depth,
                                       K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR,
                                       &color_coords, &val);
        if(res == K4A_RESULT_SUCCEEDED) {
            c_coords.push_back(color_coords.xy.x);
            c_coords.push_back(color_coords.xy.y);
        }
        else {
            c_coords.push_back(-1);
            c_coords.push_back(-1);
        }
        final_coords.push_back(c_coords);
    }
    return final_coords;
}

std::vector<std::vector<int> > Kinect::map_coords_depth_to_3D(
                            std::vector<std::vector<int> > &depth_coords) {
    std::vector<std::vector<int> > final_coords;

    // get the depth data
    int h = k4a_image_get_height_pixels(m_image_d);
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(m_image_d);

    // from depth to 3D
    for(unsigned int i=0; i < depth_coords.size(); i++) {
        std::vector<int> coords3d_vect;
        k4a_float2_t coordsdepth;
        k4a_float3_t coords3d;
        int valid;
        if(depth_coords[i][0] != -1 && depth_coords[i][1] != -1) {
            coordsdepth.xy.x = depth_coords[i][0];
            coordsdepth.xy.y = depth_coords[i][1];
            // get depth value
            float depth = (float)depth_data[h*depth_coords[i][1] + depth_coords[i][0]];
            k4a_calibration_2d_to_3d(&m_calibration, &coordsdepth, depth, 
                                     K4A_CALIBRATION_TYPE_DEPTH, 
                                     K4A_CALIBRATION_TYPE_DEPTH,
                                     &coords3d, &valid);
            if(valid == 1) {
                coords3d_vect.push_back(coords3d.xyz.x);
                coords3d_vect.push_back(coords3d.xyz.y);
                coords3d_vect.push_back(coords3d.xyz.z);
            }
            else {
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
            }
        }
        else {
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
                coords3d_vect.push_back(0);
        }
        final_coords.push_back(coords3d_vect);
    }

    return final_coords;
}

/** Transforms the depth image into 3 planar images representing X, Y and Z-coordinates of corresponding 3d points.
* Throws error on failure.
*
* \sa k4a_transformation_depth_image_to_point_cloud
*/
bool Kinect::depth_image_to_point_cloud(int width, int height, k4a_image_t &xyz_image) {
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 width, height,
                                                 width * 3 * (int)sizeof(int16_t),
                                                 &xyz_image)) {
        printf("Failed to create transformed xyz image\n");
        return false;
    }

    k4a_result_t result =
        k4a_transformation_depth_image_to_point_cloud(m_transformation,
                                                      m_image_d,
                                                      K4A_CALIBRATION_TYPE_DEPTH,
                                                      xyz_image);

    if (K4A_RESULT_SUCCEEDED != result) {
        printf("Failed to transform depth image to point cloud!");
        return false;
    }
    return true;
}

BufferPointCloud Kinect::getPointCloud() {
    if(m_image_d) {
        k4a_image_t image_xyz = NULL;

        // Get the point cloud
        if(depth_image_to_point_cloud(k4a_image_get_width_pixels(m_image_d),
            k4a_image_get_height_pixels(m_image_d), image_xyz)) {

            int w = k4a_image_get_width_pixels(image_xyz);
            int h = k4a_image_get_height_pixels(image_xyz);
            int stride = k4a_image_get_stride_bytes(image_xyz);
            int16_t *dataBuffer = (int16_t *)(void *)k4a_image_get_buffer(image_xyz);

            //uint8_t* dataBuffer = k4a_image_get_buffer(image_xyz);
            auto sz = k4a_image_get_size(image_xyz);
            void* data = malloc(sz);
            memcpy(data, dataBuffer, sz);
            BufferPointCloud m((int16_t *)data, h, w, stride);

            return m;
        }
        else {
            printf("Failed to create PointCloud\n");
            BufferPointCloud m(NULL, 0, 0, 0);
            return m;
        }
    }
    else {
        BufferPointCloud m(NULL, 0, 0, 0);
        return m;
    }
}

BufferColor Kinect::getPointCloudColor() {
    if(m_image_d) {
        k4a_image_t transformed_color_image = NULL;

        // Get the color image aligned with depth coordinates
        // in transformed_color_image
        if(align_color_to_depth(transformed_color_image)) {

            int w = k4a_image_get_width_pixels(transformed_color_image);
            int h = k4a_image_get_height_pixels(transformed_color_image);
            int stride = k4a_image_get_stride_bytes(transformed_color_image);
            uint8_t* dataBuffer = k4a_image_get_buffer(transformed_color_image);
            auto sz = k4a_image_get_size(transformed_color_image);
            void* data = malloc(sz);
            memcpy(data, dataBuffer, sz);
            BufferColor m((uint8_t *)data, h, w, stride);
            k4a_image_release(transformed_color_image);
            return m;
        }
        else {
            BufferColor m(NULL, 0, 0, 0);
            return m;
        }
    }
    else {
        BufferColor m(NULL, 0, 0, 0);
        return m;
    }
}

void Kinect::savePointCloud(const char *file_name) {
    if(m_image_d) {
        k4a_image_t image_xyz = NULL;
        k4a_image_t transformed_color_image = NULL;

        // Get the color image aligned with depth coordinates
        // in transformed_color_image
        if(align_color_to_depth(transformed_color_image)) {

            // Get the point cloud
            if(depth_image_to_point_cloud(k4a_image_get_width_pixels(m_image_d),
                k4a_image_get_height_pixels(m_image_d), image_xyz)) {

                write_point_cloud(image_xyz,
                                  transformed_color_image,
                                  file_name);
            }
            else {
                printf("Failed to create PointCloud\n");
            }
        }
        else {
            printf("Failed to get the color image aligned with depth for PointCloud\n");
        }
    }
    else {
        printf("No depth image available for generating Pointcloud\n");
    }
}



