#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "kinect.h"

namespace py = pybind11;

PYBIND11_MODULE(kinz, m) {
    
    py::class_<BufferDepth>(m, "BufferDepth", py::buffer_protocol())
   .def_buffer([](BufferDepth &m) -> py::buffer_info {
        return py::buffer_info(
            m.data(),                              
            sizeof(uint16_t),                         
            py::format_descriptor<uint16_t>::format(),
            2,                                     
            { m.rows(), m.cols()},                
            { m.stride(), sizeof(uint16_t) }
        );
    });

    py::class_<BufferColor>(m, "BufferColor", py::buffer_protocol())
   .def_buffer([](BufferColor &m) -> py::buffer_info {
        return py::buffer_info(
            m.data(),                              
            sizeof(uint8_t),                         
            py::format_descriptor<uint8_t>::format(),
            3,                                     
            { m.rows(), m.cols(), size_t(4)},                
            { sizeof(unsigned char) * 4 * m.cols(),
              sizeof(unsigned char) * 4,
              sizeof(unsigned char) }
        );
    });

    py::class_<BufferBodyIndex>(m, "BufferBodyIndex", py::buffer_protocol())
   .def_buffer([](BufferBodyIndex &m) -> py::buffer_info {
        return py::buffer_info(
            m.data(),                              
            sizeof(uint8_t),                         
            py::format_descriptor<uint8_t>::format(),
            2,                                     
            { m.rows(), m.cols()},                
            { m.stride(), sizeof(uint8_t) }
        );
    });

    py::class_<BufferPointCloud>(m, "BufferPointCloud", py::buffer_protocol())
   .def_buffer([](BufferPointCloud &m) -> py::buffer_info {
        return py::buffer_info(
            m.data(),                              
            sizeof(int16_t),                         
            py::format_descriptor<int16_t>::format(),
            3,                                     
            { m.rows(), m.cols(), size_t(3)},                
            { m.stride(),
              sizeof(int16_t) * 3,
              sizeof(int16_t)
            }
        );
    });


    py::class_<Calibration>(m, "Calibration")
        .def(py::init())
        .def("getSize", &Calibration::getSize)
        .def("getIntrinsicsMatrix", &Calibration::getIntrinsicsMatrix,
            py::arg("extended"))
        .def("getDistortionParams", &Calibration::getDistortionParams)
        .def("getRotationMatrix", &Calibration::getRotationMatrix)
        .def("getTranslationVector", &Calibration::getTranslationVector)
        .def("getCameraPose", &Calibration::getCameraPose)
        .def_readonly("width", &Calibration::width);


    py::class_<Imu_sample>(m, "Imu_sample")
        .def(py::init())
        .def_readonly("temperature", &Imu_sample::temperature)
        .def_readonly("acc_x", &Imu_sample::acc_x)
        .def_readonly("acc_y", &Imu_sample::acc_y)
        .def_readonly("acc_z", &Imu_sample::acc_z)
        .def_readonly("acc_timestamp_usec", &Imu_sample::acc_timestamp_usec)
        .def_readonly("gyro_x", &Imu_sample::gyro_x)
        .def_readonly("gyro_y", &Imu_sample::gyro_y)
        .def_readonly("gyro_z", &Imu_sample::gyro_z)
        .def_readonly("gyro_timestamp_usec", &Imu_sample::gyro_timestamp_usec);

    
    py::class_<ColorData>(m, "ColorData")
        .def(py::init())
        .def_readonly("buffer", &ColorData::buffer)
        .def_readonly("timestamp_nsec", &ColorData::timestamp_nsec);

    
    py::class_<DepthData>(m, "DepthData")
        .def(py::init())
        .def_readonly("buffer", &DepthData::buffer)
        .def_readonly("timestamp_nsec", &DepthData::timestamp_nsec);

    py::class_<BodyIndexData>(m, "BodyIndexData")
        .def(py::init())
        .def_readonly("buffer", &BodyIndexData::buffer)
        .def_readonly("timestamp_nsec", &BodyIndexData::timestamp_nsec);


    py::class_<Kinect>(m, "Kinect")
        .def(py::init<uint8_t, int, bool, bool, uint8_t, bool, bool, bool, bool, bool, bool>(),
             py::arg("deviceIndex")=0, py::arg("resolution")=1080, py::arg("wfov")=false,
             py::arg("binned")=true, py::arg("framerate")=30, py::arg("sensorColor")=true,
             py::arg("sensorDepth")=true, py::arg("sensorIR")=true,
             py::arg("imuSensors")=false, py::arg("bodyTracking")=false,
             py::arg("bodyIndex")=false)
        .def("getFrames", &Kinect::getFrames, "Read frames from Kinect",
            py::arg("getColor")=true, py::arg("getDepth")=true,
            py::arg("getIR")=true, py::arg("getSensors")=false,
            py::arg("getBody")=false, py::arg("getBodyIndex")=false)
        .def("getSensorData", &Kinect::getSensorData, "Return sensor struct")
        .def("getColorData", &Kinect::getColorData, "Return color frame")
        .def("getDepthData", &Kinect::getDepthData, "Return depth frame",
            py::arg("align")=false)
        .def("getIRData", &Kinect::getIRData, "Return IR frame")
        .def("getPointCloud", &Kinect::getPointCloud, "Return point cloud")
        .def("getPointCloudColor", &Kinect::getPointCloudColor, "Return point cloud color values")
        .def("savePointCloud", &Kinect::savePointCloud, "Save the current pointcloud to a ply file",
            py::arg("file_name"))
        .def("getDepthCalibration", &Kinect::getDepthCalibration, py::return_value_policy::copy)
        .def("getColorCalibration", &Kinect::getColorCalibration, py::return_value_policy::copy)
        .def("getSerialNumber", &Kinect::getSerialNumber, "Return the serial number of the kinect")
        .def("close", &Kinect::close)
        .def("setExposure", &Kinect::setExposure, "Set exposure time",
            py::arg("exposure"))
        .def("setGain", &Kinect::setExposure, "Set sensor gain",
            py::arg("gain"))
        .def("getExposure", &Kinect::getExposure, "Get exposure time")
        .def("map_coords_color_to_depth", &Kinect::map_coords_color_to_depth,
            "Map color pixel coordinates to depth image space")
        .def("map_coords_color_to_3D", &Kinect::map_coords_color_to_3D,
            "Map color pixel coordinates to 3D space of depth camera",
            py::arg("color_coords"), py::arg("depth_reference")=true)
        .def("map_coords_depth_to_color", &Kinect::map_coords_depth_to_color,
            "Map depth pixel coordinates to color image space")
        .def("map_coords_depth_to_3D", &Kinect::map_coords_depth_to_3D,
            "Map depth pixel coordinates to 3D space of depth camera",
            py::arg("depth_coords"))
        .def("getNumBodies", &Kinect::getNumBodies, "Get number of bodies found")
        .def("getBodies", &Kinect::getBodies, "Get bodies list")
        .def("getBodyIndexMap", &Kinect::getBodyIndexMap, "Return body index map frame",
            py::arg("returnId")=false);
}