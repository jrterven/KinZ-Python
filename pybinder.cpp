#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "kinect.h"

namespace py = pybind11;

PYBIND11_MODULE(pyk4, m) {
    
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

    py::class_<Calibration>(m, "Calibration")
        .def(py::init())
        .def("getSize", &Calibration::getSize)
        .def("getIntrinsicsMatrix", &Calibration::getIntrinsicsMatrix,
            py::arg("extended"))
        .def("getDistortionParams", &Calibration::getDistortionParams)
        .def("getRotationMatrix", &Calibration::getRotationMatrix)
        .def("getTranslationVector", &Calibration::getTranslationVector)
        .def("getCameraPose", &Calibration::getCameraPose);

    py::class_<Kinect>(m, "Kinect")
        .def(py::init<uint8_t, int, bool, bool, uint8_t, bool, bool, bool>(), py::arg("deviceIndex")=0, py::arg("resolution")=1080, py::arg("wfov")=false,
            py::arg("binned")=true, py::arg("framerate")=30, py::arg("sensorColor")=true, py::arg("sensorDepth")=true, py::arg("sensorIR")=true)
        .def("getFrames", &Kinect::getFrames, "Read frames from Kinect",
            py::arg("getColor")=true, py::arg("getDepth")=true, py::arg("getIR")=true)
        .def("getColorData", &Kinect::getColorData, "Return color frame")
        .def("getDepthData", &Kinect::getDepthData, "Return depth frame",
            py::arg("align")=false)
        .def("getIRData", &Kinect::getIRData, "Return IR frame")
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
            "Map depth pixel coordinates to color image space");
}