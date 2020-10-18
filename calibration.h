#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>

namespace py = pybind11;

class Calibration {
public:
    int width, height;
    float cx, cy;
    float fx, fy;
    float k1, k2, k3, k4, k5, k6;
    float p1, p2;
    float codx, cody;
    float metric_radius; 
    float rotation[9];
    float translation[3];

    py::list get_size();
    py::array_t<double> get_intrinsics_matrix(bool);
    py::array_t<double> get_distortion_params();
    py::array_t<double> get_rotation_matrix();
    py::array_t<double> get_translation_vector();
    py::array_t<double> get_camera_pose();
};

#endif
