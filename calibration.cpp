#include "calibration.h"

py::list Calibration::getSize() { 
    py::list res;
    res.append(width);
    res.append(height);
    return res;
}

py::array_t<double> Calibration::getIntrinsicsMatrix(bool extended=false) {
    int msize = 3;
    if(extended)
        msize = 4;
    size_t size = msize*msize;

    double *intrinsics = new double[size];
    for(unsigned int i=0; i<size; i++)
        intrinsics[i] = 0;

    if(extended) {
        intrinsics[0] = fx;
        intrinsics[2] = cx;
        intrinsics[5] = fy;
        intrinsics[6] = cy;
        intrinsics[10] = 1;
        intrinsics[15] = 1;
    }
    else {
        intrinsics[0] = fx;
        intrinsics[2] = cx;
        intrinsics[4] = fy;
        intrinsics[5] = cy;
        intrinsics[8] = 1;
    }

    // Python object that frees the allocated memory when destroyed
    py::capsule free_when_done(intrinsics, [](void *f) {
        double *intrinsics = reinterpret_cast<double *>(f);
        delete[] intrinsics;
    });

    return py::array_t<double>(
        {msize, msize}, // shape
        {msize*8, 8}, // C-style contiguous strides for double
        intrinsics, // the data pointer
        free_when_done); // numpy array references this parent
}

py::array_t<double> Calibration::getDistortionParams() {
    constexpr size_t size = 1*8;
    double *dist_params = new double[size];
    dist_params[0] = k1;
    dist_params[1] = k2;
    dist_params[2] = p1;
    dist_params[3] = p2;
    dist_params[4] = k3;
    dist_params[5] = k4;
    dist_params[6] = k5;
    dist_params[7] = k6;

    // Python object that frees the allocated memory when destroyed
    py::capsule free_when_done(dist_params, [](void *f) {
        double *dist_params = reinterpret_cast<double *>(f);
        delete[] dist_params;
    });

    return py::array_t<double>(
        {1, 8}, // shape
        {1*8, 8}, // strides
        dist_params, // data pointer
        free_when_done); // numpy array references this parent
}

py::array_t<double> Calibration::getRotationMatrix() {
    constexpr size_t size = 3*3;
    double *rot = new double[size];
    for(unsigned int i=0; i<size; i++)
        rot[i] = rotation[i];

    // Python object that frees the allocated memory when destroyed
    py::capsule free_when_done(rot, [](void *f) {
        double *rot = reinterpret_cast<double *>(f);
        delete[] rot;
    });

    return py::array_t<double>(
        {3, 3}, // shape
        {3*8, 8}, // strides
        rot, // data pointer
        free_when_done); // numpy array references this parent
}

py::array_t<double> Calibration::getTranslationVector() {
    constexpr size_t size = 3*1;
    double *t = new double[size];
    for(unsigned int i=0; i<size; i++)
        t[i] = translation[i];

    // Python object that frees the allocated memory when destroyed
    py::capsule free_when_done(t, [](void *f) {
        double *t = reinterpret_cast<double *>(f);
        delete[] t;
    });

    return py::array_t<double>(
        {3, 1}, // shape
        {1*8, 8}, // strides
        t, // data pointer
        free_when_done); // numpy array references this parent
}

py::array_t<double> Calibration::getCameraPose() {
    constexpr size_t size = 4*4;
    double *pose = new double[size];
    for(unsigned int i=0; i<size; i++)
        pose[i] = 0;
    for(unsigned int l=0; l<3; l++) {
        for(int c=0; c<3; c++)
            pose[l*4 + c] = rotation[l*3 + c];
        pose[l*4 + 3] = translation[l];
    }
    pose[15] = 1;

    // Python object that frees the allocated memory when destroyed
    py::capsule free_when_done(pose, [](void *f) {
        double *pose = reinterpret_cast<double *>(f);
        delete[] pose;
    });

    return py::array_t<double>(
        {4, 4}, // shape
        {4*8, 8}, // strides
        pose, // data pointer
        free_when_done); // numpy array references this parent
}
