#ifndef KINUTILS_H
#define KINUTILS_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <k4a/k4a.h>

class BufferDepth {
public:
    BufferDepth();
    BufferDepth(uint16_t* data, size_t rows, size_t cols, size_t stride);
    ~BufferDepth();
    uint16_t *data();
    size_t rows() const;
    size_t cols() const;
    size_t stride() const;
private:
    std::shared_ptr<uint16_t> m_data;
    size_t m_rows, m_cols, m_stride;
};

class BufferColor {
public:
    BufferColor();
    BufferColor(uint8_t* data, size_t rows, size_t cols, size_t stride);
    ~BufferColor();
    uint8_t *data();
    size_t rows() const;
    size_t cols() const;
    size_t stride() const;
private:
    std::shared_ptr<uint8_t> m_data;
    size_t m_rows, m_cols, m_stride;
};

class BufferBodyIndex {
public:
    BufferBodyIndex();
    BufferBodyIndex(uint8_t* data, size_t rows, size_t cols, size_t stride);
    ~BufferBodyIndex();
    uint8_t *data();
    size_t rows() const;
    size_t cols() const;
    size_t stride() const;
private:
    std::shared_ptr<uint8_t> m_data;
    size_t m_rows, m_cols, m_stride;
};

class BufferPointCloud {
public:
    BufferPointCloud(int16_t* data, size_t rows, size_t cols, size_t stride);
    ~BufferPointCloud();
    int16_t *data();
    size_t rows() const;
    size_t cols() const;
    size_t stride() const;
private:
    std::shared_ptr<int16_t> m_data;
    size_t m_rows, m_cols, m_stride;
};

struct ColorData {
    BufferColor buffer;
    uint64_t timestamp_nsec;
};

struct DepthData {
    BufferDepth buffer;
    uint64_t timestamp_nsec;
};

struct BodyIndexData {
    BufferBodyIndex buffer;
    uint64_t timestamp_nsec;
};

struct Imu_sample {
    float temperature;
    float acc_x, acc_y, acc_z;
    uint64_t acc_timestamp_usec;
    float gyro_x, gyro_y, gyro_z;
    uint64_t gyro_timestamp_usec;
};

void write_point_cloud(const k4a_image_t point_cloud_image,
                               const k4a_image_t color_image,
                               const char *file_name);

#endif
