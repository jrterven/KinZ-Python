#ifndef KINUTILS_H
#define KINUTILS_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>

class BufferDepth {
public:
    BufferDepth(uint16_t* data, size_t rows, size_t cols, size_t stride) : 
        m_data(data), m_rows(rows), m_cols(cols), m_stride(stride) {
    }
    ~BufferDepth(){}
    uint16_t *data() { return m_data.get(); }
    size_t rows() const { return m_rows; }
    size_t cols() const { return m_cols; }
    size_t stride() const { return m_stride; }
private:
    size_t m_rows, m_cols, m_stride;
    std::shared_ptr<uint16_t> m_data;
};

class BufferColor {
public:
    BufferColor(uint8_t* data, size_t rows, size_t cols, size_t stride) : 
        m_data(data), m_rows(rows), m_cols(cols), m_stride(stride) {
    }
    ~BufferColor(){}
    uint8_t *data() { return m_data.get(); }
    size_t rows() const { return m_rows; }
    size_t cols() const { return m_cols; }
    size_t stride() const { return m_stride; }
private:
    size_t m_rows, m_cols, m_stride;
    std::shared_ptr<uint8_t> m_data;
};

#endif
