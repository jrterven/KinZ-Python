#include "utils.h"


#include <iostream>
#include <fstream>
#include <sstream>

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

BufferDepth::BufferDepth() {
    m_data = NULL;
    m_rows = 0;
    m_cols = 0;
    m_stride = 0;
}

BufferDepth::BufferDepth(uint16_t* data, size_t rows, size_t cols, size_t stride) : 
    m_data(data), m_rows(rows), m_cols(cols), m_stride(stride) {
}

BufferDepth::~BufferDepth(){}
uint16_t* BufferDepth::data() {
    return m_data.get(); 
}
size_t BufferDepth::rows() const {
    return m_rows; 
}
size_t BufferDepth::cols() const { 
    return m_cols; 
}
size_t BufferDepth::stride() const { 
    return m_stride; 
}

BufferColor::BufferColor() {
    m_data = NULL;
    m_rows = 0;
    m_cols = 0;
    m_stride = 0;
}
BufferColor::BufferColor(uint8_t* data, size_t rows, size_t cols, size_t stride) : 
    m_data(data), m_rows(rows), m_cols(cols), m_stride(stride) {
}
BufferColor::~BufferColor(){}
uint8_t* BufferColor::data() { 
    return m_data.get(); 
}
size_t BufferColor::rows() const { 
    return m_rows; 
}
size_t BufferColor::cols() const { 
    return m_cols; 
}
size_t BufferColor::stride() const { 
    return m_stride; 
}

BufferBodyIndex::BufferBodyIndex() {
    m_data = NULL;
    m_rows = 0;
    m_cols = 0;
    m_stride = 0;
}

BufferBodyIndex::BufferBodyIndex(uint8_t* data, size_t rows, size_t cols, size_t stride) : 
    m_data(data), m_rows(rows), m_cols(cols), m_stride(stride) {
}

BufferBodyIndex::~BufferBodyIndex(){}
uint8_t* BufferBodyIndex::data() {
    return m_data.get(); 
}
size_t BufferBodyIndex::rows() const {
    return m_rows; 
}
size_t BufferBodyIndex::cols() const { 
    return m_cols; 
}
size_t BufferBodyIndex::stride() const { 
    return m_stride; 
}

BufferPointCloud::BufferPointCloud(int16_t* data, size_t rows, size_t cols, size_t stride) : 
    m_data(data), m_rows(rows), m_cols(cols), m_stride(stride) {
}

BufferPointCloud::~BufferPointCloud(){}
int16_t *BufferPointCloud::data() { 
    return m_data.get(); 
}
size_t BufferPointCloud::rows() const { 
    return m_rows; 
}
size_t BufferPointCloud::cols() const { 
    return m_cols; 
}
size_t BufferPointCloud::stride() const { 
    return m_stride; 
}


struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};


void write_point_cloud(const k4a_image_t point_cloud_image,
                               const k4a_image_t color_image,
                               const char *file_name) {
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    for (int i = 0; i < width * height; i++)
    {
        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
        ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
