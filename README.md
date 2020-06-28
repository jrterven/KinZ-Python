# Python Library for Kinect for Azure
At the time of this writing, there are no official python bindings for Kinect for Azure.  
This library allows to use Kinect for Azure directly in Python.


## Installation:
First install the k4a library as described in the [official repo](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).

Then install the requirements (see environment.yaml).
To create a fully functional conda environment run:
```sh
conda env create --file environment.yaml
```

Then install the library with:
```sh
pip install .
```

## Demos
Inside demos directory, you'll find demos showing all the features of the library.  
Currently, there are only 3 demos:
- **streamer-demo.py**: shows how to stream color, depth and IR images at different resolutions.
- **calibration-demo.py**: shows how to extract camera calibration values.
- **map_depth_to_color_and_3d_demo.py**: shows how to map sparse points from depth to color and from depth to 3D.
- **map_color_to_depth_and_3d_demo.py**: shows how to map sparse points from color to depth and from color to 3D.


## Streaming Speed
The streaming speeds in Python are the following:

| Resolution | FPS Unaligned | FPS Aligned Depth2Color |  
|------------|---------------|-------------------------|  
| 1280 x 720   | 30            | 30                      |  
| 1920 x 1080  | 30            | 30                      |  
| 2560 x 1440  | 30            | 22                      |  
| 2048 x 1536  | 30            | 30                      |  
| 3840 x 2160  | 30            | 16                      |  
| 4096 x 3072  | 15            | 9                       |  


## Uninstall:
```sh
pip uninstall pyk4
```