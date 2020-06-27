# Python Library for Kinect for Azure
At the time of this writing, there are no official python bindings for Kinect for Azure.  
This library allows to use Kinect for Azure directly in Python.


## Installation:
First install the k4a library as described in the [official repo](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).

Then install the Python library with:
```sh
pip install .
```

## Demos
Inside demos directory, you'll find demos showing all the features of the library.  
Currently, there are only 3 demos:
- **streamer-demo.py**: shows how to stream color, depth and IR images at different resolutions.
- **calibration-demo.py**: shows how to extract camera calibration values.
- **sparse-mapping-demo.py**: shows how to map sparse points from depth to color instead of using full aligned frames.


## Streaming Speed
The streaming speeds in Python are the following:

| Resolution | FPS Unaligned | FPS Aligned Depth2Color |  
|------------|---------------|-------------------------|  
| 1280 x 720   | 25            | 22                      |  
| 1920 x 1080  | 24            | 18                      |  
| 2560 x 1440  | 22            | 14                      |  
| 2048 x 1536  | 23            | 16                      |  
| 3840 x 2160  | 20            | 10                      |  
| 4096 x 3072  | 12            | 7                       |  


## Uninstall:
```sh
pip uninstall pyk4
```