# KinZ for Python A library for Azure Kinect
At the time of this writing, there are no official python bindings for Kinect for Azure.  
This library allows to use Azure Kinect directly in Python.


## Installation:
1. Install the Azure Kinect SDK as described [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)  where it says Microsoft installer. For Windows, Download the .exe and follow the steps. For Ubuntu use the *sudo apt install* commands shown in the same page.
2. For Body tracking functionality (optional), you need an NVIDIA GPU and install CUDA. Download from [here](https://developer.nvidia.com/cuda-downloads?/).
3. For Body tracking functionality (optional), install the Azure Kinect Body Tracking SDK. For Windows, download the msi installer from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/body-sdk-download). For Ubuntu simply run the *sudo apt install* command provided in the webpage.
4. Before compiling the code for Matlab, make sure the Kinect works correctly using the viewers provided my Microsoft, e.g. *C:\Program Files\Azure Kinect SDK v1.4.1\tools\k4aviewer.exe* and *C:\Program Files\Azure Kinect Body Tracking SDK\tools\k4abt_simple_3d_viewer.exe*. In Linux just type *k4aviewer* or *k4abt_simple_3d_viewer* in the terminal.
5. Install the Python requirements (see environment.yaml).
To create a fully functional conda environment run:
```sh
conda env create --file environment.yaml
```
6. Install the library with:
```sh
pip install .
```
7. To install KinZ with body tracking functionality run:
```sh
CFLAGS="-l:libk4abt.so -DBODY" pip install .
```


## Demos
Inside demos directory, you'll find demos showing all the features of the library.  
Currently, there are only 6 demos:
- **cameras_sensors_demo**: shows how to get color, depth IR, and IMU sensors.
- **calibration-demo.py**: shows how to extract camera calibration values.
- **map_depth_to_color_and_3d_demo.py**: shows how to map sparse points from depth to color and from depth to 3D.
- **map_color_to_depth_and_3d_demo.py**: shows how to map sparse points from color to depth and from color to 3D.
- **pointcloud_demo**: shows how to get the colored pointcloud and visualize it with open3D.
- **body_tracking_demo**: shows how to get body tracking information and visualize it.

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
pip uninstall kinz
```

For more details checkout the [wiki pages](https://github.com/jrterven/KinZ-Python/wiki). 
