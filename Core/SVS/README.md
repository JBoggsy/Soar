# SVS Compilation

Soar's Spatial-Visual System (SVS) includes several components. The Spatial Scene/scene graph can be built along with Soar in the normal way and doesn't require any extra dependencies or build steps. The visual components of SVS, however, do require some additional work. This section addresses the added dependencies and build steps for building SVS with the visual components.

## Requirements
**OpenCV** 

## Installation
Install dependencies
```bash
sudo apt install build-essential swig openjdk-11-jdk python-all-dev libopencv-dev
```

Export environment variables
```bash
export LD_LIBRARY_PATH=$LD_LIBRARYH_PATH:/usr/local/lib:$SOAR_HOME
```

Build with scons
```bash
python scons/scons.py all --use-opencv
```