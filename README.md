# velodyne_camera_calibration

![demo](https://github.com/Sadaku1993/velodyne_camera_calibration/blob/master/projection.gif)

## Requirement
- ros (kinetic)
- OpenCV 3.4
- PCL 1.8

## SetUp

### Download Velodyne package

```shell
sudo apt-get install libpcap-dev
cd ~/catkin_ws/src/
git clone https://github.com/ros-drivers/velodyne
cd ~/catkin_ws
catkin_make
```

### Download Sensor Fusion package
```bash
cd ~/catkin_ws/src/
git clone https://github.com/Sadaku1993/velodyne_camera_calibration
roscd
catkin_make
```

### Download Bag data
Download [data.bag](https://drive.google.com/file/d/1aP3foMD4WPVQz0ZLOP1HcivBn1AOtJz4/view?usp=sharing) and move to bagfiles.

```bash
velodyne_camera_calibration/
  bagfiles/
    data.bag
  CMakeLists.txt
  src/
  config/
  include/
  package.xml
  launch/
```

## How to Use

```
roslaunch velodyne_camera_calibration sensor_fusion.launch
```


## Reference
This code is based on [velo2cam_calibration](http://wiki.ros.org/velo2cam_calibration).
