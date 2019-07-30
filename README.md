# Lidar_Obstacle_Detection
use lidar to detection the obstacle in the environment.For every obstacle in the envionment,give it a bounding box.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

## Installation
```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/zhuhao425/Lidar_Obstacle_Detection.git
$> cd Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```
