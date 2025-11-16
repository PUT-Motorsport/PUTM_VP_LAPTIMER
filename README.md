**PUTM Lap Timer**
=====================

**Overview**
------------

The PUTM Lap Timer is a ROS2 package designed to measure lap times for the Poznan University of Technology Racing Team. The package uses GPS data to track the vehicle's position and calculate lap times.

**Features**
------------

* Measures lap times using GPS data
* Calculates delta time between current lap and reference lap
* Publishes lap time data to a ROS topic
* Configurable parameters for GPS data processing

**Usage**
-----

1. Launch the ROS node: `ros2 run putm_lap_timer lap_timer`
2. Configure the GPS device to provide NavSatFix messages
3. The lap timer will start measuring lap times and publishing data to the `/putm_vcl/lap_timer` topic

**Configuration**
-------------

The package provides several configurable parameters:

* `START_LAT` and `START_LON`: coordinates of the start/finish line
* `LAP_DISTANCE`: distance between start/finish line and sector points
* `DELTA_DISTANCE`: minimum distance between sector points

These parameters can be modified in the `lap_timer.cpp` file.


**Testing**
-------
Clone the repository: 
```bash
git clone https://github.com/PUT-Motorsport/PUTM_VP_LAPTIMER.git
```

Change branch to test branch

```bash
git checkout test
```
In order to read the db3 file you need to source putm_ws:
```bash
git clone --recurse-submodules git@github.com:PUT-Motorsport/putm_ws.git
cd putm_ws
colcon build
source install/setup.bash
```
Run plotjuggler

```bash
ros2 run plotjuggler plotjuggler
```
Open db3 file with metadata.yaml. Make sure that ROS2 Topic Re-Publisher box is checked.

Enter your built LapTimer workspace
```bash
cd putm_lap_timer
source install/setup.bash
ros2 run putm_lap_timer lap_timer
```


**License**
-------

This package is licensed under the Apache-2.0 license.

**Acknowledgments**
----------------

This package was developed by the Poznan University of Technology Racing Team.