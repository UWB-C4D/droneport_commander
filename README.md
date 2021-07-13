# droneport_commander

Commander for DronePort Battery Management System

# Install and run

Drone communication layer is based on a pymavlink package (https://mavlink.io/en/mavgen_python/, https://www.samba.org/tridge/UAV/pymavlink/apidocs/). Pymavlink supports both MAVLink 1 and 2 and its implementations for PX4 and ArduPilot.
```
pip install pymavlink
```

Testing comm code is implemented in "droneport_commander.ipynb".

Rewrite home coordinates to location next to NTIS/FAV building in the environment variables for usage with standard PX4_SITL:
```
export PX4_HOME_LAT=49.7264592
export PX4_HOME_LON=13.3508606
export PX4_HOME_ALT=350.0
```

Headless mode for Gazebo:
```
HEADLESS=1 make px4_sitl gazebo
```

OR use with docker:
```
docker run --rm -it --env PX4_HOME_LAT=49.7264592 --env PX4_HOME_LON=13.3508606 --env PX4_HOME_ALT=350.0 jonasvautherin/px4-gazebo-headless

```

Repository is supplemented with QGroundControl mission plans in 'data' folder.
