# ROPOD World model mediator
Makes queries to world models (currently OSM for static and ED for dynamic details)

## Message definitions
See message definitions in [ropod_ros_msgs](https://git.ropod.org/ropod/communication/ropod_ros_msgs)

## Services provided by the world model mediator

#### Get shape server
* Given the area/sub-area it returns the geometry of the area/sub-area

#### Get topology node server
* Given the area/sub-area it returns the topology node of the area/sub-area

#### Get path plan server
* Given the start and destination details, it returns the area level plan for navigation

#### Get nearest WiFi access points
* Given area/sub-area/position details, it returns the nearest WiFi access points

#### Get elevator waypoints
* Given elevator and elevator door id, it returns the waypoints to wait inside and outside the elevator

#### Get objects
* Given the area id and object type, it returns objects of specified type in specified area (currently only dynamic objects)


## Launch

To launch world model mediator
```
roslaunch ropod_wm_mediator wm_mediator.launch
```