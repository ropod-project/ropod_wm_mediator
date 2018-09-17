# ropod_wm_mediator
Makes queries to world models

#### Message definitions
See message definitions in [ropod_com_mediator](https://git.ropod.org/ropod/communication/ropod_com_mediator/)
[Message definitions](https://git.ropod.org/ropod/communication/ropod_com_mediator/blob/master/doc/ropod_msgs.md)

#### To install python dependencies
```
pip install -r requirements.txt
```

#### OSM WM bridge
Provides `/osm_query` action server to make queries to OSM world model
* It requires 3 inputs:
```
- ids: array of element id's we are interested in
- data_type: way,node,relation
- query_type:
  - 'info' - returns geometry and semantic info of all element ids (all data_types)
  - 'area' - returns areas corresponding to given topology nodes (only for nodes)
  - 'graph' - returns graphs containing given topology nodes (only for nodes)
```

Note: Update overpass url and global reference point in launch file before launching!

#### WM mediator
Provides `/get_waypt_position` action server
* Takes list of `Waypoint` ids as input
* Returns list of `Position` in local cartesian coordinate system

Provides `/get_waypt_shape` action server
* Takes list of `Waypoint` ids as input
* Returns list of `Shape`  


#### Launch

To launch world model mediator
```
roslaunch ropod_wm_mediator wm_mediator.launch
```