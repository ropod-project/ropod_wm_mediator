# ropod_wm_mediator
Makes queries to world models

#### Message definitions
See message definitions in [ropod_com_mediator](https://git.ropod.org/ropod/communication/ropod_com_mediator/)
[Message definitions](https://git.ropod.org/ropod/communication/ropod_com_mediator/blob/master/doc/ropod_msgs.md)

#### OSM world model
Queries to OSM world model can be made using `/ropod_wm_mediator/osm_query` service
* It requires 3 inputs:
- ids: array of element id's we are interested in
- data_type: way,node,relation
- query_type:
  - 'info' - returns geometry and semantic info of all element ids (all data_types)
  - 'area' - returns areas corresponding to given topology nodes (only for nodes)
  - 'graph' - returns graphs containing given topology nodes (only for nodes)

Note: Update overpass url and global reference point in launch file before launching!

#### Launch

To launch world model mediator
```
roslaunch ropod_wm_mediator wm_mediator.launch
```

To launch query interface only to OSM world model
```
roslaunch ropod_wm_mediator osm_wm_bridge.launch
```

#### OSM world model query examples

##### Example 1: Find geometry and semantic info of node with id 3141

Input:
```
"ids:
- 3141
query_type: 'info'
data_type: 'node'"  
```
Output:
```
nodes: 
  - 
    id: 3141
    x: 1.92652487755
    y: -2.3814702034
    tags: 
      - 
        key: "levels"
        value: "-1;4;"
      - 
        key: "highway"
        value: "elevator"
ways: []
relations: []
```

##### Example 2: Find area corresponding to node with id 3141

Input:
```
"ids:
- 3141
query_type: 'area'
data_type: 'node'" 
```
Output:
```
nodes: []
ways: 
  - 
    id: 1126
    nodes: 
      - 
        id: 4626
        x: 0.571497797966
        y: -1.46499931812
        tags: []
      - 
        id: 4627
        x: 3.29427337646
        y: -1.45213878155
        tags: []
      - 
        id: 4628
        x: 3.30230808258
        y: -3.1532971859
        tags: []
      - 
        id: 4629
        x: 0.565240561962
        y: -3.16622519493
        tags: []
    tags: 
      - 
        key: "type"
        value: "elevator"
      - 
        key: "indoor"
        value: "room"
      - 
        key: "ref"
        value: "AMK_B_E1"
relations: []
```

##### Example 3: Find graphs containing node with id 3141

Input:
```
"ids:
- 3141
query_type: 'graph'
data_type: 'node'" 
```
Output
```
nodes: []
ways: 
  - 
    id: 641
    nodes: 
      - 
        id: 3117
        x: 0.340288668871
        y: -2.44455742836
        tags: []
      - 
        id: 3141
        x: 1.92652487755
        y: -2.3814702034
        tags: 
          - 
            key: "levels"
            value: "-1;4;"
          - 
            key: "highway"
            value: "elevator"
      - 
        id: 3142
        x: -1.05341517925
        y: -2.4177839756
        tags: []
      - 
        id: 3143
        x: -0.958890318871
        y: -5.78637361526
        tags: []
    tags: 
      - 
        key: "lanes:backward"
        value: "1"
      - 
        key: "lanes"
        value: "2"
      - 
        key: "lanes:forward"
        value: "1"
      - 
        key: "highway"
        value: "footway"
      - 
        key: "level"
        value: "-1"
  - 
    id: 1144
    nodes: 
      - 
        id: 3141
        x: 1.92652487755
        y: -2.3814702034
        tags: 
          - 
            key: "levels"
            value: "-1;4;"
          - 
            key: "highway"
            value: "elevator"
      - 
        id: 4625
        x: 0.225526228547
        y: -2.35614800453
        tags: []
      - 
        id: 4680
        x: -1.30978298187
        y: 1.92850673199
        tags: []
      - 
        id: 4681
        x: -1.73295259476
        y: -2.28756690025
        tags: []
    tags: 
      - 
        key: "lanes:backward"
        value: "1"
      - 
        key: "level"
        value: "4"
      - 
        key: "oneway"
        value: "no"
      - 
        key: "lanes"
        value: "2"
      - 
        key: "lanes:forward"
        value: "1"
      - 
        key: "highway"
        value: "footway"
relations: []
```

##### Example 4: Find geometric and semantic info for way with id 1144

Input:
```
"ids:
- 1144
query_type: 'info'
data_type: 'way'" 
```
Output
```
nodes: []
ways: 
  - 
    id: 1144
    nodes: 
      - 
        id: 3141
        x: 1.92652487755
        y: -2.3814702034
        tags: 
          - 
            key: "levels"
            value: "-1;4;"
          - 
            key: "highway"
            value: "elevator"
      - 
        id: 4625
        x: 0.225526228547
        y: -2.35614800453
        tags: []
      - 
        id: 4680
        x: -1.30978298187
        y: 1.92850673199
        tags: []
      - 
        id: 4681
        x: -1.73295259476
        y: -2.28756690025
        tags: []
    tags: 
      - 
        key: "lanes:backward"
        value: "1"
      - 
        key: "level"
        value: "4"
      - 
        key: "oneway"
        value: "no"
      - 
        key: "lanes"
        value: "2"
      - 
        key: "lanes:forward"
        value: "1"
      - 
        key: "highway"
        value: "footway"
relations: []
```
Note: `area` and `graph` query types are not supported for OSM ways


##### Example 5: Find details of relation with id 5

Input:
```
"ids:
- 5   
query_type: 'info'
data_type: 'relation'" 
```
Output:
```
nodes: []
ways: []
relations: 
  - 
    id: 5
    tags: 
      - 
        key: "type"
        value: "elevator"
      - 
        key: "ref"
        value: "AMK_B_E1"
    members: 
      - 
        id: 1319
        type: "way"
        role: "local_connection"
      - 
        id: 1323
        type: "way"
        role: "local_connection"
      - 
        id: 163
        type: "relation"
        role: "local_area"
      - 
        id: 3141
        type: "node"
        role: "topology"
      - 
        id: 1126
        type: "way"
        role: "geometry"
```
Note: `area` and `graph` query types are not supported for OSM relations
