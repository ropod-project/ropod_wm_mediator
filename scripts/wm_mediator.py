#!/usr/bin/env python

import rospy
from ropod_ros_msgs.srv import *
from osm_wm_mediator import OSMWMMediator



class WMMediator(object):

  def __init__(self):
    rospy.init_node('WM_mediator')

    # get overpass server address
    api_url = rospy.get_param('~overpass_url')
    ref_lat = rospy.get_param('~ref_latitude')
    ref_lon = rospy.get_param('~ref_longitude')
    osm_wm = OSMWMMediator(api_url, [ref_lat, ref_lon])

    '''
    Service for querying OSM database
    
    Requires 3 inputs:
      - ids: array of element id's we are interested in
      - data_type: way,node,relation
      - query_type:
        - 'info' - returns geometry and semantic info of all element ids
        - 'area' - returns areas corresponding to given topology nodes
        - 'graph' - returns graphs containing given topology nodes
    ''' 
    s = rospy.Service('~osm_query', osm_query, osm_wm.query)



if __name__ == "__main__":
  wm = WMMediator()
  rospy.spin()