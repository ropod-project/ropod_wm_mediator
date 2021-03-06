#!/usr/bin/env python
import overpass
import utm
import rospy
import actionlib
from ropod_ros_msgs.msg import OSMQueryAction, OSMQueryGoal, OSMQueryResult 
from ropod_ros_msgs.msg import OSMNode, OSMTag, OSMWay, OSMRelation, OSMMember

class OSMWMBridge(object):

  def __init__(self):
      api_url = rospy.get_param('~overpass_url')
      ref_lat = rospy.get_param('~ref_latitude')
      ref_lon = rospy.get_param('~ref_longitude')
      self.origin = utm.from_latlon(ref_lat, ref_lon)
      self.api = overpass.API(endpoint=api_url)
      self.osm_query_server = actionlib.SimpleActionServer('/osm_query', OSMQueryAction, self.query, False)
      self.osm_query_server.start()
      print('OSM query server started!')

  def query(self, req):
      # print('Received new query request - ids:{},query_type:{},data_type:{}'.format(req.ids,req.query_type,req.data_type))
      query_string = self.construct_overpass_query(req.query_type, req.data_type, req.ids)
      res = OSMQueryResult()
      if query_string is not None:
          data = self.make_overpass_request(query_string)
          #print(data)
          node_list, way_list, relation_list = self.construct_output_response(data)
          res.nodes = node_list
          res.ways = way_list
          res.relations = relation_list
      self.osm_query_server.set_succeeded(res)


  '''
  Constructs queries to send to overpass server

  Query types:
    - 'info' - returns geometry and semantic info of all element ids
    - 'area' - returns areas corresponding to given topology nodes
    - 'graph' - returns graphs containing given topology nodes
  '''
  def construct_overpass_query(self, query_type, data_type, ids):
      valid_query_type = {'info','area','graph'}
      valid_data_type = {'node','way','relation'}

      if query_type not in valid_query_type or data_type not in valid_data_type or len(ids) == 0:
          rospy.logerr("Invalid OSM request")
          return None

      query_string = ""

      if query_type == 'info':
          query_string = data_type + "(id:" + ','.join([str(id) for id in ids]) +  ");"
      elif query_type == 'area':
          if data_type == 'node':
              query_string = data_type + "(id:" + ','.join([str(id) for id in ids]) +  ");rel(bn:'topology');way(r._:'geometry');"
          else:
              rospy.logerr("Invalid OSM request")
      elif query_type == 'graph':
          if data_type == 'node':
              query_string = data_type + "(id:" + ','.join([str(id) for id in ids]) +  ");way(bn);"
          else:
              rospy.logerr("Invalid OSM request")
    
      #print(query_string)
      return query_string

  '''
  Constructs output response based on data retrieved from overpass
  '''
  def construct_output_response(self, data):
      node_list = []
      way_list = []
      relation_list = []
      for element in data.get('elements'):
          element_type = element.get('type')
          if element_type == 'node':
              node_list.append(self.get_node(element))
          elif element_type == 'way':
              way_list.append(self.get_way(element))
          elif element_type == 'relation':
              relation_list.append(self.get_relation(element))
      return node_list,way_list,relation_list


  '''
  Makes request to overpass server
  '''
  def make_overpass_request(self, query_string):
      if len(query_string) > 0:
          return self.api.get(query_string) 
      else:
          return self.api.get('out;') 


  '''
  Converts node info obtained from overpass to ROS msg
  '''
  def get_node(self,data):
      n = OSMNode()
      n.id = data.get('id')
      [n.x, n.y] = self.osm_to_local([data.get('lat'),data.get('lon')])
      op_tags = []
      tags = data.get('tags')
      if tags is not None:
          for tag in tags:
              t = OSMTag()
              t.key = tag
              t.value = tags.get(t.key)
              op_tags.append(t)
      n.tags = op_tags  
      return n

  '''
  Converts way info obtained from overpass to ROS msg
  '''
  def get_way(self, data):
      w = OSMWay()
      w.id = data.get('id')
      
      nodes = data.get('nodes')
      query_string = self.construct_overpass_query('info', 'node', nodes)

      nodes_data = self.make_overpass_request(query_string)
      node_list,__,__ = self.construct_output_response(nodes_data)
      w.nodes = node_list

      op_tags = []
      tags = data.get('tags')
      if tags is not None:
          for tag in tags:
              t = OSMTag()
              t.key = tag
              t.value = tags.get(t.key)
              op_tags.append(t)
      w.tags = op_tags  
      return w

  '''
  Converts relation info obtained from overpass to ROS msg
  '''
  def get_relation(self, data):
      r = OSMRelation()
      r.id = data.get('id')

      op_members = []
      members = data.get('members')
      if members is not None:
          for member in members:
              m = OSMMember()
              m.id = member.get('ref')
              m.role = member.get('role')
              m.type = member.get('type')
              op_members.append(m)
      r.members = op_members  
      
      op_tags = []
      tags = data.get('tags')
      if tags is not None:
          for tag in tags:
              t = OSMTag()
              t.key = tag
              t.value = tags.get(t.key)
              op_tags.append(t)
      r.tags = op_tags  
      return r

  '''
  Converts global coordinates to local cartesian coordinates
  '''
  def osm_to_local(self, node):
      temp = utm.from_latlon(node[0], node[1]) 
      return [temp[0] - self.origin[0], temp[1] - self.origin[1]]


if __name__ == "__main__":
    rospy.init_node('OSM_WM_Bridge')
    osm_wm_bridge = OSMWMBridge()
    
    rospy.spin()
