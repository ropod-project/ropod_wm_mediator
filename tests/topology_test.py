#!/usr/bin/env python

PACKAGE = 'ropod_wm_mediator'
NODE = 'topology_node_test_node'

import rospy

from ropod_ros_msgs.msg import GetTopologyNodeAction, GetTopologyNodeGoal
from actionlib import SimpleActionClient


class TopologyNodeTest(object):

    """ A test for elevator waypoints """

    def __init__(self):
        SERVER = "/get_topology_node"
        self.client = SimpleActionClient(SERVER, GetTopologyNodeAction)
        self.client.wait_for_server()
        rospy.loginfo("Successfully connected to shape server")

        req = GetTopologyNodeGoal(id=152, type="local_area")

        self.client.send_goal(req, done_cb=self.cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Shape server test complete")

    def cb(self, status, result):
        try:
            print(result.position)
            assert(result.position)
            rospy.loginfo("Test successfully passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = TopologyNodeTest()
    rospy.spin()
