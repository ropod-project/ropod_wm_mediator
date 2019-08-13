#!/usr/bin/env python

# NOTE: This test uses BRSU OSM database

PACKAGE = 'ropod_wm_mediator'
NODE = 'nearest_wlan_test_node'

import rospy

from ropod_ros_msgs.msg import GetNearestWLANAction, GetNearestWLANGoal, Position
from actionlib import SimpleActionClient


class NearestWLANTest(object):

    """ A test for nearest wlan finder action """

    def __init__(self):
        SERVER = "/get_nearest_wlan"
        self.client = SimpleActionClient(SERVER, GetNearestWLANAction)
        self.client.wait_for_server()
        rospy.loginfo("Successfully connected to path planner server")

        req = GetNearestWLANGoal(position=Position(
            x=0.0, y=5.0), is_pos_provided=True, floor='BRSU_L0')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = GetNearestWLANGoal(position=Position(
            x=0.0, y=5.0), is_pos_provided=True, floor='127')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = GetNearestWLANGoal(is_pos_provided=False, area='BRSU_A_L0_A1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = GetNearestWLANGoal(is_pos_provided=False, area='125')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = GetNearestWLANGoal(
            is_pos_provided=False, local_area='BRSU_A_L0_A1_LA1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = GetNearestWLANGoal(is_pos_provided=False, local_area='120')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        rospy.signal_shutdown("Path planner server tests complete")

    def done_cb(self, status, result):
        try:
            assert(status == 3)  # succeded
            assert(abs(result.wlan_pose.x - -0.584245681763) < 0.1)
            assert(abs(result.wlan_pose.y - 22.9997749329) < 0.1)
            rospy.loginfo("Test successfully passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = NearestWLANTest()
    rospy.spin()
