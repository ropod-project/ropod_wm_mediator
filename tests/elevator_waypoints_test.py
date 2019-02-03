#!/usr/bin/env python

PACKAGE = 'ropod_wm_mediator'
NODE = 'elevator_waypoints_test_node'

import rospy
from ropod_ros_msgs.msg import *
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class ElevatorWaypointsTest(object):

    """ A test for elevator waypoints """

    def __init__(self):
        SERVER = "/get_elevator_waypoints"
        self.client = SimpleActionClient(SERVER, GetElevatorWaypointsAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Successfully connected to elevator waypoints server")
        
        req = GetElevatorWaypointsGoal(elevator_id = 121, door_id = 196)
        self.client.send_goal(req, done_cb=self.cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Path planner server tests complete")

    def cb(self, status, result):
        try:
            print(result.wp_inside)
            print(result.wp_outside)
            assert(result.wp_inside)
            assert(result.wp_outside)
            rospy.loginfo("Test successfully passed")
        except Exception as e:
            rospy.logerr("Test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = ElevatorWaypointsTest()
    rospy.spin()
