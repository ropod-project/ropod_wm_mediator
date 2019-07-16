#!/usr/bin/env python

PACKAGE = 'ropod_wm_mediator'
NODE = 'shape_test_node'

import rospy

from ropod_ros_msgs.msg import GetShapeAction, GetShapeGoal
from actionlib import SimpleActionClient


class ShapeTest(object):

    """ A test for elevator waypoints """

    def __init__(self):
        SERVER = "/get_shape"
        self.client = SimpleActionClient(SERVER, GetShapeAction)
        self.client.wait_for_server()
        rospy.loginfo("Successfully connected to shape server")

        req = GetShapeGoal(id=152, type="local_area")

        self.client.send_goal(req, done_cb=self.cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Shape server test complete")

    def cb(self, status, result):
        try:
            print(result.shape.vertices)
            assert(len(result.shape.vertices) > 0)
            rospy.loginfo("Test successfully passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = ShapeTest()
    rospy.spin()
