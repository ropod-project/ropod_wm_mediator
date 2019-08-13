#!/usr/bin/env python

PACKAGE = 'ropod_wm_mediator'
NODE = 'objects_test_node'

import rospy

from ropod_ros_msgs.msg import GetObjectsAction, GetObjectsGoal
from actionlib import SimpleActionClient


class ObjectsTest(object):

    """ A test for elevator waypoints """

    def __init__(self):
        SERVER = "/get_objects"
        self.client = SimpleActionClient(SERVER, GetObjectsAction)
        self.client.wait_for_server()
        rospy.loginfo("Successfully connected to objects server")

        req = GetObjectsGoal(area_id='152', type="mobidik")

        self.client.send_goal(req, done_cb=self.cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Objects server test complete")

    def cb(self, status, result):
        try:
            assert(result.success)
            rospy.loginfo("Test successfully passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = ObjectsTest()
    rospy.spin()
