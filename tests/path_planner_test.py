#!/usr/bin/env python

PACKAGE = 'ropod_wm_mediator'
NODE = 'path_planner_test_node'

import rospy
from ropod_ros_msgs.msg import *
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class PathPlannerTest(object):

    """ A test for path planner """

    def __init__(self):
        SERVER = "/get_path_plan"
        self.client = SimpleActionClient(SERVER, GetPathPlanAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Successfully connected to path planner server")
        rospy.loginfo("Now running tests:")
        rospy.loginfo("Case 1: no bloked connections & traffic rules in effect")
        req = GetPathPlanGoal(start_floor=-1, destination_floor=4, start_area='AMK_D_L-1_C41',\
          destination_area='AMK_B_L4_C1', start_sub_area='AMK_D_L-1_C41_LA1', destination_sub_area='AMK_B_L4_C1_LA1')
        self.client.send_goal(req, done_cb=self.case1_cb)
        self.client.wait_for_result()

        rospy.loginfo("Case 2: connection blocked but traffic rules still in effect")
        blocked_connection1 = BlockedConnection()
        blocked_connection1.start_id = 4825
        blocked_connection1.end_id = 4824

        blocked_connection2 = BlockedConnection()
        blocked_connection2.start_id = 4824
        blocked_connection2.end_id = 5055

        req = GetPathPlanGoal(start_floor=-1, destination_floor=4, start_area='AMK_D_L-1_C41',\
          destination_area='AMK_B_L4_C1', start_sub_area='AMK_D_L-1_C41_LA1', destination_sub_area='AMK_B_L4_C1_LA1',\
          blocked_connections=[blocked_connection1,blocked_connection2], relax_traffic_rules=False)
        self.client.send_goal(req, done_cb=self.case2_cb)
        self.client.wait_for_result()

        rospy.loginfo("Case 3: connection blocked but traffic rules are relaxed")
        blocked_connection1 = BlockedConnection()
        blocked_connection1.start_id = 4825
        blocked_connection1.end_id = 4824

        blocked_connection2 = BlockedConnection()
        blocked_connection2.start_id = 4824
        blocked_connection2.end_id = 5055

        req = GetPathPlanGoal(start_floor=-1, destination_floor=4, start_area='AMK_D_L-1_C41',\
          destination_area='AMK_B_L4_C1', start_sub_area='AMK_D_L-1_C41_LA1', destination_sub_area='AMK_B_L4_C1_LA1',\
          blocked_connections=[blocked_connection1,blocked_connection2], relax_traffic_rules=True)
        self.client.send_goal(req, done_cb=self.case3_cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Path planner server tests complete")

    def case1_cb(self, status, result):
        try:
            #print(result.path_plan)
            assert(result.path_plan)
            rospy.loginfo("Case 1 test successfully passed")
        except Exception as e:
            rospy.logerr("Case 1 test failed")

    def case2_cb(self, status, result):
        try:
            # This should fail as path cannot be planned with bloked connection withou
            # relaxing traffic rules 
            #print(result.path_plan)
            assert(result.path_plan == [])
            rospy.logerr("Case 2 test failed")
        except Exception as e:
            rospy.loginfo("Case 2 test successfully passed")

    def case3_cb(self, status, result):
        try:
            #print(result.path_plan)
            assert(result.path_plan)
            rospy.loginfo("Case 3 test successfully passed")
        except Exception as e:
            rospy.logerr("Case 3 test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = PathPlannerTest()
    rospy.spin()
