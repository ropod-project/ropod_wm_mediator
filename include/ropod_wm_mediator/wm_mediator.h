#ifndef WM_MEDIATOR_H
#define WM_MEDIATOR_H

// C++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

// ROPOD
#include <ropod_wm_mediator/osm.h>
#include <ropod_wm_mediator/ed.h>
#include <ftsm_base.h>

using namespace ftsm;

class WMMediator : public FTSMBase
{
protected:
    ros::NodeHandle nh_;
    OSM osm_;
    ED ed_;

    actionlib::SimpleActionServer<ropod_ros_msgs::GetTopologyNodeAction> get_topology_node_server_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetShapeAction> get_shape_server_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetPathPlanAction> get_path_plan_server_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetElevatorWaypointsAction> get_elevator_waypoints_server_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetNearestWLANAction> get_nearest_wlan_server_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetObjectsAction> get_objects_server_;

    void getTopologyNodeService(const ropod_ros_msgs::GetTopologyNodeGoalConstPtr& goal);
    void getShapeService(const ropod_ros_msgs::GetShapeGoalConstPtr& goal);
    void getPathPlanService(const ropod_ros_msgs::GetPathPlanGoalConstPtr& goal);
    void getNearestWlanService(const ropod_ros_msgs::GetNearestWLANGoalConstPtr& goal);
    void getElevatorWaypointsService(const ropod_ros_msgs::GetElevatorWaypointsGoalConstPtr& goal);
    void getObjectsService(const ropod_ros_msgs::GetObjectsGoalConstPtr& goal);

public:
    WMMediator();
    WMMediator(bool debug);

    /* FTSM base functions */
    virtual std::string init();
    std::string running();
    std::string recovering();

    virtual ~WMMediator();
};

#endif /* WM_MEDIATOR_H */
