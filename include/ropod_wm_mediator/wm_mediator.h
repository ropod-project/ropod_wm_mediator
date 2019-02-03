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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// ROPOD
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/SubArea.h>
#include <ropod_ros_msgs/Position.h>
#include <ropod_ros_msgs/Shape.h>
#include <ropod_ros_msgs/PathPlan.h>
 
#include <ropod_ros_msgs/GetTopologyNodeAction.h> 
#include <ropod_ros_msgs/GetShapeAction.h> 
#include <ropod_ros_msgs/GetPathPlanAction.h> 
#include <ropod_ros_msgs/GetElevatorWaypointsAction.h> 

// OBL
#include <osm_bridge_ros_wrapper/Area.h>
#include <osm_bridge_ros_wrapper/Corridor.h>
#include <osm_bridge_ros_wrapper/Elevator.h>
#include <osm_bridge_ros_wrapper/Stairs.h>
#include <osm_bridge_ros_wrapper/Room.h>
#include <osm_bridge_ros_wrapper/LocalArea.h>
#include <osm_bridge_ros_wrapper/Point.h>

#include <osm_bridge_ros_wrapper/WMQueryAction.h>
#include <osm_bridge_ros_wrapper/PathPlannerAction.h>


class WMMediator
{
private:
    ros::NodeHandle nh_;
    std::string building;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetTopologyNodeAction> get_topology_node_server;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetShapeAction> get_shape_server;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetPathPlanAction> get_path_planner_server;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetElevatorWaypointsAction> get_elevator_waypoints_server;
    actionlib::SimpleActionClient<osm_bridge_ros_wrapper::WMQueryAction> wm_query_ac;
    actionlib::SimpleActionClient<osm_bridge_ros_wrapper::PathPlannerAction> path_planner_ac;
    osm_bridge_ros_wrapper::WMQueryResult wm_query_result;
    osm_bridge_ros_wrapper::PathPlannerResult path_planner_result;
    
    void WMQueryResultCb(const actionlib::SimpleClientGoalState& state,const osm_bridge_ros_wrapper::WMQueryResultConstPtr& result);
    void PathPlannerResultCb(const actionlib::SimpleClientGoalState& state,const osm_bridge_ros_wrapper::PathPlannerResultConstPtr& result);
    void get_topology_node_execute(const ropod_ros_msgs::GetTopologyNodeGoalConstPtr& goal);
    void get_shape_execute(const ropod_ros_msgs::GetShapeGoalConstPtr& goal);
    void get_path_plan_execute(const ropod_ros_msgs::GetPathPlanGoalConstPtr& goal);
    void get_elevator_waypoints_execute(const ropod_ros_msgs::GetElevatorWaypointsGoalConstPtr& goal);
    ropod_ros_msgs::PathPlan decode_path_plan(const std::vector<osm_bridge_ros_wrapper::PlannerArea> &planner_areas);
    bool get_topology_node(int id, std::string type, ropod_ros_msgs::Position &position);
    bool get_shape(int id, std::string type, ropod_ros_msgs::Shape &shape);

public:
    WMMediator();
    virtual ~WMMediator();
};

#endif /* WM_MEDIATOR_H */
