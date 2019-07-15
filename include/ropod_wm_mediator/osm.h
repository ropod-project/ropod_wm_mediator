#ifndef OSM_H
#define OSM_H

// C++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
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
#include <ropod_ros_msgs/GetNearestWLANAction.h> 

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
#include <osm_bridge_ros_wrapper/NearestWLANAction.h>

class OSM
{
private:
    ros::NodeHandle nh_;
    bool status_;
    std::string building;    
    actionlib::SimpleActionClient<osm_bridge_ros_wrapper::WMQueryAction> wm_query_ac_;
    actionlib::SimpleActionClient<osm_bridge_ros_wrapper::PathPlannerAction> path_planner_ac_;
    actionlib::SimpleActionClient<osm_bridge_ros_wrapper::NearestWLANAction> nearest_wlan_ac_;
    osm_bridge_ros_wrapper::WMQueryResult wm_query_result_;
    osm_bridge_ros_wrapper::WMQueryResult wm_query_result;
    osm_bridge_ros_wrapper::PathPlannerResult path_planner_result_;
    osm_bridge_ros_wrapper::NearestWLANResult nearest_wlan_result_;
    
    void WMQueryResultCb(const actionlib::SimpleClientGoalState& state,const osm_bridge_ros_wrapper::WMQueryResultConstPtr& result);
    void pathPlannerResultCb(const actionlib::SimpleClientGoalState& state,const osm_bridge_ros_wrapper::PathPlannerResultConstPtr& result);
    void nearestWLANResultCb(const actionlib::SimpleClientGoalState& state,const osm_bridge_ros_wrapper::NearestWLANResultConstPtr& result);
    ropod_ros_msgs::PathPlan decodePathPlan(const std::vector<osm_bridge_ros_wrapper::PlannerArea> &planner_areas);
    ropod_ros_msgs::Position computeWaitingPosition(ropod_ros_msgs::Position elevator, ropod_ros_msgs::Position door, double distance_from_door);
    double wrapToPi(double angle);
    geometry_msgs::Quaternion computeOrientation(ropod_ros_msgs::Position elevator, ropod_ros_msgs::Position waiting_position);

public:
    OSM();
    bool start();
    bool getStatus();
    virtual ~OSM();

    bool getTopologyNode(int id, std::string type, ropod_ros_msgs::Position &position);
    bool getShape(int id, std::string type, ropod_ros_msgs::Shape &shape);
    bool getPathPlan(const ropod_ros_msgs::GetPathPlanGoalConstPtr& goal, ropod_ros_msgs::PathPlan &path_plan);
    bool getNearestWlan(const ropod_ros_msgs::GetNearestWLANGoalConstPtr& goal, ropod_ros_msgs::Position &wlan_pos);
    bool getElevatorWaypoints(int elevator_id, int door_id, geometry_msgs::Pose &inside_pose, geometry_msgs::Pose &outside_pose);

    osm_bridge_ros_wrapper::WMQueryResult getWMQueryResult();

};

#endif /* OSM_H */
