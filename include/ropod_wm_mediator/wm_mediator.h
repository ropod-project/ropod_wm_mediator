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
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/Position.h>
#include <ropod_ros_msgs/Shape.h>
 
#include <ropod_ros_msgs/GetWayptPositionAction.h> 
#include <ropod_ros_msgs/GetWayptShapeAction.h> 

// OBL
#include <osm_bridge_ros_wrapper/Area.h>
#include <osm_bridge_ros_wrapper/Corridor.h>
#include <osm_bridge_ros_wrapper/Elevator.h>
#include <osm_bridge_ros_wrapper/Stairs.h>
#include <osm_bridge_ros_wrapper/Room.h>
#include <osm_bridge_ros_wrapper/LocalArea.h>

#include <osm_bridge_ros_wrapper/WMQueryAction.h>


class WMMediator
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetWayptPositionAction> get_waypt_position_server;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetWayptShapeAction> get_waypt_shape_server;
    actionlib::SimpleActionClient<osm_bridge_ros_wrapper::WMQueryAction> wm_query_ac;
    osm_bridge_ros_wrapper::WMQueryResult wm_query_result;
    
    void WMQueryResultCb(const actionlib::SimpleClientGoalState& state,const osm_bridge_ros_wrapper::WMQueryResultConstPtr& result);
    void get_waypt_position_execute(const ropod_ros_msgs::GetWayptPositionGoalConstPtr& goal);
    void get_waypt_shape_execute(const ropod_ros_msgs::GetWayptShapeGoalConstPtr& goal);

public:
    WMMediator();
    virtual ~WMMediator();
};

#endif /* WM_MEDIATOR_H */
