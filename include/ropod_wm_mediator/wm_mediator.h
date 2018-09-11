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
#include <ropod_ros_msgs/OSMNode.h>
#include <ropod_ros_msgs/OSMWay.h>
#include <ropod_ros_msgs/OSMRelation.h>
#include <ropod_ros_msgs/OSMTag.h>
#include <ropod_ros_msgs/OSMMember.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/Position.h>
#include <ropod_ros_msgs/Shape.h>

#include <ropod_ros_msgs/OSMQueryAction.h> 
#include <ropod_ros_msgs/GetWayptPositionAction.h> 
#include <ropod_ros_msgs/GetWayptShapeAction.h> 


class WMMediator
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetWayptPositionAction> get_waypt_position_server;
    actionlib::SimpleActionServer<ropod_ros_msgs::GetWayptShapeAction> get_waypt_shape_server;
    actionlib::SimpleActionClient<ropod_ros_msgs::OSMQueryAction> ac;
    ropod_ros_msgs::OSMQueryResult osm_query_result;
    
    void OSMQueryResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::OSMQueryResultConstPtr& result);
    void get_waypt_position_execute(const ropod_ros_msgs::GetWayptPositionGoalConstPtr& goal);
    void get_waypt_shape_execute(const ropod_ros_msgs::GetWayptShapeGoalConstPtr& goal);

public:
    WMMediator();
    virtual ~WMMediator();
};

#endif /* WM_MEDIATOR_H */
