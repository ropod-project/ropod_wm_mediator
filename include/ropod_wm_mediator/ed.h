#ifndef ED_H
#define ED_H

// C++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Polygon.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// ROPOD
#include <ropod_ros_msgs/Object.h>
#include <ropod_ros_msgs/ObjectList.h>

#include <ropod_ros_msgs/GetObjectsAction.h> 



class ED
{
private:
    ros::NodeHandle nh_;
    bool status_;
    actionlib::SimpleActionClient<ropod_ros_msgs::GetObjectsAction> get_objects_ac_;
    ropod_ros_msgs::GetObjectsResult get_objects_result_;
    
    void getObjectsResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::GetObjectsResultConstPtr& result);

public:
    ED();
    bool start();
    bool getStatus();
    virtual ~ED();

    bool getObjects(const geometry_msgs::Polygon& area, const std::string& type, std::vector<ropod_ros_msgs::Object> &objects_list);


};

#endif /* ED_H */
