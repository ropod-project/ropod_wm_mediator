#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

#include <ropod_wm_mediator/wm_mediator.h>

WMMediator::WMMediator() : nh_("~")
{
    nh_.param<std::string>("semantic_waypoints", semantic_waypoints_file_, "config/waypoints.yaml");
    nh_.param<std::string>("locations", locations_file_, "config/locations.yaml");
    loadSemanticWaypoints();
    loadLocations();

    plan_pub_ = nh_.advertise<ropod_ros_msgs::ropod_demo_plan>("plan", 1);
    goal_sub_ = nh_.subscribe("goal", 1, &WMMediator::goalCallback, this);
}

WMMediator::~WMMediator()
{
}

void WMMediator::goalCallback(const ropod_ros_msgs::ropod_sem_waypoint_list::ConstPtr &msg) const
{
    ropod_ros_msgs::ropod_demo_plan plan_msg;
    plan_msg.header.stamp = ros::Time::now();
    // loop through each semantic location
    for (int i = 0; i < msg->sem_waypoint.size(); i++)
    {
        ropod_ros_msgs::ropod_demo_location location;
        location.command = msg->sem_waypoint[i].command;
        location.locationName = msg->sem_waypoint[i].location;
        // location.id = ?
        // location.locationStatus = ?

        if (location.command == "PAUSE" || location.command == "RESUME")
        {
            plan_msg.locations.push_back(location);
            break;
        }
        else if (location.command == "ENTER_ELEVATOR")
        {
            location.command = "TAKE_ELEVATOR";
            location.locationName = "TAKE_ELEVATOR";
        }
        else if (location.command == "EXIT_ELEVATOR")
        {
            location.command = "EXIT_ELEVATOR";
            location.locationName = "EXIT_ELEVATOR";
        }


        if (!locationExists(location.locationName))
        {
            ROS_ERROR_STREAM("Skipping location " << location.locationName << " since it does not exist");
            continue;
        }

        std::vector<MapAreas> areas = getAreasForLocation(location.locationName);
        ROS_INFO_STREAM("num areas: " << areas.size());
        // loop through all areas associated with this location
        for (int j = 0; j < areas.size(); j++)
        {
            for (MapAreas::iterator it = areas[j].begin(); it != areas[j].end(); ++it)
            {
                ropod_ros_msgs::ropod_demo_area area_msg;
                area_msg.areaName = it->first;
                // area_msg.id = ?
                // area.locationStatus = ?
                std::vector<std::string>::iterator waypoint_it = it->second.begin();
                // loop through all waypoints for this area
                for (int k = 0; k < it->second.size(); k++)
                {
                    ropod_ros_msgs::ropod_demo_waypoint waypoint_msg;
                    waypoint_msg.id = it->second[k]; // name of waypoint
                    if (waypointExists(it->second[k]))
                    {
                        waypoint_msg.waypointPosition = getWaypointPose(it->second[k]);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Skipping waypoint " << it->second[k] << " since it does not exist");
                        continue;
                    }
                    area_msg.waypoints.push_back(waypoint_msg);
                }
                location.areas.push_back(area_msg);
            }
        }
        plan_msg.locations.push_back(location);
    }
    plan_pub_.publish(plan_msg);
}

bool WMMediator::locationExists(const std::string &location) const
{
    return locations_.find(location) != locations_.end();
}

std::vector<MapAreas> WMMediator::getAreasForLocation(const std::string &location) const
{
    return locations_.at(location);
}

bool WMMediator::waypointExists(const std::string &waypoint) const
{
    return waypoints_.find(waypoint) != waypoints_.end();
}

geometry_msgs::Pose WMMediator::getWaypointPose(const std::string &waypoint) const
{
    return waypoints_.at(waypoint);
}

void WMMediator::loadSemanticWaypoints()
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(semantic_waypoints_file_);
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
    }

    for (YAML::const_iterator it=root.begin(); it != root.end(); ++it)
    {
        std::string name = it->begin()->first.as<std::string>();
        YAML::Node node = it->begin()->second;

        std::vector<double> position = node["position"].as<std::vector<double> >();
        std::vector<double> orientation = node["orientation"].as<std::vector<double> >();

        geometry_msgs::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];

        waypoints_[name] = pose;
    }
}

void WMMediator::loadLocations()
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(locations_file_);
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
    }

    // loop through locations
    for (YAML::const_iterator loc_it=root.begin(); loc_it != root.end(); ++loc_it)
    {
        std::string loc_name = loc_it->begin()->first.as<std::string>();
        YAML::Node area_node = loc_it->begin()->second;

        for (YAML::const_iterator area_it = area_node.begin(); area_it != area_node.end(); ++area_it)
        {
            std::string area_name = area_it->begin()->first.as<std::string>();
            YAML::Node waypoint_node = area_it->begin()->second;

            MapAreas map_area;
            for (YAML::const_iterator waypoint_it = waypoint_node.begin();
                 waypoint_it != waypoint_node.end();
                 ++waypoint_it)
            {
                map_area[area_name].push_back(waypoint_it->as<std::string>());
            }
            locations_[loc_name].push_back(map_area);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WM_mediator");
    ros::NodeHandle node;

    WMMediator wm_mediator;
    ros::Rate loop_rate(10);

    ROS_INFO("Ready.");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
