#ifndef WM_MEDIATOR_H
#define WM_MEDIATOR_H

#include <vector>
#include <map>
#include <ros/ros.h>
#include <ropod_ros_msgs/ropod_sem_waypoint_list.h>
#include <ropod_ros_msgs/ropod_demo_plan.h>

// a waypoint is identified by a (semantic) name, and is a pose on the map
typedef std::map<std::string, geometry_msgs::Pose> MapWaypoints;
// an area is identified by a name, and consists of a list of waypoints
typedef std::map<std::string, std::vector<std::string> > MapAreas;
// a location is identified by a name, and consists of a list of areas
typedef std::map<std::string, std::vector<MapAreas> > MapLocations;

// a path is identified by a name, and consists of a list of locations

class WMMediator
{
private:
    ros::NodeHandle nh_;
    ros::Publisher plan_pub_;
    ros::Subscriber goal_sub_;

    std::string semantic_waypoints_file_;
    std::string locations_file_;

    MapWaypoints waypoints_;
    MapLocations locations_;

    /**
     * callback for receiving list of semantic waypoints from com_mediator
     *
     * a ropod_demo_plan message is constructed and published (received by route navigation)
     *
     * Each corresponding semantic waypoint corresponds to a location.
     * A location consists of a set of areas, which in turn consist of a set of waypoints
     */
    void goalCallback(const ropod_ros_msgs::ropod_sem_waypoint_list::ConstPtr &msg) const;

    /**
     * returns true if location exists in world model
     */
    bool locationExists(const std::string &location) const;

    /**
     * returns set of areas corresponding to a location
     */
    std::vector<MapAreas> getAreasForLocation(const std::string &location) const;

    /**
     * returns true if mapping from waypoint name to pose exists
     */
    bool waypointExists(const std::string &waypoint) const;

    /**
     * returns pose for a given waypoint
     */
    geometry_msgs::Pose getWaypointPose(const std::string &waypoint) const;

    /**
     * load mapping from names to poses from yaml file
     */
    void loadSemanticWaypoints();

    /**
     * load mapping from locations to areas to waypoints from yaml file
     */
    void loadLocations();

public:
    WMMediator();
    virtual ~WMMediator();
};

#endif /* WM_MEDIATOR_H */
