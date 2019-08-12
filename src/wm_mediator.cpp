#include <ropod_wm_mediator/wm_mediator.h>

WMMediator::WMMediator() :
    FTSMBase("wm_mediator", {"roscore", "osm_bridge_ros_wrapper"},
             {{"functional", {{"roscore", "ros/ros_master_monitor"},
                              {"osm_bridge_ros_wrapper", "ros/ros_node_monitor"}}}}), nh_("~"),
    get_topology_node_server_(nh_,"/get_topology_node", boost::bind(&WMMediator::getTopologyNodeService, this, _1),false),
    get_shape_server_(nh_,"/get_shape", boost::bind(&WMMediator::getShapeService, this, _1),false),
    get_path_plan_server_(nh_,"/get_path_plan", boost::bind(&WMMediator::getPathPlanService, this, _1),false),
    get_nearest_wlan_server_(nh_,"/get_nearest_wlan", boost::bind(&WMMediator::getNearestWlanService, this, _1),false),
    get_elevator_waypoints_server_(nh_,"/get_elevator_waypoints", boost::bind(&WMMediator::getElevatorWaypointsService, this, _1),false),
    get_objects_server_(nh_,"/get_objects", boost::bind(&WMMediator::getObjectsService, this, _1),false)
{
}

WMMediator::~WMMediator()
{
}

/*
Given the area/sub-area id from OSM WM, it returns the position of the topology node of corresponding area/sub-area
*/
void WMMediator::getTopologyNodeService(const ropod_ros_msgs::GetTopologyNodeGoalConstPtr& goal)
{
    ropod_ros_msgs::Position topology_node;
    ropod_ros_msgs::GetTopologyNodeResult get_topology_node_result;

    if (osm_.getTopologyNode(goal->id, goal->type, topology_node))
    {
        ropod_ros_msgs::Position p;
        get_topology_node_result.position = topology_node;
        get_topology_node_server_.setSucceeded(get_topology_node_result);
    }
    else
        get_topology_node_server_.setAborted(get_topology_node_result);
}

/*
Given the area/sub-area id from OSM WM, it returns the geometry of corresponding area/sub-area
*/
void WMMediator::getShapeService(const ropod_ros_msgs::GetShapeGoalConstPtr& goal)
{
    ropod_ros_msgs::Shape shape;
    ropod_ros_msgs::GetShapeResult get_shape_result;

    if (osm_.getShape(goal->id, goal->type, shape))
    {
        get_shape_result.shape = shape;
        get_shape_server_.setSucceeded(get_shape_result);
    }
    else
        get_shape_server_.setAborted(get_shape_result);
}

/*
Given start and destination information it provides the area level path plan using OSM WM
*/
void WMMediator::getPathPlanService(const ropod_ros_msgs::GetPathPlanGoalConstPtr& goal)
{
    ropod_ros_msgs::PathPlan path_plan;
    ropod_ros_msgs::GetPathPlanResult get_path_planner_result;

    if (osm_.getPathPlan(goal, path_plan))
    {
        get_path_planner_result.path_plan = path_plan;
        get_path_plan_server_.setSucceeded(get_path_planner_result);
    }
    else
        get_path_plan_server_.setAborted(get_path_planner_result);
}

/*
Given area/sub-area/position information, it returns the nearest WiFi access points from OSM WM
*/
void WMMediator::getNearestWlanService(const ropod_ros_msgs::GetNearestWLANGoalConstPtr& goal)
{
    ropod_ros_msgs::Position wlan_pose;
    ropod_ros_msgs::GetNearestWLANResult get_neareast_wlan_result;

    if (osm_.getNearestWlan(goal, wlan_pose))
    {
        get_neareast_wlan_result.wlan_pose = wlan_pose;
        get_nearest_wlan_server_.setSucceeded(get_neareast_wlan_result);
    }
    else
        get_nearest_wlan_server_.setAborted(get_neareast_wlan_result);
}

/*
Given the elevator and door id from OSM WM, it returns the waypoints to wait inside and outside the elevator 
*/
void WMMediator::getElevatorWaypointsService(const ropod_ros_msgs::GetElevatorWaypointsGoalConstPtr& goal)
{
    ropod_ros_msgs::GetElevatorWaypointsResult get_elevator_waypoints_result_;

    geometry_msgs::Pose inside_pose, outside_pose;

    if (osm_.getElevatorWaypoints(goal->elevator_id, goal->door_id, inside_pose, outside_pose))
    {
        get_elevator_waypoints_result_.wp_inside = inside_pose;
        get_elevator_waypoints_result_.wp_outside = outside_pose;
        get_elevator_waypoints_server_.setSucceeded(get_elevator_waypoints_result_);
    }
    else
        get_elevator_waypoints_server_.setAborted(get_elevator_waypoints_result_);
}

/*
Currently just returns the dynamic objects from ed in specified OSM area 
*/
void WMMediator::getObjectsService(const ropod_ros_msgs::GetObjectsGoalConstPtr& goal)
{
    ropod_ros_msgs::GetObjectsResult get_objects_result_;
    std::vector<ropod_ros_msgs::Object> objects_list;
    geometry_msgs::Polygon area;

    if (!goal->area_id.empty())
    {
        ropod_ros_msgs::Shape area_shape;
        if (osm_.getShape(std::stoi(goal->area_id), "local_area", area_shape))
        {
            for(int  i = 0; i < area_shape.vertices.size(); i++)
            {
                geometry_msgs::Point32 p;
                p.x = area_shape.vertices[i].x; 
                p.y = area_shape.vertices[i].y;
                p.z = 0;
                area.points.push_back(p);
            }
        }
    }

    if (ed_.getObjects(area, goal->type, objects_list))
    {
        get_objects_result_.objects = objects_list;
        get_objects_server_.setSucceeded(get_objects_result_);
    }
    else
        get_objects_server_.setAborted(get_objects_result_);
}


std::string WMMediator::init()
{
    if (!osm_.start())
    {
        return FTSMTransitions::INIT_FAILED;
    }

    if (!ed_.start())
    {
        return FTSMTransitions::INIT_FAILED;
    }

    ROS_INFO_STREAM("Initialising action servers");
    get_topology_node_server_.start();
    get_shape_server_.start();
    get_path_plan_server_.start();
    get_elevator_waypoints_server_.start();
    get_nearest_wlan_server_.start();
    get_objects_server_.start();
    
    return FTSMTransitions::INITIALISED;
}

std::string WMMediator::running()
{
    return FTSMTransitions::CONTINUE;
}

std::string WMMediator::recovering()
{
    if (!ros::ok())
    {
        return FTSMTransitions::FAILED_RECOVERY;
    }
    if (osm_.getStatus() && ed_.getStatus())
    {
        return FTSMTransitions::DONE_RECOVERING;
    }
    else
    {
        ros::shutdown();
        return FTSMTransitions::FAILED_RECOVERY;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "WM_mediator");
    ros::NodeHandle node;
    WMMediator wm_mediator;
    ROS_INFO("World Model mediator ready!");

    ros::Rate loop_rate(10);
    wm_mediator.run();
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
