#include <ropod_wm_mediator/osm.h>

void OSM::WMQueryResultCb(const actionlib::SimpleClientGoalState& state, const osm_bridge_ros_wrapper::WMQueryResultConstPtr& result)
{
    wm_query_result_ = *result;
}

osm_bridge_ros_wrapper::WMQueryResult OSM::getWMQueryResult()
{
    return wm_query_result_;
}

void OSM::pathPlannerResultCb(const actionlib::SimpleClientGoalState& state, const osm_bridge_ros_wrapper::PathPlannerResultConstPtr& result)
{
    path_planner_result_ = *result;
}

void OSM::nearestWLANResultCb(const actionlib::SimpleClientGoalState& state, const osm_bridge_ros_wrapper::NearestWLANResultConstPtr& result)
{
    nearest_wlan_result_ = *result;
}

OSM::OSM() :
    nh_("~"), status_(false),
    wm_query_ac_("/wm_query", true),
    wm_query_result_(),
    path_planner_ac_("/path_planner", true),
    path_planner_result_(),
    nearest_wlan_ac_("/nearest_wlan", true),
    nearest_wlan_result_()
{
}

OSM::~OSM()
{
}

bool OSM::getStatus()
{
    return status_;
}

bool OSM::start()
{
    ros::param::get("~building", this->building);
    if (this->building.empty())
    {
        ROS_ERROR("Please set correct building name in world model mediator launch file");
        status_ = false;
        return false;
    }
    ROS_DEBUG_STREAM("Using building: " << this->building);

    bool wm_query_server_timeout_status, path_planner_server_timeout_status, wlan_server_timeout_status;
    
    ROS_INFO_STREAM("wm_mediator waiting for wm_query action server to come up...");
    wm_query_server_timeout_status = wm_query_ac_.waitForServer(ros::Duration(10.0));
    
    ROS_INFO_STREAM("wm_mediator waiting for path_planner action server to come up...");
    path_planner_server_timeout_status = path_planner_ac_.waitForServer(ros::Duration(10.0));

    ROS_INFO_STREAM("wm_mediator waiting for nearest_wlan action server to come up...");
    wlan_server_timeout_status = nearest_wlan_ac_.waitForServer(ros::Duration(10.0));

    if(wm_query_server_timeout_status && path_planner_server_timeout_status && path_planner_server_timeout_status) 
    {
        status_ = true;
        return true;
    }
    else
    {
        ROS_ERROR("Couldn't connect one or multiple action servers provided by OSM bridge");
        ROS_ERROR("Status of action servers - WM Query: %d, Path planner: %d, WLAN: %d", wm_query_server_timeout_status, 
                   path_planner_server_timeout_status, wlan_server_timeout_status);
        return false;
    }
}

bool OSM::getPathPlan(const ropod_ros_msgs::GetPathPlanGoalConstPtr& goal, ropod_ros_msgs::PathPlan &path_plan)
{
    osm_bridge_ros_wrapper::PathPlannerGoal req;
    req.start_floor =  building + "_L" + std::to_string(goal->start_floor);
    req.destination_floor = building + "_L" + std::to_string(goal->destination_floor);

    req.start_area = goal->start_area;
    req.destination_area = goal->destination_area;
    req.start_local_area = goal->start_sub_area;
    req.destination_local_area = goal->destination_sub_area;
    req.start_position = osm_bridge_ros_wrapper::Point();
    req.start_position.x = goal->start_position.x;
    req.start_position.y = goal->start_position.y;
    req.destination_task = goal->destination_task;


    for (int i = 0; i < goal->blocked_connections.size(); i++)
    {
        osm_bridge_ros_wrapper::BlockedConnection temp;
        temp.start_id = goal->blocked_connections[i].start_id;
        temp.end_id = goal->blocked_connections[i].end_id;
        req.blocked_connections.push_back(temp);
    }
    req.relax_traffic_rules = goal->relax_traffic_rules;

    path_planner_ac_.sendGoal(req, boost::bind(&OSM::pathPlannerResultCb, this, _1, _2));
    bool finished_before_timeout = path_planner_ac_.waitForResult(ros::Duration(60.0));
    if (finished_before_timeout)
    {
        if (path_planner_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            path_plan = decodePathPlan(path_planner_result_.planner_areas);
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

bool OSM::getNearestWlan(const ropod_ros_msgs::GetNearestWLANGoalConstPtr& goal, ropod_ros_msgs::Position &wlan_pos)
{
    /* create request for osm_bridge_ros_wrapper's action server */
    osm_bridge_ros_wrapper::NearestWLANGoal req;
    req.is_x_y_provided = goal->is_pos_provided;
    if (goal->is_pos_provided)
    {
        req.x = goal->position.x;
        req.y = goal->position.y;
    }
    req.floor = goal->floor;
    req.area = goal->area;
    req.local_area = goal->local_area;

    /* send request and wait for response */
    nearest_wlan_ac_.sendGoal(req, boost::bind(&OSM::nearestWLANResultCb, this, _1, _2));
    bool finished_before_timeout = nearest_wlan_ac_.waitForResult(ros::Duration(60.0));
    ropod_ros_msgs::GetNearestWLANResult get_nearest_wlan_result;

    /* convert osm_bridge_ros_wrapper's response to wm_mediator's response */
    if (finished_before_timeout)
    {
        if (nearest_wlan_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            wlan_pos.x = nearest_wlan_result_.point.x;
            wlan_pos.y = nearest_wlan_result_.point.y;
            /* translate point to pose */
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

ropod_ros_msgs::PathPlan OSM::decodePathPlan(const std::vector<osm_bridge_ros_wrapper::PlannerArea> &planner_areas)
{
    ropod_ros_msgs::PathPlan path_plan;
    for(auto area_it = planner_areas.begin(); area_it != planner_areas.end(); area_it++)
    {
        ropod_ros_msgs::Area area_temp;
        if( area_it->area_type == "corridor")
        {
            area_temp.id = std::to_string(area_it->corridor.id);
            area_temp.name = area_it->corridor.ref;
            area_temp.floor_number = area_it->corridor.level;
        }
        else if (area_it->area_type == "room")
        {
            area_temp.id = std::to_string(area_it->room.id);
            area_temp.name = area_it->room.ref;
            area_temp.floor_number = area_it->room.level;
        }
        else if (area_it->area_type == "area")
        {
            area_temp.id = std::to_string(area_it->area.id);
            area_temp.name = area_it->area.ref;
            area_temp.floor_number = area_it->area.level;
        }
        else if (area_it->area_type == "elevator")
        {
            area_temp.id = std::to_string(area_it->elevator.id);
        }
        else if (area_it->area_type == "junction")
        {
            area_temp.id = std::to_string(area_it->corridor.id);
            area_temp.name = area_it->corridor.ref;
            area_temp.floor_number = area_it->corridor.level;
        }
        area_temp.type = area_it->area_type;

        for(auto nav_area_it = area_it->navigation_areas.begin(); nav_area_it != area_it->navigation_areas.end(); nav_area_it++)
        {
            ropod_ros_msgs::SubArea sub_area_temp;
            sub_area_temp.id = std::to_string(nav_area_it->id);
            sub_area_temp.name =  nav_area_it->ref;
            sub_area_temp.floor_number = area_temp.floor_number;
            area_temp.sub_areas.push_back(sub_area_temp);
        }

        path_plan.areas.push_back(area_temp);

        if(area_it->exit_door.id > 0)
        {
            ropod_ros_msgs::Area area_door;
            area_door.id = std::to_string(area_it->exit_door.id);
            area_door.floor_number = area_temp.floor_number;
            area_door.name = area_it->area.ref;
            area_door.type = "door";
            path_plan.areas.push_back(area_door);
        }

    }
    return path_plan;
}

bool OSM::getTopologyNode(int id, std::string type, ropod_ros_msgs::Position &position)
{
    bool finished_before_timeout;

    osm_bridge_ros_wrapper::WMQueryGoal req;
    req.id = id;
    req.type = type;

    wm_query_ac_.sendGoal(req, boost::bind(&OSM::WMQueryResultCb, this, _1, _2));
    finished_before_timeout = wm_query_ac_.waitForResult(ros::Duration(5.0));
    if (finished_before_timeout)
    {
        int topology_id = -1;
        if (wm_query_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (wm_query_result_.output == "elevator")
            {
                topology_id = wm_query_result_.elevator.topology_id;
            }
            else if (wm_query_result_.output == "door")
            {
                topology_id = wm_query_result_.door.topology_id;
            }
            else if (wm_query_result_.output == "area")
            {
                topology_id = wm_query_result_.area.topology_id;
            }
            else if (wm_query_result_.output == "corridor" || wm_query_result_.output == "junction")
            {
                topology_id = wm_query_result_.corridor.topology_id;
            }
            else if (wm_query_result_.output == "room")
            {
                topology_id = wm_query_result_.room.topology_id;
            }
            else if (wm_query_result_.output == "stairs")
            {
                topology_id = wm_query_result_.stairs.topology_id;
            }
            else if (wm_query_result_.output == "local_area")
            {
                topology_id = wm_query_result_.local_area.topology_id;
            }

            req.id = topology_id;
            req.type = "point";
            wm_query_ac_.sendGoal(req, boost::bind(&OSM::WMQueryResultCb, this, _1, _2));
            finished_before_timeout = wm_query_ac_.waitForResult(ros::Duration(5.0));

            if (finished_before_timeout)
            {
                if (wm_query_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    position.x = wm_query_result_.point.x;
                    position.y = wm_query_result_.point.y;
                    return true;
                }
            }
        }
    }
    return false;
}


bool OSM::getShape(int id, std::string type, ropod_ros_msgs::Shape &shape)
{
    osm_bridge_ros_wrapper::WMQueryGoal req;
    req.id = id;
    req.type = type;

    wm_query_ac_.sendGoal(req, boost::bind(&OSM::WMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = wm_query_ac_.waitForResult(ros::Duration(5.0));
    if (finished_before_timeout)
    {
        int shape_id = -1;
        if (wm_query_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (wm_query_result_.output == "area")
            {
                shape_id = wm_query_result_.area.shape_id;
            }
            else if (wm_query_result_.output == "corridor" || wm_query_result_.output == "junction")
            {
                shape_id = wm_query_result_.corridor.shape_id;
            }
            else if (wm_query_result_.output == "room")
            {
                shape_id = wm_query_result_.room.shape_id;
            }
            else if (wm_query_result_.output == "elevator")
            {
                shape_id = wm_query_result_.elevator.shape_id;
            }
            else if (wm_query_result_.output == "stairs")
            {
                shape_id = wm_query_result_.stairs.shape_id;
            }
            else if (wm_query_result_.output == "local_area")
            {
                shape_id = wm_query_result_.local_area.shape_id;
            }
            else if (wm_query_result_.output == "door")
            {
                shape_id = wm_query_result_.door.shape_id;
            }
            req.id = shape_id;
            req.type = "shape";
            wm_query_ac_.sendGoal(req, boost::bind(&OSM::WMQueryResultCb, this, _1, _2));
            finished_before_timeout = wm_query_ac_.waitForResult(ros::Duration(5.0));

            if (finished_before_timeout)
            {
                if (wm_query_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    for (auto it_pt = wm_query_result_.shape.points.begin(); it_pt != wm_query_result_.shape.points.end(); it_pt++)
                    {
                        ropod_ros_msgs::Position p;
                        p.x = it_pt->x;
                        p.y = it_pt->y;
                        shape.vertices.push_back(p);
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

ropod_ros_msgs::Position OSM::computeWaitingPosition(ropod_ros_msgs::Position elevator, ropod_ros_msgs::Position door,
  double distance_from_door)
{
    ropod_ros_msgs::Position waiting_position;

    double ang = atan2(elevator.y - door.y, elevator.x - door.x);
    ang = wrapToPi(ang + M_PI);

    waiting_position.x = door.x + (distance_from_door * cos(ang));
    waiting_position.y = door.y + (distance_from_door * sin(ang));

    return waiting_position;
}

double OSM::wrapToPi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle >= M_PI)
        angle -= 2 * M_PI;
    return angle;
}

geometry_msgs::Quaternion OSM::computeOrientation(ropod_ros_msgs::Position elevator, ropod_ros_msgs::Position waiting_position)
{
    double ang = atan2(elevator.y - waiting_position.y, elevator.x - waiting_position.x);
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, ang);
    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return orientation;
}

bool OSM::getElevatorWaypoints(int elevator_id, int door_id, geometry_msgs::Pose &inside_pose, geometry_msgs::Pose &outside_pose)
{
    ropod_ros_msgs::Position elevator_position, door_position;
    ropod_ros_msgs::GetElevatorWaypointsResult get_elevator_waypoints_result_;

    if (getTopologyNode(elevator_id, "elevator", elevator_position)
        && getTopologyNode(door_id, "door", door_position))
    {
        ropod_ros_msgs::Position waiting_position = computeWaitingPosition(elevator_position, door_position, 1.0);
        geometry_msgs::Quaternion orientation = computeOrientation(elevator_position, waiting_position);

        // As per the current OSM mapping conventions this waypoint will be approximately at the center of the elevator
        inside_pose.position.x = elevator_position.x;
        inside_pose.position.y = elevator_position.y;
        inside_pose.orientation = orientation;

        outside_pose.position.x = waiting_position.x;
        outside_pose.position.y = waiting_position.y;
        outside_pose.orientation = orientation;
        return true;
    }
    else
        return false;
}
