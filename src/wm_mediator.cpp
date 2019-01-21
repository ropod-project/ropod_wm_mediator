#include <ropod_wm_mediator/wm_mediator.h>

void WMMediator::WMQueryResultCb(const actionlib::SimpleClientGoalState& state, const osm_bridge_ros_wrapper::WMQueryResultConstPtr& result)
{
    wm_query_result = *result;
}

void WMMediator::PathPlannerResultCb(const actionlib::SimpleClientGoalState& state, const osm_bridge_ros_wrapper::PathPlannerResultConstPtr& result)
{
    path_planner_result = *result;
}

//NOTE: http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29

WMMediator::WMMediator() : nh_("~"),get_topology_node_server(nh_,"/get_topology_node",
  boost::bind(&WMMediator::get_topology_node_execute, this, _1),false), wm_query_ac("/wm_query", true), wm_query_result(),
  get_shape_server(nh_,"/get_shape", boost::bind(&WMMediator::get_shape_execute, this, _1),false),
  get_path_planner_server(nh_,"/get_path_plan", boost::bind(&WMMediator::get_path_plan_execute, this, _1),false),
  path_planner_ac("/path_planner", true), path_planner_result()
{ 
    get_topology_node_server.start();
    get_shape_server.start();
    get_path_planner_server.start();
    wm_query_ac.waitForServer();
    path_planner_ac.waitForServer();
    ros::param::get("/ropod_wm_mediator/building", building);
    if (building.empty())
    {
        ROS_ERROR("Please set correct building name in world model mediator launch file");
    }
}

WMMediator::~WMMediator()
{
}

void WMMediator::get_topology_node_execute(const ropod_ros_msgs::GetTopologyNodeGoalConstPtr& goal)
{
    osm_bridge_ros_wrapper::WMQueryGoal req;
    req.id = goal->id;
    req.ref = goal->ref;
    req.type = goal->type;

    wm_query_ac.sendGoal(req, boost::bind(&WMMediator::WMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = wm_query_ac.waitForResult(ros::Duration(5.0));
    ropod_ros_msgs::GetTopologyNodeResult get_topology_node_result;
    if (finished_before_timeout)
    {
        int topology_id = -1;
        if (wm_query_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (wm_query_result.output == "area")
            {
                osm_bridge_ros_wrapper::Area area = wm_query_result.area;
                topology_id = area.topology_id;
            }
            else if (wm_query_result.output == "corridor" || wm_query_result.output == "junction")
            {
                osm_bridge_ros_wrapper::Corridor corridor = wm_query_result.corridor;
                topology_id = corridor.topology_id;
            }
            else if (wm_query_result.output == "room")
            {
                osm_bridge_ros_wrapper::Room room = wm_query_result.room;
                topology_id = room.topology_id;
            }
            else if (wm_query_result.output == "elevator")
            {
                osm_bridge_ros_wrapper::Elevator elevator = wm_query_result.elevator;
                topology_id = elevator.topology_id;
            }
            else if (wm_query_result.output == "stairs")
            {
                osm_bridge_ros_wrapper::Stairs stairs = wm_query_result.stairs;
                topology_id = stairs.topology_id;
            }
            else if (wm_query_result.output == "local_area")
            { 
                osm_bridge_ros_wrapper::LocalArea local_area = wm_query_result.local_area;
                topology_id = local_area.topology_id;
            }
            else if (wm_query_result.output == "door")
            { 
                osm_bridge_ros_wrapper::Door door = wm_query_result.door;
                topology_id = door.topology_id;
            }

            req.id = topology_id;
            req.type = "point";
            wm_query_ac.sendGoal(req, boost::bind(&WMMediator::WMQueryResultCb, this, _1, _2));
            finished_before_timeout = wm_query_ac.waitForResult(ros::Duration(5.0));

            if (finished_before_timeout)
            {
                if (wm_query_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ropod_ros_msgs::Position p;
                    p.x = wm_query_result.point.x;
                    p.y = wm_query_result.point.y;

                    get_topology_node_result.position = p;

                    get_topology_node_server.setSucceeded(get_topology_node_result);
                }
                else
                    get_topology_node_server.setAborted(get_topology_node_result);
            }
            else
                get_topology_node_server.setAborted(get_topology_node_result);
        }
        else
            get_topology_node_server.setAborted(get_topology_node_result);
    }
    else
      get_topology_node_server.setAborted(get_topology_node_result);

}


void WMMediator::get_shape_execute(const ropod_ros_msgs::GetShapeGoalConstPtr& goal)
{
    osm_bridge_ros_wrapper::WMQueryGoal req;
    req.id = goal->id;
    req.ref = goal->ref;
    req.type = goal->type;

    wm_query_ac.sendGoal(req, boost::bind(&WMMediator::WMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = wm_query_ac.waitForResult(ros::Duration(5.0));
    ropod_ros_msgs::GetShapeResult get_shape_result;
    if (finished_before_timeout)
    {
        int shape_id = -1;
        if (wm_query_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {    
            if (wm_query_result.output == "area")
            {
                osm_bridge_ros_wrapper::Area area = wm_query_result.area;
                shape_id = wm_query_result.area.shape_id;
            }
            else if (wm_query_result.output == "corridor" || wm_query_result.output == "junction")
            {
                osm_bridge_ros_wrapper::Corridor corridor = wm_query_result.corridor;
                shape_id = wm_query_result.corridor.shape_id;
            }
            else if (wm_query_result.output == "room")
            {
                osm_bridge_ros_wrapper::Room room = wm_query_result.room;
                shape_id = wm_query_result.room.shape_id;
            }
            else if (wm_query_result.output == "elevator")
            {
                osm_bridge_ros_wrapper::Elevator elevator = wm_query_result.elevator;
                shape_id = wm_query_result.elevator.shape_id;
            }
            else if (wm_query_result.output == "stairs")
            {
                osm_bridge_ros_wrapper::Stairs stairs = wm_query_result.stairs;
                shape_id = wm_query_result.stairs.shape_id;
            }
            else if (wm_query_result.output == "local_area")
            { 
                osm_bridge_ros_wrapper::LocalArea local_area = wm_query_result.local_area;
                shape_id = wm_query_result.local_area.shape_id;
            }
            else if (wm_query_result.output == "door")
            { 
                osm_bridge_ros_wrapper::Door door = wm_query_result.door;
                shape_id = wm_query_result.door.shape_id;
            }
            req.id = shape_id;
            req.type = "shape";
            wm_query_ac.sendGoal(req, boost::bind(&WMMediator::WMQueryResultCb, this, _1, _2));
            finished_before_timeout = wm_query_ac.waitForResult(ros::Duration(5.0));

            if (finished_before_timeout)
            {
                if (wm_query_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ropod_ros_msgs::Shape s;
                    for (auto it_pt = wm_query_result.shape.points.begin(); it_pt != wm_query_result.shape.points.end(); it_pt++)
                    {
                        ropod_ros_msgs::Position p;
                        p.x = it_pt->x;
                        p.y = it_pt->y;
                        s.vertices.push_back(p);
                    }
                    get_shape_result.shape = s;
                    get_shape_server.setSucceeded(get_shape_result);
                }
                else
                    get_shape_server.setAborted(get_shape_result);
            }
            else
                get_shape_server.setAborted(get_shape_result);
        }
        else
            get_shape_server.setAborted(get_shape_result);
    }
    else
        get_shape_server.setAborted(get_shape_result);
}

void WMMediator::get_path_plan_execute(const ropod_ros_msgs::GetPathPlanGoalConstPtr& goal)
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

    path_planner_ac.sendGoal(req, boost::bind(&WMMediator::PathPlannerResultCb, this, _1, _2));
    bool finished_before_timeout = path_planner_ac.waitForResult(ros::Duration(60.0));
    ropod_ros_msgs::GetPathPlanResult get_path_planner_result;
    if (finished_before_timeout)
    {   
        if (path_planner_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {    
            get_path_planner_result.path_plan =  decode_path_plan(path_planner_result.planner_areas);
            get_path_planner_server.setSucceeded(get_path_planner_result);
        }
        else
            get_path_planner_server.setAborted(get_path_planner_result);
    }
    else
        get_path_planner_server.setAborted(get_path_planner_result);
}

ropod_ros_msgs::PathPlan WMMediator::decode_path_plan(std::vector<osm_bridge_ros_wrapper::PlannerArea> planner_areas)
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

        if(area_it->exit_door.id > 0)
        {
            ropod_ros_msgs::SubArea sub_area_temp;
            sub_area_temp.id = std::to_string(area_it->exit_door.id);
            area_temp.sub_areas.push_back(sub_area_temp);
            sub_area_temp.floor_number = area_temp.floor_number;
        }
        path_plan.areas.push_back(area_temp);
    }
    return path_plan;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "WM_mediator");
    ros::NodeHandle node;
    WMMediator wm_mediator;

    ROS_INFO("World Model mediator ready!");

    ros::spin();
    return 0;
}



