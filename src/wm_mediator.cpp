#include <ropod_wm_mediator/wm_mediator.h>

void WMMediator::WMQueryResultCb(const actionlib::SimpleClientGoalState& state, const osm_bridge_ros_wrapper::WMQueryResultConstPtr& result)
{
    wm_query_result = *result;
}

//NOTE: http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29

WMMediator::WMMediator() : nh_("~"),get_waypt_position_server(nh_,"/get_position",
  boost::bind(&WMMediator::get_waypt_position_execute, this, _1),false),wm_query_ac("/wm_query", true), wm_query_result(),
  get_waypt_shape_server(nh_,"/get_shape", boost::bind(&WMMediator::get_waypt_shape_execute, this, _1),false)
{ 
    get_waypt_position_server.start();
    get_waypt_shape_server.start();
    wm_query_ac.waitForServer();
}

WMMediator::~WMMediator()
{
}

void WMMediator::get_waypt_position_execute(const ropod_ros_msgs::GetWayptPositionGoalConstPtr& goal)
{
    osm_bridge_ros_wrapper::WMQueryGoal req;
    req.id = goal->id;
    req.ref = goal->ref;
    req.type = goal->type;

    wm_query_ac.sendGoal(req, boost::bind(&WMMediator::WMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = wm_query_ac.waitForResult(ros::Duration(5.0));
    ropod_ros_msgs::GetWayptPositionResult get_waypt_position_result;
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

                    get_waypt_position_result.position = p;

                    get_waypt_position_server.setSucceeded(get_waypt_position_result);
                }
                else
                    get_waypt_position_server.setAborted(get_waypt_position_result);
            }
            else
                get_waypt_position_server.setAborted(get_waypt_position_result);
        }
        else
            get_waypt_position_server.setAborted(get_waypt_position_result);
    }
    else
      get_waypt_position_server.setAborted(get_waypt_position_result);

}


void WMMediator::get_waypt_shape_execute(const ropod_ros_msgs::GetWayptShapeGoalConstPtr& goal)
{
    osm_bridge_ros_wrapper::WMQueryGoal req;
    req.id = goal->id;
    req.ref = goal->ref;
    req.type = goal->type;

    wm_query_ac.sendGoal(req, boost::bind(&WMMediator::WMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = wm_query_ac.waitForResult(ros::Duration(5.0));
    ropod_ros_msgs::GetWayptShapeResult get_waypt_shape_result;
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
                    get_waypt_shape_result.shape = s;
                    get_waypt_shape_server.setSucceeded(get_waypt_shape_result);
                }
                else
                    get_waypt_shape_server.setAborted(get_waypt_shape_result);
            }
            else
                get_waypt_shape_server.setAborted(get_waypt_shape_result);
        }
        else
            get_waypt_shape_server.setAborted(get_waypt_shape_result);
    }
    else
        get_waypt_shape_server.setAborted(get_waypt_shape_result);
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



