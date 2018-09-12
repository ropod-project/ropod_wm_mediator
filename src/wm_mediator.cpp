#include <ropod_wm_mediator/wm_mediator.h>

void WMMediator::OSMQueryResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::OSMQueryResultConstPtr& result)
{
  osm_query_result = *result;
}

//NOTE: http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29

WMMediator::WMMediator() : nh_("~"),get_waypt_position_server(nh_,"/get_waypt_position",
  boost::bind(&WMMediator::get_waypt_position_execute, this, _1),false),ac("/osm_query", true), osm_query_result(),
  get_waypt_shape_server(nh_,"/get_waypt_shape", boost::bind(&WMMediator::get_waypt_shape_execute, this, _1),false)
{ 
    get_waypt_position_server.start();
    get_waypt_shape_server.start();
    ac.waitForServer();
}

WMMediator::~WMMediator()
{
}

void WMMediator::get_waypt_position_execute(const ropod_ros_msgs::GetWayptPositionGoalConstPtr& goal)
{
    ropod_ros_msgs::OSMQueryGoal req;
    req.ids = goal->ids;
    req.data_type = "node";
    req.query_type = "info";
    ac.sendGoal(req, boost::bind(&WMMediator::OSMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));
    ropod_ros_msgs::GetWayptPositionResult get_waypt_position_result;
    if (finished_before_timeout)
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            for (auto it_node = osm_query_result.nodes.begin(); it_node != osm_query_result.nodes.end(); it_node++)
            {
                ropod_ros_msgs::Position p;
                p.x = it_node->x;
                p.y = it_node->y;
                get_waypt_position_result.positions.push_back(p);
                get_waypt_position_result.ids.push_back(it_node->id);
            }
            get_waypt_position_server.setSucceeded(get_waypt_position_result);
        }
        else
            get_waypt_position_server.setAborted(get_waypt_position_result);
    }
    else
      get_waypt_position_server.setAborted(get_waypt_position_result);

}


void WMMediator::get_waypt_shape_execute(const ropod_ros_msgs::GetWayptShapeGoalConstPtr& goal)
{
    ropod_ros_msgs::OSMQueryGoal req;
    req.ids = goal->ids;
    req.data_type = "node";
    req.query_type = "area";
    ac.sendGoal(req, boost::bind(&WMMediator::OSMQueryResultCb, this, _1, _2));
    bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));
    ropod_ros_msgs::GetWayptShapeResult get_waypt_shape_result;
    if (finished_before_timeout)
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            for (auto it_way = osm_query_result.ways.begin(); it_way != osm_query_result.ways.end(); it_way++)
            {
                ropod_ros_msgs::Shape s;
                get_waypt_shape_result.ids.push_back(it_way->id);

                for (auto it_node = it_way->nodes.begin(); it_node != it_way->nodes.end(); it_node++)
                {
                    ropod_ros_msgs::Position p;
                    p.x = it_node->x;
                    p.y = it_node->y;
                    s.vertices.push_back(p);
                }
                get_waypt_shape_result.shapes.push_back(s);
            }
            get_waypt_shape_server.setSucceeded(get_waypt_shape_result);
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



