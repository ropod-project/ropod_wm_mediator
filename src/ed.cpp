#include <ropod_wm_mediator/ed.h>

void ED::getObjectsResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::GetObjectsResultConstPtr& result)
{
    get_objects_result_ = *result;
}

ED::ED() :
    nh_("~"), status_(false),
    get_objects_ac_("/ed/get_objects", true), get_objects_result_()
{
}

ED::~ED()
{
}

bool ED::getStatus()
{
    return status_;
}

bool ED::start()
{
    ROS_INFO_STREAM("wm_mediator waiting for get_cart_from_ed_ropod_bridge action server to come up...");
    get_objects_ac_.waitForServer();
    status_ = true;
    return true;
}

bool ED::getObjects(geometry_msgs::Polygon area, std::string type, ropod_ros_msgs::ObjectList &objects_list)
{
    ropod_ros_msgs::GetObjectsGoal req;
    req.area = area;
    req.type = type;

    get_objects_ac_.sendGoal(req, boost::bind(&ED::getObjectsResultCb, this, _1, _2));

    bool finished_before_timeout = get_objects_ac_.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
        if (get_objects_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            objects_list = get_objects_result_.objects;
            return true;
        }
        return false;
    }
    else
        return false;
}