#include <ropod_wm_mediator/wm_mediator.h>


WMMediator::WMMediator() :
    FTSMBase("wm_mediator", {"roscore", "osm_bridge_ros_wrapper"},
             {{"functional", {{"roscore", "ros/ros_master_monitor"},
                              {"osm_bridge_ros_wrapper", "ros/ros_node_monitor"}}}}), nh_("~")
{
}

WMMediator::~WMMediator()
{
}

std::string WMMediator::init()
{
    ROS_INFO_STREAM("Initialising action servers");

    OSM osm;
    

    // ros::param::get("~building", this->building);
    // if (this->building.empty())
    // {
    //     ROS_ERROR("Please set correct building name in world model mediator launch file");
    //     return FTSMTransitions::INIT_FAILED;
    // }

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
    // if (this->building.empty()){
    //     ros::shutdown();
    //     return FTSMTransitions::FAILED_RECOVERY;
    // }
    return FTSMTransitions::DONE_RECOVERING;
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
