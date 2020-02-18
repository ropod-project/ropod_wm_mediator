#include <ropod_wm_mediator/wm_mediator_sim.h>

bool getModeStatus(int argc, char **argv, const std::string modeName="debug_mode")
{
    bool mode = false;
    for(unsigned int i = 0; i < argc; i++)
    {
        std::string argument = std::string(argv[i]);
        std::size_t found = argument.find(modeName);
        if (found != std::string::npos)
        {
            std::string arg_val = argument.substr(argument.find("=") + 1);
            mode = (arg_val == "true");
            break;
        }
    }
    return mode;
}

WMMediatorSim::WMMediatorSim() :
    WMMediator(true)
{
}

std::string WMMediatorSim::init()
{
    if (!osm_.start())
    {
        return FTSMTransitions::INIT_FAILED;
    }

    /* Disable ED in simulation mode */
    // if (!ed_.start())
    // {
    //     return FTSMTransitions::INIT_FAILED;
    // }

    ROS_INFO_STREAM("Initialising action servers");
    get_topology_node_server_.start();
    get_shape_server_.start();
    get_path_plan_server_.start();
    get_elevator_waypoints_server_.start();
    get_nearest_wlan_server_.start();
    get_objects_server_.start();
    
    return FTSMTransitions::INITIALISED;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WM_mediator_sim");
    ros::NodeHandle node;

    bool debug = getModeStatus(argc, argv, "debug_mode");
    bool sim = getModeStatus(argc, argv, "sim_mode");

    WMMediator* wm_mediator = NULL;
    if (sim)
    {
        ROS_INFO("Creating a World model simulation object");
        wm_mediator = new WMMediatorSim();
    }
    else if (debug)
    {
        ROS_INFO("Creating a World model debug object");
        wm_mediator = new WMMediator(true);
    }
    else
    {
        ROS_INFO("Creating a World model object");
        wm_mediator = new WMMediator();
    }

    if (wm_mediator != NULL)
        ROS_INFO("World Model mediator ready!");
    else
        ROS_ERROR("Could not create a World Model mediator");

    ros::Rate loop_rate(10);
    wm_mediator->run();
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete wm_mediator;
    wm_mediator = NULL;

    return 0;
}
