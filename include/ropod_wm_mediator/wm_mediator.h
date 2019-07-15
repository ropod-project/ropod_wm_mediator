#ifndef WM_MEDIATOR_H
#define WM_MEDIATOR_H

// C++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

#include <ropod_wm_mediator/osm.h>
#include <ftsm_base.h>

using namespace ftsm;

class WMMediator : public FTSMBase
{
private:
    ros::NodeHandle nh_;
    OSM osm_;

public:
    WMMediator();

    /* FTSM base functions */
    std::string init();
    std::string running();
    std::string recovering();

    virtual ~WMMediator();
};

#endif /* WM_MEDIATOR_H */
