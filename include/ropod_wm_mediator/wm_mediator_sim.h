#ifndef WM_MEDIATOR_SIM_H
#define WM_MEDIATOR_SIM_H

#include "wm_mediator.h"

class WMMediatorSim : public WMMediator
{
public:
    WMMediatorSim();

    virtual std::string init();

    virtual ~WMMediatorSim(){}
};

#endif /* WM_MEDIATOR_SIM_H */
