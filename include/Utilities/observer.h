
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "../include/Defs/putslam_defs.h"
#include <vector>
#include <list>

class Observer
{
public:
    virtual void update(putslam::MapModifier& mapModifier) = 0;
};

class Subject
{
    //Lets keep a track of all the shops we have observing
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(putslam::MapModifier& mapModifier);
};

#endif // OBSERVER_H_
