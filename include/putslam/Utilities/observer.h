
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "../../include/putslam/Defs/putslam_defs.h"
#include <vector>
#include <list>

class Observer
{
public:
    virtual void update(putslam::MapModifier& mapModifier) = 0;
    virtual void update(const cv::Mat& color, const cv::Mat& depth, int frameNo) = 0;
    virtual void update(std::vector<putslam::Edge>& features) = 0;
};

class Subject
{
    //Lets keep a track of all the shops we have observing
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(putslam::MapModifier& mapModifier);
    void notify(const cv::Mat& color, const cv::Mat& depth, int frameNo);
    void notify(std::vector<putslam::Edge>& features);
};

#endif // OBSERVER_H_
