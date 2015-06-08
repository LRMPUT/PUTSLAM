//CPP File
#include "../include/Utilities/observer.h"
#include <algorithm>
#include <iostream>

using namespace std;

void Subject::attach(Observer *observer){
    list.push_back(observer);
}
void Subject::detach(Observer *observer){
    list.erase(std::remove(list.begin(), list.end(), observer), list.end());
}

void Subject::notify(putslam::MapModifier& mapModifier){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(mapModifier);
        }
    }
}

void Subject::notify(const cv::Mat& color, const cv::Mat& depth, int frameNo){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(color, depth, frameNo);
        }
    }
}
