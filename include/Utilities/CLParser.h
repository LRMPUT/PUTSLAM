//source: http://www.cplusplus.com/forum/beginner/26251/
//@author: m4ster r0shi

#ifndef _CLPARSER_H_
#define _CLPARSER_H_

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>

class CLParser
{
public:

    CLParser(int argc_, char * argv_[],bool switches_on_=false);
    ~CLParser(){}

    std::string get_arg(int i);
    std::string get_arg(std::string s);

private:

    int argc;
    std::vector<std::string> argv;

    bool switches_on;
    std::map<std::string,std::string> switch_map;
};

#endif // _CLPARSER_H_
