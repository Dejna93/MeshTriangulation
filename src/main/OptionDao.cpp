//
// Created by Damian on 2017-02-07.
//

#include "include/OptionDao.h"

OptionDao::OptionDao() {

}

OptionDao::~OptionDao() {}

void OptionDao::loadParams(std::string name, std::string value)
{
    setAttribute(name,value);
}



int OptionDao::getAttribute(const std::string &name) {
    if (name == "visualisation")
        return  boost::lexical_cast<int>(visualisation);
    if (name == "smoothing")
        return   boost::lexical_cast<int>(smoothing);
    if (name == "savepcd")
        return  boost::lexical_cast<int>(savepcd);
    return -1;
}

void OptionDao::setAttribute(const std::string &name, std::string value) {
    if (name == "visualisation"){
        this->visualisation =  boost::lexical_cast<int>(value);
    }
    if (name == "smoothing"){
        this->smoothing =  boost::lexical_cast<int>(value);
    }
    if (name == "savepcd"){
        this->savepcd =  boost::lexical_cast<int>(value);
    }
}