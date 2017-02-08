//
// Created by Damian on 2017-02-07.
//

#include "include/OptionDao.h"

OptionDao::OptionDao() {
}

void OptionDao::INTS::init() {
    OptionDao::INTS::visualisation = "visualisation";
    OptionDao::INTS::smoothing = "smoothing";
    OptionDao::INTS::savepcd = "savepcd";
}

OptionDao::~OptionDao() {}

void OptionDao::loadParams(std::string name, std::string value)
{
    setAttribute(name,value);
}


void
OptionDao::print() {
    std::cout << "\n visualisation=" << getIntAttribute(OptionDao::INTS::visualisation)
              << "\n smoothing=" << getIntAttribute(OptionDao::INTS::smoothing)
              << "\n savepcd=" << getIntAttribute(OptionDao::INTS::savepcd)
              << "\n\n";
}



void OptionDao::setIntAttribute(std::string key, int value)
{
    this->intMap.insert(std::make_pair(key,value));
}
int OptionDao::getIntAttribute(std::string key) {
    return this->intMap.at(key);
}