//
// Created by Damian on 2017-02-07.
//

#include <boost/lexical_cast.hpp>
#include <iostream>
#include "include/Dao.h"


void Dao::setAttribute(const std::string &name, double value) {
    this->doubleMap[name] = value;
}

double Dao::getDoubleAttribute(const std::string &key) {
    return this->doubleMap[key];
}

int Dao::getIntAttribute(const std::string &key) {
    return this->intMap[key];
}

void Dao::setAttribute(const std::string &name, int value) {
    this->intMap[name] = value;
}

void Dao::loadParams(std::string name, std::string value) {
    std::cout << getType(value) << "\n ";
    if (getType(name) == 1) {
        setAttribute(name, boost::lexical_cast<int>(value));
    } else if (getType(name) == 0) {
        setAttribute(name, boost::lexical_cast<double>(value));
    } else if (getType(name) == -1) {
        setAttribute(name, value);
    }
}

int Dao::getType(std::string type) {
    //[0-9]*\.?[0-9]*
    //std::cout << "input " << type <<std::endl;
    std::regex number("(\\+|-)?[[:digit:]]+");
    std::smatch output;

    std::regex_search(type, output, number);
    // std::cout << "gettype "<<output.str() <<std::endl;
    if (output.str() != "") {
        //double
        if (type.find('.') != std::string::npos) {
            return 0;
        }
        //int
        return 1;
    }
    return -1;

}

void Dao::setAttribute(const std::string &name, std::string value) {
    this->stringMap[name] = value;
}

std::string Dao::getStringAttribute(const std::string &key) {
    return this->stringMap.at(key);
}

