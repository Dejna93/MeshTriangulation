//
// Created by Damian on 2017-02-07.
//

#include <boost/lexical_cast.hpp>
#include <iostream>
#include "include/Dao.h"


void Dao::setAttribute(const std::string &name, double value) {
    this->doubleMap[name] = value;
}

void Dao::setAttribute(const std::string &name, int value) {
    this->intMap[name] = value;
}

void Dao::setAttribute(const std::string &name, std::string value) {
    this->stringMap[name] = value;
}

double Dao::getDoubleAttribute(const std::string &key) {
    return this->doubleMap[key];
}

int Dao::getIntAttribute(const std::string &key) {
    return this->intMap[key];
}


std::string Dao::getStringAttribute(const std::string &key) {
    return this->stringMap[key];
}

void Dao::loadParams(std::string name, std::string value) {
    //std::cout << getType(value) << "\n ";
    if (getType(value) == 1) {
        std::cout << "int" << value << " \n";
        setAttribute(name, boost::lexical_cast<int>(value));
    } else if (getType(value) == 0) {
        std::cout << "double" << value << " \n";
        setAttribute(name, boost::lexical_cast<double>(value));
    } else if (getType(value) == -1) {
        std::cout << "string" << value << " \n";
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


void Dao::print() {

    std::cout << "\n\nDao printed stack " << "\n";

    typedef std::map<std::string, int>::iterator int_type;
    for (int_type iterator = intMap.begin(); iterator != intMap.end(); iterator++) {
        std::cout << iterator->first << " = " << iterator->second << std::endl;
    }
    typedef std::map<std::string, double>::iterator double_type;
    for (double_type iterator = doubleMap.begin(); iterator != doubleMap.end(); iterator++) {
        std::cout << iterator->first << " = " << iterator->second << std::endl;
    }
    typedef std::map<std::string, std::string>::iterator string_type;
    for (string_type iterator = stringMap.begin(); iterator != stringMap.end(); iterator++) {
        std::cout << iterator->first << " = " << iterator->second << std::endl;
    }
    std::cout << "\n\n";
}

bool Dao::getBoolAttribute(const std::string &key) {
    return getIntAttribute(key) != 0;
}

