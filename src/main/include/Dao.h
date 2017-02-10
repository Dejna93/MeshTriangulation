//
// Created by Damian on 2017-02-07.
//

#ifndef MY_GRAND_PROJECT_DAO_H
#define MY_GRAND_PROJECT_DAO_H

#include <string>
#include <map>
#include <regex>
class Dao
{
public:

    void loadParams(std::string name, std::string value);

    void setAttribute(const std::string &name, double value);

    void setAttribute(const std::string &name, int value);

    void setAttribute(const std::string &name, std::string value);

    double getDoubleAttribute(const std::string &key);

    int getIntAttribute(const std::string &key);

    std::string getStringAttribute(const std::string &key);

private:
    int getType(std::string type);

    std::map<std::string, int> intMap;
    std::map<std::string, double> doubleMap;
    std::map<std::string, std::string> stringMap;
};
#endif //MY_GRAND_PROJECT_DAO_H
