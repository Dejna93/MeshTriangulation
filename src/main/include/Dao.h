//
// Created by Damian on 2017-02-07.
//

#ifndef MY_GRAND_PROJECT_DAO_H
#define MY_GRAND_PROJECT_DAO_H

#include <string>

class Dao
{
public:
    virtual void loadParams(std::string name, std::string value) = 0;

protected:
    template<typename T>
    T getAttribute(const std::string &name);

    virtual void setAttribute(const std::string & name, std::string value) =0;
};
#endif //MY_GRAND_PROJECT_DAO_H
