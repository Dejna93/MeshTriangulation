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

    virtual void setDoubleAttribute(const std::string & name , double value) = 0;
    virtual void setIntAttribute(const std::string & name, int value) = 0;
    virtual void setBoolAttribute(const std::string & name, bool value) = 0;
    virtual void setStringAttribute(const std::string & name, std::string value);

    virtual double getDoubleAttribute(const std::string & key) =0 ;
    virtual int getIntAttribute(const std::string & key) = 0 ;
    virtual bool  getBoolAttribue(const std::string & key) = 0 ;
    virtual std::string getStringAttribute(const std::string & key) = 0;
};
#endif //MY_GRAND_PROJECT_DAO_H
