//
// Created by Damian on 2017-02-07.
//

#ifndef MY_GRAND_PROJECT_OPTIONDAO_H
#define MY_GRAND_PROJECT_OPTIONDAO_H

#include "Dao.h"
#include <boost/lexical_cast.hpp>

class OptionDao : protected Dao
{
public:
    OptionDao();
    ~OptionDao();

    //Overide
    void loadParams(std::string name, std::string value) ;

private:
    int visualisation;
    int smoothing;
    int savepcd;


    void setAttribute(const std::string & name, std::string value);
    std::string getAttribute(const std::string & name);
    int getAttribute(const std::string & name);
    float getAttribute(const std::string & name);

};
#endif //MY_GRAND_PROJECT_OPTIONDAO_H
