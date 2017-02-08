//
// Created by Damian on 2017-02-07.
//

#ifndef MY_GRAND_PROJECT_OPTIONDAO_H
#define MY_GRAND_PROJECT_OPTIONDAO_H

#include "Dao.h"
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <map>
class OptionDao : protected Dao
{
public:
    OptionDao();
    ~OptionDao();

    class INTS{
        public:
            static void init();

            static std::string visualisation;
            static std::string smoothing;
            static std::string savepcd;
    };


    //Overide
    void loadParams(std::string name, std::string value) ;

    void print();

    void setIntAttribute(std::string key, int value);
    int getIntAttribute(std::string key);

private:
   // int visualisation;
  //  int smoothing;
  //  int savepcd;

    std::map<std::string , int> intMap;


};
#endif //MY_GRAND_PROJECT_OPTIONDAO_H
