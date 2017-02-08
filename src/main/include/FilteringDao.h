//
// Created by dejna on 07.02.17.
//

#ifndef MESH_TRIANGULATION_FILTERINGDAO_H
#define MESH_TRIANGULATION_FILTERINGDAO_H

#include <boost/lexical_cast.hpp>
#include <iostream>

#include "Dao.h"

class FilteringDao : protected Dao {

public:
    FilteringDao();

    virtual ~FilteringDao();

    void loadParams(std::string name, std::string value) override;


    int getAttribute(const std::string &name, int &attr);

    double getAttribute(const std::string &name, double &attr);

private:


    int nr_k;
    double stddev_ml;


    void setAttribute(const std::string &name, std::string value) override;
};


#endif //MESH_TRIANGULATION_FILTERINGDAO_H
