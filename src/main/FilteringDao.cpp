//
// Created by dejna on 07.02.17.
//

#include "include/FilteringDao.h"

FilteringDao::FilteringDao() : nr_k(1), stddev_ml(1) {}

FilteringDao::~FilteringDao() {

}

void FilteringDao::loadParams(std::string name, std::string value) {
    setAttribute(name, value);
}

int FilteringDao::getAttribute(const std::string &name, int &attr) {
    if (name == "nr_k")
        return nr_k;
    return -1;
}

double FilteringDao::getAttribute(const std::string &name, double &attr) {
    if (name == "stddev_ml")
        return stddev_ml;
    return -1;
}


void FilteringDao::setAttribute(const std::string &name, std::string value) {
    if (name == "nr_k") {
        this->nr_k = boost::lexical_cast<int>(value);
    }
    if (name == "stddev_ml") {
        this->stddev_ml = boost::lexical_cast<double>(value);
    }
}
