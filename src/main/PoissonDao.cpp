//
// Created by dejna on 06.02.17.
//

#include <iostream>
#include <boost/lexical_cast.hpp>
#include "include/PoissonDao.h"

PoissonDao::PoissonDao() {}

PoissonDao::~PoissonDao() {

}
void PoissonDao::INTS::init()
{
    PoissonDao::INTS::radius = "radius";
    PoissonDao::INTS::thread_num = "thread_num";
    PoissonDao::INTS::depth = "depth";
    PoissonDao::INTS::degree= "degree";
    PoissonDao::INTS::sample_per_node = "sample_per_node";
    PoissonDao::INTS::iso_divide = "iso_divide";
    PoissonDao::INTS::confidence = "confidence";
    PoissonDao::INTS::manifold = "manifold";
    PoissonDao::INTS::output_polygon ="output_polygon";
    PoissonDao::INTS::solver_divide ="solver_divide";
    PoissonDao::INTS::iterations ="iterations";
    PoissonDao::INTS::distance_threshold="distance_threshold";
}
void PoissonDao::DOUBLES::init()
{
    PoissonDao::DOUBLES::cloud_multipler ="cloud_multipler";
    PoissonDao::DOUBLES::scale = "scale";
}

int PoissonDao::getType(std::string value) {
    if(value.find('.') != std::string::npos){
        return 1;
    }
    return 0;
}


void PoissonDao::loadParams(std::string name, std::string value) {

    if(getType(value)){
        setDoubleAttribute(name,value);
    }else{
        setIntAttribute(name,value);
    }
}



void
PoissonDao::print() {
    std::cout << "\n Print "
              << "\n radius= " << getRadius()
              << "\n thread_num= " << getThreadNumber()
              << "\n depth= " << getDepth()
              << "\n degree= " << getDegree()
              << "\n samples_per_node= " << getSamplePerNode()
              << "\n scale= " << getScale()
              << "\n iso_divide= " << getIsoDivide()
              << "\n confidence= " << getCofidence()
              << "\n manifold= " << getManifold()
              << "\n output_polygon= " << getOutputPolygon()
              << "\n solver_divide= " << getSolverDivide()
              << "\n\n";
}


void PoissonDao::setDoubleAttribute(const std::string &name, double value) {
    this->doubleMap[name] = value;
}

void PoissonDao::setIntAttribute(const std::string &name, int value) {
    this->boolMap[name] = value;
}

int PoissonDao::getIntAttribute(const std::string &name) {
    return this->intMap[name];
}

double PoissonDao::getDoubleAttribute(const std::string &name) {
    return this->doubleMap[name];
}

