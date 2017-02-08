//
// Created by dejna on 06.02.17.
//

#include <iostream>
#include <boost/lexical_cast.hpp>
#include "include/PoissonDao.h"

PoissonDao::PoissonDao() {}

PoissonDao::~PoissonDao() {

}

void PoissonDao::loadParams(std::string name, std::string value) {

        //std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
    if (name == "radius")
        {
            setRadius(boost::lexical_cast<int>(value));
        }
    if (name == "thread_num")
        {
            setThreadNum(boost::lexical_cast<int>(value));
        }
    if (name == "depth")
        {
            setDepth(boost::lexical_cast<int>(value));
        }
    if (name == "degree") {
        setDegree(boost::lexical_cast<int>(value));
        }
    if (name == "samples_per_node")
        {
            setSamplePerNode(boost::lexical_cast<int>(value));
        }
    if (name == "scale")
        {
            setScale(boost::lexical_cast<float>(value));
        }
    if (name == "iso_divide") {
        setIsoDivide(boost::lexical_cast<int>(value));
        }
    if (name == "confidence") {
        setConfidence(boost::lexical_cast<int>(value));
        }
    if (name == "manifold") {
        setManifold(boost::lexical_cast<int>(value));
        }
    if (name == "output_polygon") {
        setOutputPolygon(boost::lexical_cast<int>(value));
        }
    if (name == "solver_divide") {
        setSolverDivide(boost::lexical_cast<int>(value));
    }
    // print();
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

int
PoissonDao::getRadius() {
    return this->radius;
}

int
PoissonDao::getThreadNumber() {
    return this->thread_num;
}

int
PoissonDao::getDepth() {
    return this->depth;
}
int
PoissonDao::getDegree() {
    return this->degree;
}
int
PoissonDao::getSamplePerNode() {
    return this->sample_per_node;
}

int
PoissonDao::getOutputPolygon() {
    return this->output_polygon;
}

int
PoissonDao::getSolverDivide() {
    return this->solver_divide;
}

int
PoissonDao::getIsoDivide() {
    return this->iso_divide;
}

float
PoissonDao::getScale() {
    return this->scale;
}

bool
PoissonDao::getCofidence() {
    return this->confidence;
}
bool
PoissonDao::getManifold() {
    return this->manifold;
}

//set
void
PoissonDao::setRadius(int radius) {
    this->radius = radius;
}
void
PoissonDao::setDepth(int depth) {
    this->depth = depth;
}

void
PoissonDao::setDegree(int degree) {
    this->degree = degree;
}
void
PoissonDao::setThreadNum(int thread_num) {
    this->thread_num = thread_num;
}

void
PoissonDao::setSamplePerNode(float sample_per_node) {
    this->sample_per_node = sample_per_node;
}

void
PoissonDao::setIsoDivide(float iso_divide) {
    this->iso_divide = iso_divide;
}

void
PoissonDao::setOutputPolygon(int output_polygon) {
    this->output_polygon = output_polygon;
}

void
PoissonDao::setSolverDivide(int solver_divide) {
    this->solver_divide = solver_divide;
}
void
PoissonDao::setScale(float scale) {
    this->scale = scale;
}
void
PoissonDao::setConfidence(int confidence) {
    this->confidence = confidence;
}
void
PoissonDao::setManifold(int manifold) {
    this->manifold = manifold;
}


void PoissonDao::setAttribute(const std::string &name, std::string value) {

    if(name == "radius"){
        setRadius( boost::lexical_cast<int>(value));
    }
    if (name == "thread_num"){
        setThreadNum(boost::lexical_cast<int>(value));
    }
    if (name == "depth"){
        setDepth( boost::lexical_cast<int>(value));
    }
    if (name == "degree"){
        setDegree( boost::lexical_cast<int>(value));
    }
    if (name == "sample_per_node"){
        setSamplePerNode( boost::lexical_cast<int>(value));
    }
    if ( name  == "output_polygon"){
        setOutputPolygon(boost::lexical_cast<int>(value));
    }
    if (name == "solver_divide"){
        setSolverDivide(boost::lexical_cast<int>(value));
    }
    if (name == "scale"){
        setScale(boost::lexical_cast<float>(value));
    }
    if (name == "confidence"){
        setConfidence(boost::lexical_cast<bool>(value));
    }
    if (name == "manifold"){
        setManifold(boost::lexical_cast<bool>(value));
    }
}

int PoissonDao::getIterations() const {
    return iterations;
}

void PoissonDao::setIterations(int iterations) {
    PoissonDao::iterations = iterations;
}

int PoissonDao::getDistance_threshold() const {
    return distance_threshold;
}

void PoissonDao::setDistance_threshold(int distance_threshold) {
    PoissonDao::distance_threshold = distance_threshold;
}

double PoissonDao::getCloud_multipler() const {
    return cloud_multipler;
}

void PoissonDao::setCloud_multipler(double cloud_multipler) {
    PoissonDao::cloud_multipler = cloud_multipler;
}


void PoissonDao::setAttribute(const std::string &name, int value) {
    this->intMap[name] = value;
}

void PoissonDao::setAttribute(const std::string &name, double value) {
    this->doubleMap[name] = value;
}

void PoissonDao::setAttribute(const std::string &name, bool value) {
    this->boolMap[name] = value;
}

int PoissonDao::getIntAttribute(const std::string &name) {
    return this->intMap[name];
}

double PoissonDao::getDoubleAttribute(const std::string &name) {
    return this->doubleMap[name];
}

bool PoissonDao::getBoolAttribute(const std::string &name) {
    return this->boolMap[name];
}
