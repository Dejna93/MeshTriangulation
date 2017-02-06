//
// Created by dejna on 06.02.17.
//

#include "include/PoissonDao.h"


PoissonDao::PoissonDao() {

}

PoissonDao::~PoissonDao() {

}

void PoissonDao::loadParams(auto &section) {
    for (auto &key : section.second) {
        std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
        if (key.first == "radius") {
            setRadius(key.second.get_value<int>());
        }
        if (key.first == "thread_num") {
            setThreadNum(key.second.get_value<int>());
        }
        if (key.first == "depth") {
            setDepth(key.second.get_value<int>());
        }
        if (key.first == "degree") {
            setDegree(key.second.get_value<int>());
        }
        if (key.first == "samples_per_node") {
            setSamplePerNode(key.second.get_value<int>());
        }
        if (key.first == "scale") {
            setScale(key.second.get_value<float>());
        }
        if (key.first == "iso_divide") {
            setIsoDivide(key.second.get_value<float>());
        }
        if (key.first == "confidence") {
            setConfidence(key.second.get_value<int>());
        }
        if (key.first == "manifold") {
            setManifold(key.second.get_value<int>());
        }
        if (key.first == "output_polygon") {
            setOutputPolygon(key.second.get_value<int>());
        }
        if (key.first == "solver_divide") {
            setSolverDivide(key.second.get_value<int>());
        }
    }
    print();
}

void
PoissonDao::print() {
    cout << "\n Print "
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