//
// Created by dejna on 06.02.17.
//
#ifndef MESH_TRIANGULATION_POISSONDAO_H
#define MESH_TRIANGULATION_POISSONDAO_H

#include "Dao.h"
#include <map>

class PoissonDao : protected Dao
{

public:
    PoissonDao();

    ~PoissonDao();

    void loadParams(std::string name, std::string value);

    //void loadParams(auto &section);
    void print();

    int getRadius();

    int getThreadNumber();

    int getDepth();

    int getDegree();

    int getSamplePerNode();

    int getOutputPolygon();

    int getSolverDivide();

    int getIsoDivide();

    float getScale();

    bool getCofidence();

    bool getManifold();

    //set
    void setRadius(int radius);

    void setDepth(int depth);

    void setDegree(int degree);

    void setThreadNum(int thread_num);

    void setSamplePerNode(float sample_per_node);

    void setIsoDivide(float iso_divide);

    void setOutputPolygon(int output_polygon);

    void setSolverDivide(int solver_divide);

    void setScale(float scale);

    void setConfidence(int confidence);

    void setManifold(int manifold);

    int getIterations() const;

    void setIterations(int iterations);

    int getDistance_threshold() const;

    void setDistance_threshold(int distance_threshold);

    double getCloud_multipler() const;

    void setCloud_multipler(double cloud_multipler);


private:
    int radius;
    int thread_num;
    int depth;
    int degree;
    int sample_per_node;
    float scale;
    int iso_divide;
    bool confidence;
    bool manifold;
    int output_polygon;
    int solver_divide;
    //clustering
    int iterations;
    int distance_threshold;
    double cloud_multipler;


    // CHANGE ALL TO MAP ->
    std::map<std::string, int> intMap;
    std::map<std::string, double> doubleMap;
    std::map<std::string, bool> boolMap;

    void setAttribute(const std::string &name, int value);

    void setAttribute(const std::string &name, double value);

    void setAttribute(const std::string &name, bool value);

    void setAttribute(const std::string & name, std::string value);

    int getIntAttribute(const std::string &name);

    double getDoubleAttribute(const std::string &name);

    bool getBoolAttribute(const std::string &name);

};


#endif //MESH_TRIANGULATION_POISSONDAO_H
