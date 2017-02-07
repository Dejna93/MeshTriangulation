//
// Created by dejna on 06.02.17.
//
#ifndef MESH_TRIANGULATION_POISSONDAO_H
#define MESH_TRIANGULATION_POISSONDAO_H

#include "Dao.h"

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


    void setAttribute(const std::string & name, std::string value);
    std::string getAttribute(const std::string & name);
    int getAttribute(const std::string & name);
};


#endif //MESH_TRIANGULATION_POISSONDAO_H
