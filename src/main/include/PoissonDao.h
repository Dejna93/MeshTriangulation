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

    class INTS{
        static void init();
        static std::string radius;
        static std::string thread_num;
        static std::string depth;
        static std::string degree;
        static std::string sample_per_node;

        static std::string iso_divide;
        static std::string confidence;
        static std::string manifold;
        static std::string output_polygon;
        static std::string solver_divide;
        //clustering
        static std::string iterations;
        static std::string distance_threshold;

    };
    class DOUBLES{
        static void init();

        static std::string cloud_multipler;
        static std::string scale;
    };

    //void loadParams(auto &section);
    void print();


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

    int getType(std::string value);

    void seIntAttribute(const std::string &name, int value);

    void setDoubleAttribute(const std::string &name, double value);

    int getIntAttribute(const std::string &name);

    double getDoubleAttribute(const std::string &name);

};


#endif //MESH_TRIANGULATION_POISSONDAO_H
