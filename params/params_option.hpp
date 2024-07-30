#pragma once
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "parameter_reader.h"

struct NavControlParams {
    int n;
    int m;
    int Nc;
    double l;
    double T;
    int Np;
    double qWeightStateOffset;
    double rWeightControlIncrement;
    double epsilonRelaxationFactor;
    double rhoWeightCoefficient;
    double vMax;
    double deltaMax;
    double deltaDeltaMax;
};

struct NavRoadParams {
    double q;
    double r;
    double aymin;
    double aymax;
    int Nc;
    int Np;
    double T;
    double zeta;
    double Sob;
    int num;
};

static NavControlParams CreateControlTaskDecOptions(const std::string& file_name)
{
    std::string task_option_name = file_name + "/task_control_params.txt";
    std::cout << "read task_control_params   " << task_option_name << std::endl;
    rac::ParameterReader paramReader(task_option_name);
    NavControlParams navControlParams;
    paramReader.param("n", navControlParams.n, 1);
    paramReader.param("m", navControlParams.m, 2);
    paramReader.param("Nc", navControlParams.Nc, 1);
    paramReader.param("l", navControlParams.l, 1.2);
    paramReader.param("T", navControlParams.T, 0.05);
    paramReader.param("Np", navControlParams.Np, 10);
    paramReader.param("qWeightStateOffset", navControlParams.qWeightStateOffset, 1.0);
    paramReader.param("rWeightControlIncrement", navControlParams.rWeightControlIncrement, 1.0);
    paramReader.param("epsilonRelaxationFactor", navControlParams.epsilonRelaxationFactor, 1.0);
    paramReader.param("rhoWeightCoefficient", navControlParams.rhoWeightCoefficient, 100.0);
    paramReader.param("vMax", navControlParams.vMax, 1.0);
    paramReader.param("deltaMax", navControlParams.deltaMax, 0.4712);//0.0873,0.4712,0.6109
    paramReader.param("deltaDeltaMax", navControlParams.deltaDeltaMax, 1.5);
    return navControlParams;
}

static NavRoadParams CreateRoadTaskDecOptions(const std::string& file_name)
{
    std::string task_option_name = file_name + "/task_road_params.txt";
    std::cout << "read task_road_params   " << task_option_name << std::endl;
    rac::ParameterReader paramReader(task_option_name);
    NavRoadParams navRoadParams;
    paramReader.param("q", navRoadParams.q, 200.0);
    paramReader.param("r", navRoadParams.r, 200.0);
    paramReader.param("aymin", navRoadParams.aymin, -1.0);
    paramReader.param("aymax", navRoadParams.aymax, 1.0);
    paramReader.param("Nc", navRoadParams.Nc, 5);
    paramReader.param("Np", navRoadParams.Np, 10);
    paramReader.param("T", navRoadParams.T, 0.2);
    paramReader.param("zeta", navRoadParams.zeta, 0.1);
    paramReader.param("Sob", navRoadParams.Sob, 1000.0);
    paramReader.param("num", navRoadParams.num, 50);
    return navRoadParams;
}
