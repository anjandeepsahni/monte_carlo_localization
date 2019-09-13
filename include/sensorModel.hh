#ifndef _SENSORMODEL_H
#define _SENSORMODEL_H

#include <iostream>
#include "particleFilter.hh"
using namespace std;


class SensorModel
{
private:
    sm_t sm_params;

public:
    SensorModel(sm_t sm);

    /*
     * beam_range_finder_model
     * - z_t: range measurements
     * - x_t1: predicted belief
     */
    double beam_range_finder_model(const vector<double> z_t1,
                                   const state_t x_t1);
    double p_hit(double z, double z_true);
    double p_short(double z, double z_true);
    double p_max(double z);
    double p_rand(double z);
    double ray_casting(state_t x_t1, double angle);
};

#endif
