#ifndef _SENSORMODEL_H
#define _SENSORMODEL_H

#include <iostream>
#include "particleFilter.hh"
#include "mapReader.hh"
using namespace std;


typedef struct sensor_model_parms {
    double z_hit;           // coefficient for p_hit
    double z_short;         // coefficient for p_short
    double z_max;           // coefficient for p_max
    double z_rand;          // coefficient for p_rand
    double z_max_range;     // maximum sensor range
    double z_theta_step;    // step size between sensor measurements (degrees)
    double inv_var_hit;     // inverse of the variance of p_hit
    double lambda_short;    // lambda of p_short
    double laser_offset;    // gap between robot center and laser
    double threshold;        // prob > threshold means occupied.
    map_type occupancy_map;  // occupancy map
} sm_t;



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

#endif  /* _SENSORMODEL_H */
