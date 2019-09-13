#ifndef _PARTICLEFILTER_H
#define _PARTICLEFILTER_H

#include <iostream>
using namespace std;


typedef struct state {
    double x;
    double y;
    double theta;
    double weight;
} state_t;


typedef struct sensor_model_parms {
    double z_hit;           // coefficient for p_hit
    double z_short;         // coefficient for p_short
    double z_max;           // coefficient for p_max
    double z_rand;          // coefficient for p_rand
    double z_max_range;     // maximum sensor range
    int z_theta_range;      // maximum sensor rotation range (degrees)
    double inv_var_hit;     // inverse of the variance of p_hit
    double lambda_short;    // lambda of p_short
    double k;               // 1 / sqrt(2 * pi * var)
    double laser_offset;    // gap between robot center and laser
    double threshold;        // prob > threshold means occupied.
    float** occupancy_map;  // occupancy map
} sm_t;

#endif