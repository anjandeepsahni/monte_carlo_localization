#ifndef _SENSORMODEL_H
#define _SENSORMODEL_H

#include <iostream>
using namespace std;


class SensorModel
{
    private:
        double z_hit;           // coefficient for p_hit
        double z_short;         // coefficient for p_short
        double z_max;           // coefficient for p_max
        double z_rand;          // coefficient for p_rand
        double z_max_range;     // maximum sensor range
        double inv_var_hit;     // inverse of the variance of p_hit
        double lambda_short;    // lambda of p_short
        double k;         // 1 / sqrt(2 * pi * var)
        vector<vector<double>> occupancy_map; // occupancy map

    public:
        SensorModel(double z_h, double z_s, double z_m,
                    double z_r, double z_m_r, double v_h,
                    double l_s, vector<vector<double>> o_m);
        ~SensorModel();

        /*
         * beam_range_finder_model
         * - z_t: range measurements
         * - x_t1: predicted belief
         */
        double beam_range_finder_model(const vector<double> z_t1,
                                       const vector<double> x_t1);
        double p_hit(double z, double z_true);
        double p_short(double z, double z_true);
        double p_max(double z);
        double p_rand();
};

#endif