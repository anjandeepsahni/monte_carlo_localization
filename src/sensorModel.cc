#include <iostream>
#include "sensorModel.hh"
using namespace std;


SensorModel::SensorModel(double z_h, double z_s, double z_m, double z_r,
                         double m_r, vector<vector<double>> o_m)
{
    z_hit = z_h;
    z_short = z_s;
    z_max = z_m;
    z_rand = z_r;
    max_range = m_r;
    occupancy_map = o_m;
}


SensorModel::~SensorModel(void)
{
}


double SensorModel::beam_range_finder_model(const vector<double> z_t1,
                                            const vector<double> x_t1)
{
    double q = 1;
    for (auto it: z_t1) {

    }
}

double SensorModel::ray_casting(void)
{
    double z_true = 1;
    return z_true;
}
