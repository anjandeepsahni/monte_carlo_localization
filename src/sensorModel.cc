#include <iostream>
#include <cmath>
#include <random>
#include "sensorModel.hh"
using namespace std;


SensorModel::SensorModel(double z_h, double z_s, double z_m,
                         double z_r, double z_m_r, double v_h,
                         double l_s, vector<vector<double>> o_m)
{
    z_hit = z_h;
    z_short = z_s;
    z_max = z_m;
    z_rand = z_r;
    z_max_range = z_m_r;
    inv_var_hit = 1 / v_h;
    lambda_short = l_s;
    k = 1 / sqrt(2 * M_PI * v_h);
    occupancy_map = o_m;
}


SensorModel::~SensorModel()
{
}


double SensorModel::beam_range_finder_model(const vector<double> z_t1,
                                            const vector<double> x_t1)
{
    double q = 1;
    for (auto z: z_t1)
    {
        double z_true = ray_casting();
        double p, p_h, p_s, p_m, p_r;
        p_h = p_hit(z, z_true);
        p_s = p_short(z, z_true);
        p_m = p_max(z);
        p_r = p_rand();

        p = z_hit * p_h + z_short * p_s
            + z_max * p_m + z_rand * p_r;
        q *= p;
    }

    return q;
}


double SensorModel::p_hit(double z, double z_true)
{
    double p;
    if (z >= 0 && z <= z_max_range)
    {
        double temp = z - z_true, foo = sqrt(inv_var_hit) / sqrt(2);
        p = k * exp(-0.5 * temp * temp * inv_var_hit);
        // Normalization
        double cdf_0 = 0.5 * erfc(- (0 - z_true) * foo);
        double cdf_z_max = 0.5 * erfc(- (z_max_range - z_true) * foo);
        double normalizer = cdf_z_max - cdf_0;
        p /= normalizer;
    }

    else
    {
        p = 0;
    }

    return p;
}

double SensorModel::p_short(double z, double z_true)
{
    double p;
    if (z >= 0 && z <= z_true)
    {
        p = lambda_short * exp(-lambda_short * z);
        // Normalization
        double normalizer = 1 - exp(-lambda_short * z_true);
        p /= normalizer;
    }

    else
    {
        p = 0;
    }

    return p;
}


double SensorModel::p_max(double z)
{
    return (z == z_max_range) ? 1 : 0;
}


double SensorModel::p_rand()
{
    double p;
    if (z >= 0 && z <= z_max_range)
        p = 1 / z_max_range;
    else
        p = 0;

    return p;
}


double SensorModel::ray_casting(void)
{
    double z_true = 1;
    return z_true;
}
