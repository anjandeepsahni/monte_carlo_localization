#include <iostream>
#include <cmath>
#include <random>
#include "sensorModel.hh"
using namespace std;


SensorModel::SensorModel(double z_h, double z_s, double z_m,
                         double z_r, double z_m_r, int z_t_r, double v_h,
                         double l_s, double l_o, float th, float** o_m)
{
    z_hit = z_h;
    z_short = z_s;
    z_max = z_m;
    z_rand = z_r;
    z_max_range = z_m_r;
    z_theta_range = z_t_r;
    inv_var_hit = 1 / v_h;
    lambda_short = l_s;
    k = 1 / sqrt(2 * M_PI * v_h);
    laser_offset = l_o;
    threshold = th;
    occupancy_map = o_m;
}


double SensorModel::beam_range_finder_model(vector<double> z_t1,
                                            vector<double> x_t1)
{
    double q = 1;
    for (int i=0; i<z_t1.size(); ++i)
    {
        double z = z_t1[i];
        double p, p_h, p_s, p_m, p_r;
        // angle wrt x axis
        float angle = (float)i * M_PI / 180 + x_t1[2] - M_PI_2;  // FIXME: Assumes entire 180 degree coverage.
        double z_true = ray_casting(x_t1, angle);
        p_h = p_hit(z, z_true);
        p_s = p_short(z, z_true);
        p_m = p_max(z);
        p_r = p_rand(z);

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


double SensorModel::p_rand(double z)
{
    double p;
    if (z >= 0 && z <= z_max_range)
        p = 1 / z_max_range;
    else
        p = 0;

    return p;
}


double SensorModel::ray_casting(vector<double> x_t1, float angle)
{
    // Adjust for laser offset. Adjust by 10cm for map resolution.
    // FIXME: Remove hard-coding.
    float x = x_t1[0] + (laser_offset/10.0) * cos(x_t1[2]);
    float y = x_t1[1] + (laser_offset/10.0) * sin(x_t1[2]);
    // Step size along the ray
    int step = 1;
    // Move along ray and find first obstacle
    int obs_dist = z_max_range;
    for (int dist=0; dist<=z_max_range; dist=dist+step) // FIXME: should we start distance from 1?
    {
        float x_end = x + dist * cos(angle - M_PI / 2) / 10.0;
        float y_end = y + dist * sin(angle - M_PI / 2) / 10.0;
        if (occupancy_map[(int)x_end][(int)y_end] >= threshold)
        {
            obs_dist = dist;
            break;
        }
    }

    return obs_dist;
}
