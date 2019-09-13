#include <iostream>
#include <cmath>
#include <random>
#include "sensorModel.hh"
using namespace std;


SensorModel::SensorModel(sm_t sm_p)
{
    sm_params.z_hit = sm_p.z_hit;
    sm_params.z_short = sm_p.z_short;
    sm_params.z_max = sm_p.z_max;
    sm_params.z_rand = sm_p.z_rand;
    sm_params.z_max_range = sm_p.z_max_range;
    sm_params.z_theta_step = sm_p.z_theta_step;
    sm_params.inv_var_hit = sm_p.inv_var_hit;
    sm_params.lambda_short = sm_p.lambda_short;
    sm_params.laser_offset = sm_p.laser_offset;
    sm_params.threshold = sm_p.threshold;
    sm_params.occupancy_map = sm_p.occupancy_map;
}


double SensorModel::beam_range_finder_model(const vector<double> z_t1,
                                            const state_t x_t1)
{
    double q = 1;
    for (int i = 0; i < z_t1.size(); i+=sm_params.z_theta_step)
    {
        double z = z_t1[i];
        double p, p_h, p_s, p_m, p_r;
        // Angle wrt x axis
        double angle = (double)i * M_PI / 180 + x_t1.theta - M_PI_2;
        double z_true = ray_casting(x_t1, angle);
        p_h = p_hit(z, z_true);
        p_s = p_short(z, z_true);
        p_m = p_max(z);
        p_r = p_rand(z);

        p = sm_params.z_hit * p_h + sm_params.z_short * p_s
            + sm_params.z_max * p_m + sm_params.z_rand * p_r;
        q *= p;
    }

    return q;
}


double SensorModel::p_hit(double z, double z_true)
{
    double p;
    if (z >= 0 && z <= sm_params.z_max_range)
    {
        double i_v_h = sm_params.inv_var_hit;
        double temp = z - z_true, foo = sqrt(i_v_h) / sqrt(2);
        double k = sqrt(i_v_h / (2 * M_PI));
        p = k * exp(-0.5 * temp * temp * i_v_h);
        // Normalization
        double cdf_0 = 0.5 * erfc(- (0 - z_true) * foo);
        double cdf_z_max = 0.5 * erfc(- (sm_params.z_max_range - z_true) * foo);
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
        double l_s = sm_params.lambda_short;
        p = l_s * exp(-l_s * z);
        // Normalization
        double normalizer = 1 - exp(-l_s * z_true);
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
    return (z == sm_params.z_max_range) ? 1 : 0;
}


double SensorModel::p_rand(double z)
{
    double p;
    if (z >= 0 && z <= sm_params.z_max_range)
        p = 1 / sm_params.z_max_range;
    else
        p = 0;

    return p;
}


double SensorModel::ray_casting(state_t x_t1, double angle)
{
    // Adjust for laser offset. Adjust by for map resolution.
    double map_res = sm_params.occupancy_map.resolution;
    double x = x_t1.x + (sm_params.laser_offset/map_res) * cos(x_t1.theta);
    double y = x_t1.y + (sm_params.laser_offset/map_res) * sin(x_t1.theta);
    // Step size along the ray
    int step = 1;
    // Move along ray and find first obstacle
    int obs_dist = sm_params.z_max_range;
    // Start ray tracing from dist=0, in case particle is at occupied location
    for (int dist=0; dist <= sm_params.z_max_range; dist=dist+step)
    {
        double x_end = x + dist * cos(angle - M_PI / 2) / map_res;
        double y_end = y + dist * sin(angle - M_PI / 2) / map_res;
        if (sm_params.occupancy_map.prob[(int)x_end][(int)y_end] >= sm_params.threshold)
        {
            obs_dist = dist;
            break;
        }
    }

    return obs_dist;
}
