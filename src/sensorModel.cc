#include <iostream>
#include <cmath>
#include <random>
#include <stdexcept>
#include "sensorModel.hh"
using namespace std;


SensorModel::SensorModel(sm_t sm_p)
{
    double sum_weights = sm_p.z_hit + sm_p.z_short + sm_p.z_max + sm_p.z_rand;
    if (abs(1 - sum_weights) >= 0.00001)
        throw runtime_error("Weights of sensor model errors must sum to 1, but got " + to_string(sum_weights));
    sm_params.z_hit = sm_p.z_hit;
    sm_params.z_short = sm_p.z_short;
    sm_params.z_max = sm_p.z_max;
    sm_params.z_rand = sm_p.z_rand;
    sm_params.z_max_range = sm_p.z_max_range;
    sm_params.z_theta_step = sm_p.z_theta_step;
    sm_params.z_dist_step = sm_p.z_dist_step;
    sm_params.p_hit_std = sm_p.p_hit_std;
    sm_params.lambda_short = sm_p.lambda_short;
    sm_params.laser_offset = sm_p.laser_offset;
    sm_params.threshold = sm_p.threshold;
    sm_params.occupancy_map = sm_p.occupancy_map;
}


double SensorModel::beam_range_finder_model(const vector<double> z_t1,
                                            const state_t x_t1)
{
    double q = 0;
    // Traversing through measurement angles
    for (int i = 0; i < z_t1.size(); i += sm_params.z_theta_step)
    {
        double z = z_t1[i];
        double p, p_h, p_s, p_m, p_r;

        // Angle wrt x axis
        double angle = ((double)i * (M_PI / 180)) + x_t1.theta;
        double z_true = ray_casting(x_t1, angle);

        p_h = p_hit(z, z_true);
        p_s = p_short(z, z_true);
        p_m = p_max(z);
        p_r = p_rand(z);

        p = sm_params.z_hit * p_h + sm_params.z_short * p_s
            + sm_params.z_max * p_m + sm_params.z_rand * p_r;
        q += (-log(1-p));   // Exponential mapping between 0 and 1.
    }

    return q;  // Log likelihood
}


double SensorModel::p_hit(double z, double z_true)
{
    double p;
    if (z >= 0 && z <= sm_params.z_max_range)
    {
        double var = pow(sm_params.p_hit_std,2);
        double k = sqrt(1 / (2 * M_PI * var));
        p = k * exp(-(((0.5 * pow(z - z_true, 2)) / var)));

        // Normalization
        double cdf_0 = 0.5 * erfc(- (0 - z_true) / sqrt(2*var));
        double cdf_z_max = 0.5 * erfc(- (sm_params.z_max_range - z_true) * sqrt(2*var));
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
    double x = x_t1.x + sm_params.laser_offset * cos(x_t1.theta);   // laser x
    double y = x_t1.y + sm_params.laser_offset * sin(x_t1.theta);   // laser y

    // Step size along the ray
    int step = sm_params.z_dist_step;

    // Move along ray and find first obstacle
    // Start ray tracing from dist=0, in case particle is at occupied location
    int obs_dist = sm_params.z_max_range;
    for (int dist=0; dist <= sm_params.z_max_range; dist=dist+step)
    {
        int x_end = (int)((x + dist * cos(angle - M_PI_2)) / map_res);
        int y_end = (int)((y + dist * sin(angle - M_PI_2)) / map_res);
        if (x_end > sm_params.occupancy_map.max_x ||
            x_end < sm_params.occupancy_map.min_x ||
            y_end > sm_params.occupancy_map.max_y ||
            y_end < sm_params.occupancy_map.min_y ||
            sm_params.occupancy_map.prob[x_end][y_end] < 0)
        {
            obs_dist = dist;
            break;
        }
        else if (sm_params.occupancy_map.prob[x_end][y_end] <= sm_params.threshold)
        {
            obs_dist = dist;
            break;
        }
    }

    return obs_dist;
}
