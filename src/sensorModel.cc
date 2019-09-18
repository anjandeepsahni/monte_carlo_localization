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
        double angle = ((double)i * (M_PI / 180)) + x_t1.theta - M_PI_2;
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
    double x = x_t1.x + sm_params.laser_offset * cos(x_t1.theta);
    double y = x_t1.y + sm_params.laser_offset * sin(x_t1.theta);
    // Step size along the ray
    int step = sm_params.z_dist_step;
    // Move along ray and find first obstacle
    int obs_dist = sm_params.z_max_range;
    // Start ray tracing from dist=0, in case particle is at occupied location
    for (int dist=0; dist <= sm_params.z_max_range; dist=dist+step)
    {
        double x_end = (x + dist * cos(angle)) / map_res;
        double y_end = (y + dist * sin(angle)) / map_res;
#ifdef FLIP_Y_AXIS
        // Account for flipped y axis.
        y_end = sm_params.occupancy_map.size_y - y_end;
#endif
        if (x_end > sm_params.occupancy_map.max_x or
            x_end < sm_params.occupancy_map.min_x or
            y_end > sm_params.occupancy_map.max_y or
            y_end < sm_params.occupancy_map.min_y)
        {
            obs_dist = dist;
            break;
        }
        else if (sm_params.occupancy_map.prob[(int)x_end][(int)y_end] <= sm_params.threshold)
        {
            obs_dist = dist;
            break;
        }
    }

    return obs_dist;
}


vector<double> SensorModel::get_intrinsic_parameters(vector<vector<double>> Z,
                                                     vector<state_t> X)
{
    if (Z.size() != X.size())
        throw runtime_error("Number of measurements and poses must be equal");

    double z_h = 0, z_s = 0, z_m = 0, z_r = 0;
    double std_h = sm_params.p_hit_std; // Initialize properly
    double lambda_s = sm_params.lambda_short;
    double num_std_h = 0;       // Running sum of numerator of std formula
    double num_lambda_s = 0;    // Running sum of denominator of lambda formula
    double p_ml = 1;            // Probability of Maximum Likelihood
    double threshold = 0.8;     // Threshold for Maximum Likelihood
    bool launcher = true;       // Initiate the convergence loop below

    while (launcher || p_ml < threshold)
    {
        if (launcher)
            launcher = false;
        p_ml = 1;       // Compute a new p_ml
        double e_h = 0;
        double e_s = 0;
        double e_m = 0;
        double e_r = 0;

        // For all measurements z in Z
        for (int i = 0; i < Z.size(); i++)
        {
            double p_z = 1;
            double p_z_h = 1;
            double p_z_s = 1;
            double p_z_m = 1;
            double p_z_r = 1;
            double diff_z_h = 0;    // Running sum of (z_k - z_k_true)^2
            double z_magnitude = 0; // Magnitude of z
            int num_angles = 0;     // Number of angles considered

            // For all measurement angles
            for (int j = 0; j < Z[i].size(); j += sm_params.z_theta_step)
            {
                double z = Z[i][j]; // Measurement at a certain angle
                z_magnitude += z * z;
                double p_h=1, p_s=1, p_m=1, p_r=1;
                // Angle wrt x axis
                double angle = (double)i * M_PI / 180 + X[i].theta - M_PI_2;
                double z_true = ray_casting(X[i], angle);
                diff_z_h += (z - z_true) * (z - z_true);
                p_h *= p_hit(z, z_true);
                p_s *= p_short(z, z_true);
                p_m *= p_max(z);
                p_r *= p_rand(z);
                // Update the probabilities for each angle to get p_z
                p_z *= (z_h * p_h + z_s * p_s + z_m * p_m + z_r * p_r);
                // Update the individial sensor noise probabilities
                p_z_h *= p_h;
                p_z_s *= p_s;
                p_z_m *= p_m;
                p_z_r *= p_r;

                num_angles++;
            }

            double normalizer = p_z_h + p_z_s + p_z_m + p_z_r;
            double e_z_h = p_z_h / normalizer;
            double e_z_s = p_z_s / normalizer;
            double e_z_m = p_z_m / normalizer;
            double e_z_r = p_z_r / normalizer;
            // TODO: Verify the two lines below. Confirm the normalization.
            num_std_h += e_z_h * (diff_z_h / num_angles);
            num_lambda_s += e_z_s * (sqrt(z_magnitude) / num_angles);
            e_h += e_z_h;
            e_s += e_z_s;
            e_m += e_z_m;
            e_r += e_z_r;
            // Update p_ml
            p_ml *= p_z;
        }

        z_h = e_h / Z.size();
        z_s = e_s / Z.size();
        z_m = e_m / Z.size();
        z_r = e_r / Z.size();
        std_h = sqrt(num_std_h / e_h);
        lambda_s = e_s / num_lambda_s;
        // Update sensor model parameters
        sm_params.p_hit_std = std_h;
        sm_params.lambda_short = lambda_s;
    }

    // Either return the paramaters or update directly in the sensor model
    vector<double> parameters = {z_h, z_s, z_m, z_r, std_h, lambda_s};
    return parameters;
}
