#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>

#include "mapreader.hh"
using namespace std;


// TODO: Implement this function
void visualize_map()
{

}


// TODO: Implement this function
void visualize_timestep()
{

}


vector<vector<double>> init_particles_random(int num_particles)
{
    vector<vector<double>> x_bar_init(num_particles, vector<double>(4));
    default_random_engine generator;
    uniform_real_distribution<double> dist_x(3000, 7000);
    uniform_real_distribution<double> dist_y(0, 7000);
    uniform_real_distribution<double> dist_theta(-3.14, 3.14);
    for (int m = 0; m < num_particles; m++)
    {
        double x = dist_x(generator);
        double y = dist_y(generator);
        double theta = dist_theta(generator);
        double w = 1 / num_particles;
        vector<double> meas = {x, y, theta, w};
        x_bar_init[m] = meas;
    }

    return x_bar_init;
}


// TODO: Implement this function
vector<vector<double>> init_particles_freespace(int num_particles)
{
    vector<vector<double>> x_bar_init(num_particles, vector<double>(4));
    return x_bar_init;
}


int main(int argc, const char * argv[])
{
    /*
     * Description of variables used:
     * u_t0: particle state odometry reading [x, y, theta] at time (t-1)
     *      [odometry_frame]   
     * u_t1: particle state odometry reading [x, y, theta] at time t
     *      [odometry_frame]
     * x_t0: particle state belief [x, y, theta] at time (t-1)
     *      [world_frame]
     * x_t1: particle state belief [x, y, theta] at time t
     *      [world_frame]
     * x_bar: [num_particles x 4] sized array containing [x, y, theta, wt]
     *      values for all particles
     * z_t: array of 180 range measurements for each laser scan
     */

    /*
     * Initialize Parameters
     */
    
    string src_path_map = "../data/map/wean.dat";
    string src_path_log = "../data/log/robotdata1.log";
    mapreader();
    // Get occupancy map (Anjandeep)

    // Instantiate Motion Model, Sensor Model and Resampler

    bool vis_flag = true;
    int num_particles = 500;
    vector<vector<double>> x_bar;
    x_bar = init_particles_random(num_particles);

    /*
     * Monte Carlo Localization Algorithm
     */

    if (vis_flag)
    {
        // Call visualize_map
    }

    ifstream log_file (src_path_log);   // Read the log file
    if (log_file.is_open())
    {
        vector<double> u_t0;
        vector<double> u_t1;
        vector<double> x_t0;
        vector<double> x_t1;
        vector<double> z_t;
        string line;
        int time_idx = 0;
        while (getline(log_file, line))
        {
            time_idx++;
            vector<double> odometry_robot;  // Odometry reading: [x, y, theta]
            vector<double> odometry_laser;  // Laser coordinates in O frame
            vector<double> meas_vals;       // numerical values
            vector<double> ranges;          // 180 range measurements
            char meas_type = line[0];       // L: laser; O: odometry
            string meas = line.substr(2);   // slice first two chars
            stringstream stream(meas);
            while (1)
            {
                double n;
                stream >> n;
                if (!stream)
                    break;
                meas_vals.push_back(n);
            }

            for (int i = 0; i < 3; i++)
                odometry_robot.push_back(meas_vals[i]);
            double time_stamp = meas_vals[meas_vals.size() - 1];

            // if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure
            // odometry measurements for now (faster debugging) continue

            if (meas_type == 'L')
            {
                for (int i = 3; i < 6; i++)
                    odometry_laser.push_back(meas_vals[i]);
                for (int i = 6; i < meas_vals.size() - 1; i++)
                    ranges.push_back(meas_vals[i]);
            }

            cout << "Processing time step " << time_idx + 1;
            cout << " at time " << time_stamp;
            cout << endl;

            if (time_idx == 1)
            {
                u_t0 = odometry_robot;
                continue;
            }

            vector<vector<double>> x_bar_new(num_particles, vector<double>(4));
            u_t1 = odometry_robot;
            // For all particles
            for (int m = 0; m < num_particles; m++)
            {
                // MOTION MODEL (Sanjana)
                x_t0 = vector<double>(x_bar[m].begin(), x_bar[m].begin() + 3);
                // TODO: Update x_t1 by calling the motion model update function
                // x_t1 = motion_model.update(u_t0, u_t1, x_t0)

                // SENSOR MODEL (Dhananjai)
                vector<double> x_t1_copy(x_t1.begin(), x_t1.end());
                if (meas_type == 'L')
                {
                    z_t = ranges;
                    double w_t;
                    // TODO: Weights using the sensor model
                    // w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
                    // w_t = 1 / num_particles
                    x_t1_copy.push_back(w_t);
                }

                else
                {
                    x_t1_copy.push_back(x_bar[m][3]);   // Use the old weight
                }

                x_bar_new[m] = x_t1_copy;
            }

            x_bar = x_bar_new;
            u_t0 = u_t1;

            // RESAMPLING
            // X_bar = resampler.low_variance_sampler(X_bar)

            if (vis_flag)
            {
                // Call visualize_timestep
            }
        }

        log_file.close();
    }

    else
    {
        cout << "Log file " << src_path_log << " could not be opened.";
        cout << endl;
    }

    return 0;
}
