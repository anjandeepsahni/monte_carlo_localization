#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include "mapReader.hh"
#include "sensorModel.hh"
#include "motionModel.hh"
#include "resampler.hh"
#include "particleFilter.hh"

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
    vector<state_t> x_bar_init(num_particles);
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
        state_t meas = {x, y, theta, w};
        x_bar_init[m] = meas;
    }

    return x_bar_init;
}


// TODO: Implement this function
vector<vector<double>> init_particles_freespace(int num_particles)
{
    vector<state_t> x_bar_init(num_particles);
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

    // Get occupancy map (Anjandeep)
    MapReader map_obj = MapReader((const char *)src_path_map);
    map_type occupancy_map = map_obj.map;

    // Instantiate Motion Model, Sensor Model and Resampler
    MotionModel motion_model = MotionModel(0.1, 0.1, 0.1, 0.1);
    SensorModel sensor_model = SensorModel();
    Resampler resampler = Resampler();

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
        vector<double> z_t;
        state_t x_t0;
        state_t x_t1;
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

            // Ignoring odometry reading
            if ((time_stamp <= 0.0) || (meas_type == 'O'))
                continue;

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

            vector<state_t> x_bar_new(num_particles);
            u_t1 = odometry_robot;
            // For all particles
            for (int m = 0; m < num_particles; m++)
            {
                // Motion model
                x_t0 = x_bar[m];
                x_t1 = motion_model.update(u_t0, u_t1, x_t0);
                x_bar_new[m] = x_t1;

                // Sensor model
                if (meas_type == 'L')
                {
                    double w_t;
                    z_t = ranges;
                    w_t = sensor_model.beam_range_finder_model(z_t, x_t1);
                    w_t = 1 / num_particles;
                    x_t1.weight = w_t;
                }
                else
                {
                    x_t1.weight = x_bar[m].weight;   // Use the old weight
                }

                x_bar_new[m] = x_t1;
            }

            x_bar = x_bar_new;
            u_t0 = u_t1;

            // Resampling
            x_bar = resampler.low_variance_sampler(x_bar);

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
