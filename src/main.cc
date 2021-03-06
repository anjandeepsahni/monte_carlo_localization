#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include "mapReader.hh"
#include "sensorModel.hh"
#include "motionModel.hh"
#include "resampler.hh"
#include "particleFilter.hh"
#include "config.hh"

using namespace std;


vector<state_t> init_particles(int num_particles, map_type map, bool freeSpace=false)
{
    vector<state_t> x_bar_init(num_particles);

    double res = map.resolution;
    double start_x = map.min_x * res;
    double end_x = map.max_x * res;
    double start_y = map.max_y * res;
    double end_y = map.min_y * res;

    uniform_real_distribution<double> dist_x(start_x, end_x);
    uniform_real_distribution<double> dist_y(start_y, end_y);
    uniform_real_distribution<double> dist_theta(-3.14, 3.14);
    double w = (double)(1.0 / (double)num_particles);
    mt19937 gen(rand());

    for (int m = 0; m < num_particles; m++)
    {
        double x, y, theta;
        // If freeSpace = true, keep looping until we find a particle in free space.
        do
        {
            x = dist_x(gen);
            y = dist_y(gen);
            theta = dist_theta(gen);
#ifdef FLIP_Y_AXIS
        } while (freeSpace and (map.prob[(int)(x/res)][map.size_y - (int)(y/res)] == -1 ||
                 map.prob[(int)(x/res)][map.size_y - (int)(y/res)] <= FREE_SPACE_THRESH));
#else
    } while (freeSpace and (map.prob[(int)(x/res)][(int)(y/res)] == -1 ||
                            map.prob[(int)(x/res)][(int)(y/res)] <= FREE_SPACE_THRESH));
#endif

        state_t meas = {x, y, theta, w};
        x_bar_init[m] = meas;
    }

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

    string src_path_map = MAP_FILE_PATH;
    string src_path_log = LOG_FILE_PATH;
    // Get occupancy map
    MapReader map_obj = MapReader(src_path_map);
    if (map_obj.read_map() < 0)
        exit(-1);
    map_type occupancy_map = map_obj.map;

    // Instantiate Motion Model, Sensor Model and Resampler
    MotionModel motion_model = MotionModel(ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4);
    sm_t sm_init = {
        Z_HIT,
        Z_SHORT,
        Z_MAX,
        Z_RAND,
        LASER_MAX_RANGE,
        LASER_THETA_STEP,
        LASER_DIST_STEP,
        P_HIT_STD,
        LAMBDA_SHORT,
        LASER_OFFSET,
        PARTICLE_THRESH,
        occupancy_map
    };
    SensorModel sensor_model = SensorModel(sm_init);
    Resampler resampler = Resampler();

    bool vis_flag = true;
    int num_particles = NUM_PARTICLES;
    vector<state_t> x_bar;
    x_bar = init_particles(num_particles, occupancy_map, true);

    /*
     * Monte Carlo Localization Algorithm
     */

#ifdef MAP_VISUALIZE
    if (vis_flag)
    {
        // Visualize initial particles.
//        map_obj.visualize_map(x_bar);

        // Visualize ray casting.
//        map_obj.visualize_map(x_bar, false, true, &sensor_model);
    }
#endif

    ifstream log_file (src_path_log);   // Read the log file
    if (log_file.is_open())
    {
        vector<double> u_t0;
        vector<double> u_t1;
        vector<double> z_t;
        state_t x_t0;
        state_t x_t1;
        string line;
        int log_idx = 0;
        bool firstPass = true;
        while (getline(log_file, line))
        {
            log_idx++;

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

#ifdef SKIP_ODO_READINGS
            // Ignoring odometry reading
            if (meas_type == 'O')
                continue;
#endif

            if (meas_type == 'L')
            {
                for (int i = 3; i < 6; i++)
                    odometry_laser.push_back(meas_vals[i]);
                for (int i = 6; i < meas_vals.size() - 1; i++)
                    ranges.push_back(meas_vals[i]);
            }

            cout << "Processing log step " << log_idx;
            cout << " at timestamp " << time_stamp;

            // Forcing first reading to be laser.
            if (firstPass)
            {
                u_t0 = odometry_robot;
                firstPass = false;
                cout << endl;
                continue;
            }

            vector<state_t> x_bar_new(num_particles);
            u_t1 = odometry_robot;

            // Skip reading if no motion in robot.
            if ((u_t1[0] - u_t0[0] == 0) &&
                (u_t1[1] - u_t0[1] == 0) &&
                (u_t1[2] - u_t0[2] == 0))
            {
                cout << ", skipping." << endl;
                continue;
            }
            else
                cout << endl;

#ifdef MOTION_MODEL_CALIBRATION_VIZ
//            map_obj.visualize_motion_model_calibration(x_bar[0], u_t0, u_t1, 1000, &motion_model);
//            exit(0);
#endif

            double w_t_sum = 0;
            // For all particles
            for (int m = 0; m < num_particles; m++)
            {
                // Motion model
                x_t0 = x_bar[m];
                x_t1 = motion_model.update(u_t0, u_t1, x_t0);

                // Sensor model
                if (meas_type == 'L')
                {
                    double w_t;
                    z_t = ranges;
                    w_t = sensor_model.beam_range_finder_model(z_t, x_t1);
                    x_t1.weight = w_t;
                    w_t_sum += w_t;
                }
                else
                {
                    x_t1.weight = x_bar[m].weight;   // Use the old weight
                }

                x_bar_new[m] = x_t1;
            }

            // Normalize weights.
            if ((int)w_t_sum == 0)
                throw runtime_error("Sum of particle weights is zero!");

            for (int m = 0; m < num_particles; m++)
            {
                x_bar_new[m].weight /= w_t_sum;
            }

            x_bar = x_bar_new;
            u_t0 = u_t1;

            // Resampling
            x_bar = resampler.low_variance_sampler(x_bar);
            // x_bar = resampler.multinomial_sampler(x_bar);

#ifdef MAP_VISUALIZE
            if (vis_flag)
            {
                // To visualize particles only
                map_obj.visualize_map(x_bar, true);
                // To visualize the particles with measurements
                // map_obj.visualize_map(x_bar, true, false, NULL, true, ranges);
            }
#endif
        }

        log_file.close();
    }

    else
    {
        cout << "Log file " << src_path_log << " could not be opened.";
        cout << endl;
    }


#ifdef MAP_VISUALIZE
    map_obj.save_video("../result/robotmovie.avi");
#endif
    return 0;
}
