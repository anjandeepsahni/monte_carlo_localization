#include <iostream>
#include <cmath>
#include "../include/motionModel.hh"
#include <random>
#include <vector>
//#include <fstream>
//#include <sstream>
using namespace std;
#include <bits/stdc++.h>



//construcor initializing the tuning paramters, starting from small uniform values

MotionModel::MotionModel(double alpha1, double alpha2, double alpha3, double alpha4)
{
            alpha1=0.1;
            alpha2=0.1;
            alpha3=0.1;
            alpha4=0.1;
}
MotionModel::~MotionModel()
{
}
 
vector<double> MotionModel::update(vector<double> u_t0, vector<double> u_t1, vector<double> x_t0)
{
    double del_rot1, del_trans, del_rot2, ddash_rot1, ddash_trans, ddash_rot2, term1, term2, term3;


    //Simple trignometric operations to calculate the robots displacement <s,s'>
    del_rot1 = atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2];
    del_trans = sqrt(pow((u_t0[0]-u_t1[0]),2)+ pow((u_t0[1]-u_t1[1]),2));
    del_rot2= u_t1[2]- u_t0[2]- del_rot1;

    //standard deviation for each parmeter used in the next segment 
    term1 = alpha1*del_rot1 + alpha2*del_trans;
    term2= alpha3*del_trans + alpha4*(del_rot1 + del_rot2);
    term3= alpha1*del_rot2 + alpha2*del_trans;

    normal_distribution<double> d1{0, term1};
    normal_distribution<double> d2{0, term2};
    normal_distribution<double> d3{0, term3};
    //Gaussian noise distribution with zero mean and sd for each paramter modelling the noise of each parameter
    random_device rd{};
    mt19937 gen{rd()};
    ddash_rot1= del_rot1 - d1(gen);
    ddash_trans= del_trans - d2(gen);
    ddash_rot2= del_trans - d3(gen);

    // converting parameters from the robot's coordinate frame to the world coordinate frame, again by simple trignometric calculations. 
    vector<double> xt(3, 0.0);
    xt[0]= x_t0[0] + ddash_trans* cos(x_t0[2] + ddash_rot1);
    xt[1]= x_t0[1] + ddash_trans* sin(x_t0[2]+ ddash_rot1);
    xt[2]= x_t0[2] + ddash_rot1 + ddash_rot2;
    
    return xt;
    
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
    // loading map and log file containing odometry data
    string src_path_map = "../data/map/wean.dat";
    string src_path_log = "../data/log/robotdata1.log";

    // mapreader();
    // Get occupancy map (Anjandeep)
   
   
    // Calling motion model only
    MotionModel motion_model(0.1,0.1,0.1,0.1);

    // Checking with only one particle 
    int num_particles = 1;
    vector<vector<double>> x_bar;
    x_bar = init_particles_random(num_particles);

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
            vector<double> path_u_t0x;
		    vector<double> path_u_t0y;
		    vector<double> path_x_t0x;
		    vector<double> path_x_t0y;
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

            for (int m = 0; m < num_particles; m++)
            {
                // MOTION MODEL (Sanjana)
                x_t0 = vector<double>(x_bar[m].begin(), x_bar[m].begin() + 3);
                x_t1= motion_model.update(u_t0, u_t1, x_t0);
                x_bar_new[m] = x_t1;
            }

            x_bar = x_bar_new;
            u_t0 = u_t1;

            path_u_t0x.push_back(u_t0[0]);
            path_u_t0y.push_back(u_t0[1]);
            path_x_t0x.push_back(x_bar[0][0]);
            path_x_t0y.push_back(x_bar[1][1]);
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
