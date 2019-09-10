#include <iostream>
using namespace std;


class SensorModel
{
    private:
        double z_hit;                           // coefficient for p_hit
        double z_short;                         // coefficient for p_short
        double z_max;                           // coefficient for p_max
        double z_rand;                          // coefficient for p_rand
        double max_range;                       // maximum range
        vector<vector<double>> occupancy_map;   // occupancy map

    public:
        SensorModel(double z_h,
                    double z_s,
                    double z_m,
                    double z_r,
                    double m_r,
                    vector<vector<double>> o_m);
        ~SensorModel();

        /*
         * beam_range_finder_model
         * - z_t: range measurements
         * - x_t1: predicted belief
         */
        double beam_range_finder_model(const vector<double> z_t1,
                                       const vector<double> x_t1);

        double ray_casting(void);
};