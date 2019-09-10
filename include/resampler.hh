#ifndef _RESAMPLER_H
#define _RESAMPLER_H

#include <iostream>

using namespace std;

class Resampler
{
public:
    vector<vector<double>> multinomial_sampler(vector<vector<double>> x_bar);
    vector<vector<double>> low_variance_sampler(vector<vector<double>> x_bar);
};

#endif  /* _RESAMPLER_H */
