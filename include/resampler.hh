#ifndef _RESAMPLER_H
#define _RESAMPLER_H

#include <iostream>
#include <vector>
#include "particleFilter.hh"

using namespace std;

class Resampler
{
public:
    vector<state_t> multinomial_sampler(vector<state_t> x_bar);
    vector<state_t> low_variance_sampler(vector<state_t> x_bar);
};

#endif  /* _RESAMPLER_H */
