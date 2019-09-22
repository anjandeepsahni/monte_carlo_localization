#include <iostream>
#include <random>
#include <stdlib.h>
#include "resampler.hh"
#include "particleFilter.hh"

using namespace std;


vector<state_t> Resampler::multinomial_sampler(vector<state_t> x_bar)
{
    // number of particles
    long M = x_bar.size();

    // Extract the particle weights.
    vector<double> weights;
    for(int i=0; i<M; ++i) {
        weights.push_back(x_bar[i].weight);
    }

    // Create the distribution with those weights.
    discrete_distribution<> d(weights.begin(), weights.end());

    // Use the distribution to resample particles.
    vector<state_t> x_bar_resampled(M);
    default_random_engine generator;
    for(int i=0; i<M; ++i) {
        x_bar_resampled[i] = x_bar[d(generator)];
    }

    return x_bar_resampled;
}


vector<state_t> Resampler::low_variance_sampler(vector<state_t> x_bar)
{
    // number of particles
    long M = x_bar.size();

    // sampler parameters
    float r = (float) (rand()) / ((float) (RAND_MAX * M));
    float c = x_bar[0].weight;
    int i = 0;
    vector<state_t> x_bar_resampled(M);

    for (int m=1; m<=M; ++m)
    {
        float u = r + ((float) (m-1) / (float) M);
        while (u > c)
        {
            ++i;
            c += x_bar[i].weight;
        }
        x_bar_resampled[m-1] = x_bar[i];
    }

    return x_bar_resampled;
}

