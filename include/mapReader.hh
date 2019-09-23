#ifndef _MAPREADER_H
#define _MAPREADER_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include "particleFilter.hh"
#include "config.hh"

#if defined(MAP_VISUALIZE) || defined(MOTION_MODEL_CALIBRATION_VIZ)
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

using namespace std;
#ifdef MAP_VISUALIZE
using namespace cv;
#endif

class SensorModel;  // Forward declaration
class MotionModel;  // Forward declaration

typedef struct {
    int resolution, size_x, size_y;
    double offset_x, offset_y;
    int min_x, max_x, min_y, max_y;
    vector<vector<double>> prob;   // prob of (x,y) being occupied
} map_type;


class MapReader
{
public:
    map_type map;
    string mapName;

    MapReader(string mapName);
    int read_map();
#ifdef MAP_VISUALIZE
    int visualize_map(vector<state_t> x_bar={}, bool storeForVideo=false, bool visRays=false, SensorModel* sensor_model=NULL, bool visMeas=false, vector<double> z_t={});
    int save_video(string videoPath);
#endif
#ifdef MOTION_MODEL_CALIBRATION_VIZ
    int visualize_motion_model_calibration(state_t x_t0 ,vector<double> u_t0,
                                           vector<double> u_t1, int num_samples, MotionModel* motion_model);
#endif

private:
#ifdef MAP_VISUALIZE
    Mat mapBaseImage;
    vector<Mat> videoFrames;
#endif

};

#endif  /* _MAPREADER_H */
