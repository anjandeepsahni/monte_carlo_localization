#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include "mapReader.hh"
#include "sensorModel.hh"
#include "motionModel.hh"
#include "particleFilter.hh"

#ifdef MAP_VISUALIZE
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

using namespace std;

#ifdef MAP_VISUALIZE
using namespace cv;
#endif

MapReader::MapReader(string mapName): mapName(mapName)
{
}


int MapReader::read_map()
{
    int x, y, count;
    float temp;
    char line[256];
    FILE *fp;

    if((fp = fopen(mapName.c_str(), "rt")) == NULL)
    {
        fprintf(stderr, "ERROR: Could not open file %s. %s\n", mapName.c_str(), strerror(errno));
        return -1;
    }

    fprintf(stderr, "Reading map: %s\n", mapName.c_str());
    while((fgets(line, 256, fp) != NULL)
          && (strncmp("global_map[0]", line , 13) != 0))
    {
        if(strncmp(line, "robot_specifications->resolution", 32) == 0)
        {
            if(sscanf(&line[32], "%d", &(map.resolution)) != 0)
            {
                printf("Map resolution: %d cm\n", map.resolution);
            }
        }

        if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
        {
            if(sscanf(&line[35], "%lf", &(map.offset_x)) != 0)
            {
                printf("Map offsetX: %lf cm\n", map.offset_x);
            }
        }

        if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0)
        {
            if (sscanf(&line[35], "%lf", &(map.offset_y)) != 0)
            {
                printf("Map offsetY: %lf cm\n", map.offset_y);
            }
        }
    }

    if(sscanf(line,"global_map[0]: %d %d", &map.size_y, &map.size_x) != 2)
    {
        fprintf(stderr, "ERROR: Corrupted file %s\n", mapName.c_str());
        fclose(fp);
        return -1;
    }

    printf("Map size: %dx%d\n", map.size_x, map.size_y);

    map.min_x = map.size_x;
    map.max_x = 0;
    map.min_y = map.size_y;
    map.max_y = 0;
    count = 0;
    for(x = 0; x < map.size_x; x++)
    {
        vector<double> prob;
        for(y = 0; y < map.size_y; y++, count++)
        {
            fscanf(fp, "%e", &temp);
            if(temp < 0.0)
            {
                // prob < 0 means don't know
                prob.push_back(-1);
            }
            else
            {
                if(x < map.min_x)
                    map.min_x = x;
                else if(x > map.max_x)
                    map.max_x = x;
                if(y < map.min_y)
                    map.min_y = y;
                else if(y > map.max_y)
                    map.max_y = y;
                // probability of (x,y) being free
                prob.push_back(temp);
            }
        }
        map.prob.push_back(prob);
    }
    printf("Map min_X: %d, max_X %d\n", map.min_x, map.max_x);
    printf("Map min_Y: %d, max_Y %d\n", map.min_y, map.max_y);
    fclose(fp);
    fprintf(stderr, "Reading map: COMPLETED.\n");
    return 0;
}

#ifdef MAP_VISUALIZE

int MapReader::visualize_map(vector<state_t> x_bar, bool storeForVideo, bool visRays, SensorModel* sensor_model, bool visMeas, vector<double> z_t)
{
    if (visRays & visMeas)
        throw runtime_error("Cannot visualize both ray casting and sensor measurements together !");

    if (visMeas && (z_t.size() != MAX_SENSOR_THETA))
        throw runtime_error("Requested visualization of measurements but z_t does not have enough elements!");

    int res = map.resolution;

    // Mat is accessed as (row,col), which means (y,x)
    // 0/0 --col-->
    // |
    // row
    // |
    // v
    Mat image = Mat::zeros(map.size_y, map.size_x, CV_32FC1);

    for (unsigned int i = 0; i < image.rows; i++)
    {
        for (unsigned int j = 0; j < image.cols; j++)
        {
#ifdef FLIP_Y_AXIS
            // Mat is accessed as (row,col), which means (y,x)
            // Assuming that map.dat was stored as (x,y)
            // ^
            // |
            // Y
            // |
            // 0/0 --X-->
            // So we should access as map.prob[j][i]
            // But image axis is:
            // 0/0 --X-->
            // |
            // Y
            // |
            // v
            // So we should access as map.prob[j][image.rows-i-1]
            if (map.prob[j][image.rows-i-1] > 0.0)
                image.at<float>(i, j) = map.prob[j][image.rows-i-1];
#else
            if (map.prob[i][j] > 0.0)
                image.at<float>(j, i) = map.prob[i][j];
#endif
        }
    }

    cvtColor(image, image, COLOR_GRAY2BGR);

    if (!x_bar.empty())
    {
        for (int i=0; i < x_bar.size(); ++i)
        {
            state_t x_t1 = x_bar[i];
            Point particle = Point_<int>((int)(x_t1.x/res), (int)(x_t1.y/res));
            circle(image, particle, 1, Scalar(0, 0, 255), 2, 8);

            if (visRays || visMeas)
            {
                double x = x_t1.x + LASER_OFFSET * cos(x_t1.theta);
                double y = x_t1.y + LASER_OFFSET * sin(x_t1.theta);
                Point ray_start = Point_<int>((int)(x/res), (int)(y/res));
                for (int j = 0; j < MAX_SENSOR_THETA; j+=5)
                {
                    // Angle wrt x axis
                    double angle = ((double)j * (M_PI / 180)) + x_t1.theta - M_PI_2;
                    double dist;
                    if (visRays)
                        dist = sensor_model->ray_casting(x_t1, angle);
                    else
                        dist = z_t[j];
                    double x_end = (x + dist * cos(angle));
                    double y_end = (y + dist * sin(angle));
                    Point ray_end = Point_<int>((int)(x_end/res), (int)(y_end/res));
                    line(image, ray_start, ray_end, Scalar(255, 0, 0));
                }
            }
        }
    }

    if (storeForVideo)
    {
        image.convertTo(image, CV_8UC3, 255.0);
        videoFrames.push_back(image);
    }
    else
    {
        imshow("Image", image);
        waitKey(0);
        destroyWindow("Image");
    }
    return 0;
}

#ifdef MOTION_MODEL_CALIBRATION_VIZ
int MapReader::visualize_motion_model_calibration(state_t x_t0 ,vector<double> u_t0,
                                                  vector<double> u_t1, int num_samples,
                                                  MotionModel* motion_model)
{
    vector<state_t> x_t1_samples;
    state_t x_t1;
    int res = map.resolution;
    double x_t1_meanx = 0.0, x_t1_meany = 0.0;

    for (int i = 0; i < num_samples; i++)
    {
        x_t1 = motion_model->update(u_t0, u_t1, x_t0);
        x_t1_samples.push_back(x_t1);
        x_t1_meanx += x_t1.x;
        x_t1_meany += x_t1.y;
    }

    x_t1_meanx /= num_samples;
    x_t1_meany /= num_samples;

    Mat image = Mat::zeros(map.size_y, map.size_x, CV_32FC1);
    cvtColor(image, image, COLOR_GRAY2BGR);

    // Plot x_t0 and angle direction.
    Point particle = Point_<int>((int)(x_t0.x/res), (int)(x_t0.y/res));
    circle(image, particle, 1, Scalar(0, 0, 255), 2, 8);
    Point avg_pred_particle = Point_<int>((int)(x_t1_meanx/res), (int)(x_t1_meany/res));
    line(image, particle, avg_pred_particle, Scalar(255, 0, 0));

    if (!x_t1_samples.empty())
    {
        for (int i=0; i < x_t1_samples.size(); ++i)
        {
            state_t x_t1 = x_t1_samples[i];
            Point particle = Point_<int>((int)(x_t1.x/res), (int)(x_t1.y/res));
            circle(image, particle, 1, Scalar(0, 0, 255), 2, 8);
        }
    }

    imshow("Image", image);
    waitKey(0);
    destroyWindow("Image");

    return 0;
}

#endif /* MOTION_MODEL_CALIBRATION_VIZ */

int MapReader::save_video(string videoPath)
{
    if (videoFrames.empty())
        return -1;

    int width = (videoFrames[0].size()).width;
    int height = (videoFrames[0].size()).height;
    VideoWriter video = VideoWriter(videoPath, 0, 1, Size2i(width, height));

    for (int i = 0; i < videoFrames.size(); ++i)
        video.write(videoFrames[i]);

    video.release();
    return 0;
}

#endif /* MAP_VISUALIZE */

