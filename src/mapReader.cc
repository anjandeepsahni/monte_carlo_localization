#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include "mapReader.hh"
#include "particleFilter.hh"

#ifdef MAP_VISUALIZE
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif /* MAP_VISUALIZE */

using namespace std;

#ifdef MAP_VISUALIZE
using namespace cv;
#endif /* MAP_VISUALIZE */

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

    // allocate memory for map prob
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
                // probability of (x,y) being occupied
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

int MapReader::visulize_map(vector<state_t> x_bar, bool storeForVideo)
{
    int res = map.resolution;

    Mat image = Mat::zeros(map.size_x, map.size_y, CV_32FC1);

    for (unsigned int i = 0; i < image.rows; i++)
    {
        for (unsigned int j = 0; j < image.cols; j++)
        {
            if (map.prob[i][j] > 0.0)
                image.at<float>(j, i) = map.prob[i][j];
        }
    }

    Mat image_color = Mat::zeros(map.size_x, map.size_y, CV_32FC3);
    cvtColor(image, image_color, COLOR_GRAY2BGR);
    
    if (!x_bar.empty())
    {
        for (int i=0; i < x_bar.size(); ++i)
        {
            circle(image_color, Point_<int>((int)(x_bar[i].y/res), (int)(x_bar[i].x/res)), 1, Scalar(0, 0, 255), 2, 8);
        }
    }

    if (storeForVideo)
    {
        image_color.convertTo(image_color, CV_8UC3, 255.0);
        videoFrames.push_back(image_color);
    }
    else
    {
        imshow("Image", image_color);
        waitKey(0);
    }
    return 0;
}


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

