#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mapreader.hh"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

MapReader::MapReader(const char *mapName): mapName(mapName)
{
}


int MapReader::read_map()
{
    int x, y, count;
    float temp;
    char line[256];
    FILE *fp;
    map = this->map;

    if((fp = fopen(mapName, "rt")) == NULL)
    {
        fprintf(stderr, "ERROR: Could not open file %s. %s\n", mapName, strerror(errno));
        return -1;
    }

    fprintf(stderr, "Reading map: %s\n", mapName);
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
            if(sscanf(&line[35], "%g", &(map.offset_x)) != 0)
            {
                printf("Map offsetX: %g cm\n", map.offset_x);
            }
        }

        if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0)
        {
            if (sscanf(&line[35], "%g", &(map.offset_y)) != 0)
            {
                printf("Map offsetY: %g cm\n", map.offset_y);
            }
        }
    }

    if(sscanf(line,"global_map[0]: %d %d", &map.size_y, &map.size_x) != 2)
    {
        fprintf(stderr, "ERROR: Corrupted file %s\n", mapName);
        fclose(fp);
        return -1;
    }

    printf("Map size: %dx%d\n", map.size_x, map.size_y);

    // allocate memory for map prob
    map.prob = (float **)calloc(map.size_x, sizeof(float *));
    for(int i = 0; i < map.size_x; i++)
        map.prob[i] = (float *)calloc(map.size_y, sizeof(float));

    map.min_x = map.size_x;
    map.max_x = 0;
    map.min_y = map.size_y;
    map.max_y = 0;
    count = 0;
    for(x = 0; x < map.size_x; x++)
    {
        for(y = 0; y < map.size_y; y++, count++)
        {
            fscanf(fp, "%e", &temp);
            if(temp < 0.0)
            {
                map.prob[x][y] = -1;
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
                map.prob[x][y] = temp;
            }
        }
    }
    printf("Map min_X: %d, max_X %d\n", map.min_x, map.max_x);
    printf("Map min_Y: %d, max_Y %d\n", map.min_y, map.max_y);
    fclose(fp);
    fprintf(stderr, "Reading map: COMPLETED.\n");
    return 0;
}

int MapReader::visulize_map()
{
    map = this->map;
    Mat image = Mat::zeros(map.size_x, map.size_y, CV_32FC1);

    for (unsigned int i = 0; i < image.rows; i++)
    {
        for (unsigned int j = 0; j < image.cols; j++)
        {
            if (map.prob[i][j] > 0.0)
                image.at<float>(i, j) = map.prob[i][j];
        }
    }
    imshow("Image", image);
    waitKey(0);
    return 0;
}
