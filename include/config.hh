#ifndef _CONFIG_H
#define _CONFIG_H

// General parameters
#define NUM_PARTICLES 1
#define MAP_FILE_PATH "../data/map/wean.dat"
#define LOG_FILE_PATH "../data/log/robotdata1.log"
#define SKIP_ODO_READINGS
#define MAP_VISUALIZE

// Motion model parameters
#define ALPHA_1 0.01
#define ALPHA_2 0.01
#define ALPHA_3 0.1
#define ALPHA_4 0.1

// Sensor model parameters
#define Z_HIT   0.7
#define Z_SHORT 0.24
#define Z_MAX   0.0055
#define Z_RAND  0.0545
#define LASER_MAX_RANGE     8191.0  // cm
#define LASER_THETA_STEP    1       // degrees
#define LASER_DIST_STEP     1       // cm
#define P_HIT_STD           20      // cm
#define LAMBDA_SHORT        0.01
#define LASER_OFFSET        25.0    // cm
#define FREE_SPACE_THRESH   0.9999

#endif /* _CONFIG_H */
