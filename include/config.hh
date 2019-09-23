#ifndef _CONFIG_H
#define _CONFIG_H

// Input file paths
#define MAP_FILE_PATH "../data/map/wean.dat"
#define LOG_FILE_PATH "../data/log/robotdata1.log"

// Particle parameters
#define NUM_PARTICLES       3000
#define PARTICLE_THRESH     0.9     // probability
#define FREE_SPACE_THRESH   0.999   // probability

// Ray casting parameters
#define LASER_OFFSET        25.0    // cm
#define MAX_SENSOR_THETA    180     // degrees
#define LASER_THETA_STEP    5       // degrees
#define LASER_DIST_STEP     1       // cm
#define LASER_MAX_RANGE     8183.0  // cm, log1 -> 8183.0, others -> 8191.0

// Motion model parameters
#define ALPHA_1 0.005   // 0.0001 -> Initial config
#define ALPHA_2 0.005   // 0.0001 -> Initial config
#define ALPHA_3 0.1     // 0.01 -> Initial config
#define ALPHA_4 0.1     // 0.01 -> Initial config

// Sensor model parameters
#define Z_HIT   0.7
#define Z_SHORT 0.24
#define Z_MAX   0.0055
#define Z_RAND  0.0545
#define P_HIT_STD       20      // cm
#define LAMBDA_SHORT    0.01

// Feature support
#define SKIP_ODO_READINGS
#define MAP_VISUALIZE
//#define MOTION_MODEL_CALIBRATION_VIZ

#endif /* _CONFIG_H */
