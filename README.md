# Monte Carlo Localization
This repository implements the particle filter algorithm. The implementation is done in `C++`. Below is a brief description of source files:
1.  `main.cc`: The main source file.
2.  `mapReader.cc`: To load the map.
3.  `motionModel.cc`: Implements motion model.
4.  `resampler.cc`: Implements multinomial and low variance resampler.
5.  `sensorModel.cc`: Implements sensor mode.

## Motion Model
The role of the motion model is to model the state transition of the robot. The model takes into account the previous state and the motion command leading to the robot’s current state. We use an Odometry Motion Model and hence the motion commands are a series of readings from the robot’s odometer. The state is defined by three parameters, the x and y coordinates of the robot and theta – the angular orientation of the robot.

There are 4 noise parameters:
1.  **Angular transition noise**: α₁ and α₂
2.  **Translation noise**: α₃ and α₄

These parameters are configured to the following values:

| **Noise coefficient** | **Value** |
| :-------------------: | :-------: |
|      α₁      |   0.005   |
|      α₂      |   0.005   |
|      α₃      |    0.1    |
|      α₄      |    0.1    |

## Sensor Model
The sensor model takes into account the prediction of the motion model and corrects it using the sensor readings. In this case, we considered 4 sources of sensor noise:

1.  **Correct range with local measurement noise.** This is represented by a Gaussian distribution and is implemented in the function `SensorModel::p_hit`.
2.  **Unexpected objects.** This is represented by an exponential distribution and is implemented in the function `SensorModel::p_short`.
3.  **Failures.** This is represented by an Indicator function and is implemented in the function `SensorModel::p_max`.
4.  **Random measurements.** This is represented by a uniform distribution and is implemented in `SensorModel::p_rand`.

The noise parameters of sensor model are configured as below:

| **Noise coefficient** | **Value**  |
| :-------------------: | :--------: |
|      z_hit      |  0.7   |
|     z_short     |  0.24  |
|      z_max      | 0.0055 |
|     z_rand      | 0.0545 |
|   sigma_hit    |   20   |
|  lambda_short  |    0.01    |

## Resampling Process
This repo implements both multinomial and low-variance sampling schemes. In general, low-variance sampler performs better.

## Ray Casting
Ray casting is used within the sensor model to retrieve what the robot is seeing, i.e., the laser readings given the robot’s coordinate. By comparing this with the laser measurements from the log file, we validate the position of random particles and the sensor model accordingly allots weight to particles.

The performance of ray casting is controlled by configuring the parameters shown in table below:

|      **Parameter**      |  **Value**   |
| :---------------------: | :----------: |
|     dist_step      |   1 cm   |
|     theta_step     | 5◦ |
| particle_threshold |   0.9    |

## Results
Table below specifies robot’s convergence time in its own time frame as well as overall `C++` code runtime.

|       **Item**       |      **Value**       |
| :-------------------:| :------------------: |
| Avg Convergence time |       12s        |
| Avg Code Runtime     |      150s        |

Our implementation of the particle filter is sufficiently robust as it achieves convergence on all the test log files for the same set of hyper-parameters. We began testing our implementation by using 10000 particles and reduced it down to 3000 as we found it suitable for convergence across all the log files in multiple runs.

## Demo
![Result - Log 1](result/log_1.gif)
![Result - Log 1](result/log_2.gif)
