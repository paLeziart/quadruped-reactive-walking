#ifndef TRAJGEN_H_INCLUDED
#define TRAJGEN_H_INCLUDED

#include "pinocchio/math/rpy.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#define N0_gait 20
// Number of rows in the gait matrix. Arbitrary value that should be set high enough so that there is always at
// least one empty line at the end of the gait matrix

typedef Eigen::MatrixXd matXd;

class TrajGen
{
    /* Class that generates a reference trajectory in position, velocity and acceleration that feet it swing phase
     should follow */

private:
    double lastCoeffs_x[6];                                                      // Coefficients for the X component
    double lastCoeffs_y[6];                                                      // Coefficients for the Y component
    double h = 0.05;                                                             // Apex height of the swinging trajectory
    double time_adaptative_disabled = 0.2;                                       // Target lock before the touchdown
    double x1 = 0.0;                                                             // Target for the X component
    double y1 = 0.0;                                                             // Target for the Y component
    Eigen::Matrix<double, 11, 1> result = Eigen::Matrix<double, 11, 1>::Zero();  // Output of the generator

    // Coefficients
    double Ax5 = 0.0, Ax4 = 0.0, Ax3 = 0.0, Ax2 = 0.0, Ax1 = 0.0, Ax0 = 0.0, Ay5 = 0.0, Ay4 = 0.0, Ay3 = 0.0, Ay2 = 0.0,
           Ay1 = 0.0, Ay0 = 0.0, Az6 = 0.0, Az5 = 0.0, Az4 = 0.0, Az3 = 0.0;

public:
    TrajGen();                                                         // Empty constructor
    TrajGen(double h_in, double t_lock_in, double x_in, double y_in);  // Default constructor
    Eigen::Matrix<double, 11, 1> get_next_foot(double x0, double dx0, double ddx0, double y0, double dy0, double ddy0,
                                               double x1_in, double y1_in, double t0, double t1, double dt);
};
#endif  // PLANNER_H_INCLUDED
