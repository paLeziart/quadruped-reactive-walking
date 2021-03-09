#include "quadruped-reactive-walking/TrajGen.hpp"

// Trajectory generator functions (output reference pos, vel and acc of feet in swing phase)

TrajGen::TrajGen() {}

TrajGen::TrajGen(double h_in, double t_lock_in, double x_in, double y_in)
{
    h = h_in;
    time_adaptative_disabled = t_lock_in;
    x1 = x_in;
    y1 = y_in;

    for (int i = 0; i < 6; i++)
    {
        lastCoeffs_x[i] = 0.0;
        lastCoeffs_y[i] = 0.0;
    }
}

Eigen::Matrix<double, 11, 1> TrajGen::get_next_foot(double x0, double dx0, double ddx0, double y0, double dy0,
                                                    double ddy0, double x1_in, double y1_in, double t0, double t1,
                                                    double dt)
{
    /* Compute the reference position, velocity and acceleration of a foot in swing phase

  Args:
    x0 (double): current X position of the foot
    dx0 (double): current X velocity of the foot
    ddx0 (double): current X acceleration of the foot
    y0 (double): current Y position of the foot
    dy0 (double): current Y velocity of the foot
    ddy0 (double): current Y acceleration of the foot
    x1 (double): desired target location for X at the end of the swing phase
    y1 (double): desired target location for Y at the end of the swing phase
    t0 (double): time elapsed since the start of the swing phase
    t1 (double): duration of the swing phase
    dt (double): time step of the control
  */

    double epsilon = 0.0;
    double t2 = t1;
    double t3 = t0;
    t1 -= 2 * epsilon;
    t0 -= epsilon;

    if ((t1 - t0) > time_adaptative_disabled)
    {  // adaptative_mode

        // compute polynoms coefficients for x and y
        Ax5 = (ddx0 * std::pow(t0, 2) - 2 * ddx0 * t0 * t1 - 6 * dx0 * t0 + ddx0 * std::pow(t1, 2) + 6 * dx0 * t1 + 12 * x0 - 12 * x1_in) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ax4 = (30 * t0 * x1_in - 30 * t0 * x0 - 30 * t1 * x0 + 30 * t1 * x1_in - 2 * std::pow(t0, 3) * ddx0 - 3 * std::pow(t1, 3) * ddx0 + 14 * std::pow(t0, 2) * dx0 - 16 * std::pow(t1, 2) * dx0 + 2 * t0 * t1 * dx0 + 4 * t0 * std::pow(t1, 2) * ddx0 + std::pow(t0, 2) * t1 * ddx0) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ax3 = (std::pow(t0, 4) * ddx0 + 3 * std::pow(t1, 4) * ddx0 - 8 * std::pow(t0, 3) * dx0 + 12 * std::pow(t1, 3) * dx0 + 20 * std::pow(t0, 2) * x0 - 20 * std::pow(t0, 2) * x1_in + 20 * std::pow(t1, 2) * x0 - 20 * std::pow(t1, 2) * x1_in + 80 * t0 * t1 * x0 - 80 * t0 * t1 * x1_in + 4 * std::pow(t0, 3) * t1 * ddx0 + 28 * t0 * std::pow(t1, 2) * dx0 - 32 * std::pow(t0, 2) * t1 * dx0 - 8 * std::pow(t0, 2) * std::pow(t1, 2) * ddx0) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ax2 = -(std::pow(t1, 5) * ddx0 + 4 * t0 * std::pow(t1, 4) * ddx0 + 3 * std::pow(t0, 4) * t1 * ddx0 + 36 * t0 * std::pow(t1, 3) * dx0 - 24 * std::pow(t0, 3) * t1 * dx0 + 60 * t0 * std::pow(t1, 2) * x0 + 60 * std::pow(t0, 2) * t1 * x0 - 60 * t0 * std::pow(t1, 2) * x1_in - 60 * std::pow(t0, 2) * t1 * x1_in - 8 * std::pow(t0, 2) * std::pow(t1, 3) * ddx0 - 12 * std::pow(t0, 2) * std::pow(t1, 2) * dx0) / (2 * (std::pow(t0, 2) - 2 * t0 * t1 + std::pow(t1, 2)) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ax1 = -(2 * std::pow(t1, 5) * dx0 - 2 * t0 * std::pow(t1, 5) * ddx0 - 10 * t0 * std::pow(t1, 4) * dx0 + std::pow(t0, 2) * std::pow(t1, 4) * ddx0 + 4 * std::pow(t0, 3) * std::pow(t1, 3) * ddx0 - 3 * std::pow(t0, 4) * std::pow(t1, 2) * ddx0 - 16 * std::pow(t0, 2) * std::pow(t1, 3) * dx0 + 24 * std::pow(t0, 3) * std::pow(t1, 2) * dx0 - 60 * std::pow(t0, 2) * std::pow(t1, 2) * x0 + 60 * std::pow(t0, 2) * std::pow(t1, 2) * x1_in) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ax0 = (2 * x1_in * std::pow(t0, 5) - ddx0 * std::pow(t0, 4) * std::pow(t1, 3) - 10 * x1_in * std::pow(t0, 4) * t1 + 2 * ddx0 * std::pow(t0, 3) * std::pow(t1, 4) + 8 * dx0 * std::pow(t0, 3) * std::pow(t1, 3) + 20 * x1_in * std::pow(t0, 3) * std::pow(t1, 2) - ddx0 * std::pow(t0, 2) * std::pow(t1, 5) - 10 * dx0 * std::pow(t0, 2) * std::pow(t1, 4) - 20 * x0 * std::pow(t0, 2) * std::pow(t1, 3) + 2 * dx0 * t0 * std::pow(t1, 5) + 10 * x0 * t0 * std::pow(t1, 4) - 2 * x0 * std::pow(t1, 5)) / (2 * (std::pow(t0, 2) - 2 * t0 * t1 + std::pow(t1, 2)) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));

        Ay5 = (ddy0 * std::pow(t0, 2) - 2 * ddy0 * t0 * t1 - 6 * dy0 * t0 + ddy0 * std::pow(t1, 2) + 6 * dy0 * t1 + 12 * y0 - 12 * y1_in) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ay4 = (30 * t0 * y1_in - 30 * t0 * y0 - 30 * t1 * y0 + 30 * t1 * y1_in - 2 * std::pow(t0, 3) * ddy0 - 3 * std::pow(t1, 3) * ddy0 + 14 * std::pow(t0, 2) * dy0 - 16 * std::pow(t1, 2) * dy0 + 2 * t0 * t1 * dy0 + 4 * t0 * std::pow(t1, 2) * ddy0 + std::pow(t0, 2) * t1 * ddy0) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ay3 = (std::pow(t0, 4) * ddy0 + 3 * std::pow(t1, 4) * ddy0 - 8 * std::pow(t0, 3) * dy0 + 12 * std::pow(t1, 3) * dy0 + 20 * std::pow(t0, 2) * y0 - 20 * std::pow(t0, 2) * y1_in + 20 * std::pow(t1, 2) * y0 - 20 * std::pow(t1, 2) * y1_in + 80 * t0 * t1 * y0 - 80 * t0 * t1 * y1_in + 4 * std::pow(t0, 3) * t1 * ddy0 + 28 * t0 * std::pow(t1, 2) * dy0 - 32 * std::pow(t0, 2) * t1 * dy0 - 8 * std::pow(t0, 2) * std::pow(t1, 2) * ddy0) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ay2 = -(std::pow(t1, 5) * ddy0 + 4 * t0 * std::pow(t1, 4) * ddy0 + 3 * std::pow(t0, 4) * t1 * ddy0 + 36 * t0 * std::pow(t1, 3) * dy0 - 24 * std::pow(t0, 3) * t1 * dy0 + 60 * t0 * std::pow(t1, 2) * y0 + 60 * std::pow(t0, 2) * t1 * y0 - 60 * t0 * std::pow(t1, 2) * y1_in - 60 * std::pow(t0, 2) * t1 * y1_in - 8 * std::pow(t0, 2) * std::pow(t1, 3) * ddy0 - 12 * std::pow(t0, 2) * std::pow(t1, 2) * dy0) / (2 * (std::pow(t0, 2) - 2 * t0 * t1 + std::pow(t1, 2)) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ay1 = -(2 * std::pow(t1, 5) * dy0 - 2 * t0 * std::pow(t1, 5) * ddy0 - 10 * t0 * std::pow(t1, 4) * dy0 + std::pow(t0, 2) * std::pow(t1, 4) * ddy0 + 4 * std::pow(t0, 3) * std::pow(t1, 3) * ddy0 - 3 * std::pow(t0, 4) * std::pow(t1, 2) * ddy0 - 16 * std::pow(t0, 2) * std::pow(t1, 3) * dy0 + 24 * std::pow(t0, 3) * std::pow(t1, 2) * dy0 - 60 * std::pow(t0, 2) * std::pow(t1, 2) * y0 + 60 * std::pow(t0, 2) * std::pow(t1, 2) * y1_in) / (2 * std::pow((t0 - t1), 2) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));
        Ay0 = (2 * y1_in * std::pow(t0, 5) - ddy0 * std::pow(t0, 4) * std::pow(t1, 3) - 10 * y1_in * std::pow(t0, 4) * t1 + 2 * ddy0 * std::pow(t0, 3) * std::pow(t1, 4) + 8 * dy0 * std::pow(t0, 3) * std::pow(t1, 3) + 20 * y1_in * std::pow(t0, 3) * std::pow(t1, 2) - ddy0 * std::pow(t0, 2) * std::pow(t1, 5) - 10 * dy0 * std::pow(t0, 2) * std::pow(t1, 4) - 20 * y0 * std::pow(t0, 2) * std::pow(t1, 3) + 2 * dy0 * t0 * std::pow(t1, 5) + 10 * y0 * t0 * std::pow(t1, 4) - 2 * y0 * std::pow(t1, 5)) / (2 * (std::pow(t0, 2) - 2 * t0 * t1 + std::pow(t1, 2)) * (std::pow(t0, 3) - 3 * std::pow(t0, 2) * t1 + 3 * t0 * std::pow(t1, 2) - std::pow(t1, 3)));

        // Save coeffs
        lastCoeffs_x[0] = Ax5;
        lastCoeffs_x[1] = Ax4;
        lastCoeffs_x[2] = Ax3;
        lastCoeffs_x[3] = Ax2;
        lastCoeffs_x[4] = Ax1;
        lastCoeffs_x[5] = Ax0;
        lastCoeffs_y[0] = Ay5;
        lastCoeffs_y[1] = Ay4;
        lastCoeffs_y[2] = Ay3;
        lastCoeffs_y[3] = Ay2;
        lastCoeffs_y[4] = Ay1;
        lastCoeffs_y[5] = Ay0;
        x1 = x1_in;
        y1 = y1_in;
    }
    else
    {
        // Use last coefficients
        Ax5 = lastCoeffs_x[0];
        Ax4 = lastCoeffs_x[1];
        Ax3 = lastCoeffs_x[2];
        Ax2 = lastCoeffs_x[3];
        Ax1 = lastCoeffs_x[4];
        Ax0 = lastCoeffs_x[5];
        Ay5 = lastCoeffs_y[0];
        Ay4 = lastCoeffs_y[1];
        Ay3 = lastCoeffs_y[2];
        Ay2 = lastCoeffs_y[3];
        Ay1 = lastCoeffs_y[4];
        Ay0 = lastCoeffs_y[5];
    }

    // Coefficients for z (deterministic)
    Az6 = -h / (std::pow((t2 / 2), 3) * std::pow((t2 - t2 / 2), 3));
    Az5 = (3 * t2 * h) / (std::pow((t2 / 2), 3) * std::pow((t2 - t2 / 2), 3));
    Az4 = -(3 * std::pow(t2, 2) * h) / (std::pow((t2 / 2), 3) * std::pow((t2 - t2 / 2), 3));
    Az3 = (std::pow(t2, 3) * h) / (std::pow((t2 / 2), 3) * std::pow((t2 - t2 / 2), 3));

    // Get the next point
    double ev = t0 + dt;
    double evz = t3 + dt;

    result(6, 0) = Az3 * std::pow(evz, 3) + Az4 * std::pow(evz, 4) + Az5 * std::pow(evz, 5) + Az6 * std::pow(evz, 6);                     // pos Z
    result(7, 0) = 3 * Az3 * std::pow(evz, 2) + 4 * Az4 * std::pow(evz, 3) + 5 * Az5 * std::pow(evz, 4) + 6 * Az6 * std::pow(evz, 5);     // vel Z
    result(8, 0) = 2 * 3 * Az3 * evz + 3 * 4 * Az4 * std::pow(evz, 2) + 4 * 5 * Az5 * std::pow(evz, 3) + 5 * 6 * Az6 * std::pow(evz, 4);  // acc Z
    result(9, 0) = x1;                                                                                                                    // current goal x
    result(10, 0) = y1;                                                                                                                   // current goal y

    if ((t3 < epsilon) || (t3 > (t2 - epsilon)))
    {  // Just vertical motion
        result(0, 0) = x0;
        result(1, 0) = 0.0;
        result(2, 0) = 0.0;
        result(3, 0) = y0;
        result(4, 0) = 0.0;
        result(5, 0) = 0.0;
    }
    else
    {
        // pos, vel, acc X
        result(0, 0) = Ax0 + Ax1 * ev + Ax2 * std::pow(ev, 2) + Ax3 * std::pow(ev, 3) + Ax4 * std::pow(ev, 4) + Ax5 * std::pow(ev, 5);
        result(1, 0) = Ax1 + 2 * Ax2 * ev + 3 * Ax3 * std::pow(ev, 2) + 4 * Ax4 * std::pow(ev, 3) + 5 * Ax5 * std::pow(ev, 4);
        result(2, 0) = 2 * Ax2 + 3 * 2 * Ax3 * ev + 4 * 3 * Ax4 * std::pow(ev, 2) + 5 * 4 * Ax5 * std::pow(ev, 3);

        // pos, vel, acc Y
        result(3, 0) = Ay0 + Ay1 * ev + Ay2 * std::pow(ev, 2) + Ay3 * std::pow(ev, 3) + Ay4 * std::pow(ev, 4) + Ay5 * std::pow(ev, 5);
        result(4, 0) = Ay1 + 2 * Ay2 * ev + 3 * Ay3 * std::pow(ev, 2) + 4 * Ay4 * std::pow(ev, 3) + 5 * Ay5 * std::pow(ev, 4);
        result(5, 0) = 2 * Ay2 + 3 * 2 * Ay3 * ev + 4 * 3 * Ay4 * std::pow(ev, 2) + 5 * 4 * Ay5 * std::pow(ev, 3);
    }

    return result;
}
