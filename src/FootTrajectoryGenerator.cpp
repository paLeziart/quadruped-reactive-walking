#include "qrw/FootTrajectoryGenerator.hpp"

// Trajectory generator functions (output reference pos, vel and acc of feet in swing phase)

FootTrajectoryGenerator::FootTrajectoryGenerator(double const maxHeightIn, double const lockTimeIn, Vector3 const& targetFootstepIn)
    : maxHeight_(maxHeightIn)
    , lockTime_(lockTimeIn)
    , targetFootstep_(targetFootstepIn)
    , Ax(Vector6::Zero())
    , Ay(Vector6::Zero())
{
}


void FootTrajectoryGenerator::updateFootPosition(Vector3 const& position,
                                 Vector3 const& velocity,
                                 Vector3 const& acceleration,
                                 Vector3 const& targetFootstep,
                                 double const t,
                                 double const d,
                                 double const dt)
{
    double ddx0 = acceleration(0);
    double ddy0 = acceleration(1);
    double dx0 = velocity(0);
    double dy0 = velocity(1);
    double x0 = position(0);
    double y0 = position(1);

    if (t < d - lockTime_)
    {
        // compute polynoms coefficients for x and y
        Ax[0] = (ddx0 * std::pow(t, 2) - 2 * ddx0 * t * d - 6 * dx0 * t + ddx0 * std::pow(d, 2) + 6 * dx0 * d + 12 * x0 - 12 * targetFootstep[0]) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ax[1] = (30 * t * targetFootstep[0] - 30 * t * x0 - 30 * d * x0 + 30 * d * targetFootstep[0] - 2 * std::pow(t, 3) * ddx0 - 3 * std::pow(d, 3) * ddx0 + 14 * std::pow(t, 2) * dx0 - 16 * std::pow(d, 2) * dx0 + 2 * t * d * dx0 + 4 * t * std::pow(d, 2) * ddx0 + std::pow(t, 2) * d * ddx0) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ax[2] = (std::pow(t, 4) * ddx0 + 3 * std::pow(d, 4) * ddx0 - 8 * std::pow(t, 3) * dx0 + 12 * std::pow(d, 3) * dx0 + 20 * std::pow(t, 2) * x0 - 20 * std::pow(t, 2) * targetFootstep[0] + 20 * std::pow(d, 2) * x0 - 20 * std::pow(d, 2) * targetFootstep[0] + 80 * t * d * x0 - 80 * t * d * targetFootstep[0] + 4 * std::pow(t, 3) * d * ddx0 + 28 * t * std::pow(d, 2) * dx0 - 32 * std::pow(t, 2) * d * dx0 - 8 * std::pow(t, 2) * std::pow(d, 2) * ddx0) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ax[3] = -(std::pow(d, 5) * ddx0 + 4 * t * std::pow(d, 4) * ddx0 + 3 * std::pow(t, 4) * d * ddx0 + 36 * t * std::pow(d, 3) * dx0 - 24 * std::pow(t, 3) * d * dx0 + 60 * t * std::pow(d, 2) * x0 + 60 * std::pow(t, 2) * d * x0 - 60 * t * std::pow(d, 2) * targetFootstep[0] - 60 * std::pow(t, 2) * d * targetFootstep[0] - 8 * std::pow(t, 2) * std::pow(d, 3) * ddx0 - 12 * std::pow(t, 2) * std::pow(d, 2) * dx0) / (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ax[4] = -(2 * std::pow(d, 5) * dx0 - 2 * t * std::pow(d, 5) * ddx0 - 10 * t * std::pow(d, 4) * dx0 + std::pow(t, 2) * std::pow(d, 4) * ddx0 + 4 * std::pow(t, 3) * std::pow(d, 3) * ddx0 - 3 * std::pow(t, 4) * std::pow(d, 2) * ddx0 - 16 * std::pow(t, 2) * std::pow(d, 3) * dx0 + 24 * std::pow(t, 3) * std::pow(d, 2) * dx0 - 60 * std::pow(t, 2) * std::pow(d, 2) * x0 + 60 * std::pow(t, 2) * std::pow(d, 2) * targetFootstep[0]) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ax[5] = (2 * targetFootstep[0] * std::pow(t, 5) - ddx0 * std::pow(t, 4) * std::pow(d, 3) - 10 * targetFootstep[0] * std::pow(t, 4) * d + 2 * ddx0 * std::pow(t, 3) * std::pow(d, 4) + 8 * dx0 * std::pow(t, 3) * std::pow(d, 3) + 20 * targetFootstep[0] * std::pow(t, 3) * std::pow(d, 2) - ddx0 * std::pow(t, 2) * std::pow(d, 5) - 10 * dx0 * std::pow(t, 2) * std::pow(d, 4) - 20 * x0 * std::pow(t, 2) * std::pow(d, 3) + 2 * dx0 * t * std::pow(d, 5) + 10 * x0 * t * std::pow(d, 4) - 2 * x0 * std::pow(d, 5)) / (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));

        Ay[0] = (ddy0 * std::pow(t, 2) - 2 * ddy0 * t * d - 6 * dy0 * t + ddy0 * std::pow(d, 2) + 6 * dy0 * d + 12 * y0 - 12 * targetFootstep[1]) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ay[1] = (30 * t * targetFootstep[1] - 30 * t * y0 - 30 * d * y0 + 30 * d * targetFootstep[1] - 2 * std::pow(t, 3) * ddy0 - 3 * std::pow(d, 3) * ddy0 + 14 * std::pow(t, 2) * dy0 - 16 * std::pow(d, 2) * dy0 + 2 * t * d * dy0 + 4 * t * std::pow(d, 2) * ddy0 + std::pow(t, 2) * d * ddy0) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ay[2] = (std::pow(t, 4) * ddy0 + 3 * std::pow(d, 4) * ddy0 - 8 * std::pow(t, 3) * dy0 + 12 * std::pow(d, 3) * dy0 + 20 * std::pow(t, 2) * y0 - 20 * std::pow(t, 2) * targetFootstep[1] + 20 * std::pow(d, 2) * y0 - 20 * std::pow(d, 2) * targetFootstep[1] + 80 * t * d * y0 - 80 * t * d * targetFootstep[1] + 4 * std::pow(t, 3) * d * ddy0 + 28 * t * std::pow(d, 2) * dy0 - 32 * std::pow(t, 2) * d * dy0 - 8 * std::pow(t, 2) * std::pow(d, 2) * ddy0) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ay[3] = -(std::pow(d, 5) * ddy0 + 4 * t * std::pow(d, 4) * ddy0 + 3 * std::pow(t, 4) * d * ddy0 + 36 * t * std::pow(d, 3) * dy0 - 24 * std::pow(t, 3) * d * dy0 + 60 * t * std::pow(d, 2) * y0 + 60 * std::pow(t, 2) * d * y0 - 60 * t * std::pow(d, 2) * targetFootstep[1] - 60 * std::pow(t, 2) * d * targetFootstep[1] - 8 * std::pow(t, 2) * std::pow(d, 3) * ddy0 - 12 * std::pow(t, 2) * std::pow(d, 2) * dy0) / (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ay[4] = -(2 * std::pow(d, 5) * dy0 - 2 * t * std::pow(d, 5) * ddy0 - 10 * t * std::pow(d, 4) * dy0 + std::pow(t, 2) * std::pow(d, 4) * ddy0 + 4 * std::pow(t, 3) * std::pow(d, 3) * ddy0 - 3 * std::pow(t, 4) * std::pow(d, 2) * ddy0 - 16 * std::pow(t, 2) * std::pow(d, 3) * dy0 + 24 * std::pow(t, 3) * std::pow(d, 2) * dy0 - 60 * std::pow(t, 2) * std::pow(d, 2) * y0 + 60 * std::pow(t, 2) * std::pow(d, 2) * targetFootstep[1]) / (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
        Ay[5] = (2 * targetFootstep[1] * std::pow(t, 5) - ddy0 * std::pow(t, 4) * std::pow(d, 3) - 10 * targetFootstep[1] * std::pow(t, 4) * d + 2 * ddy0 * std::pow(t, 3) * std::pow(d, 4) + 8 * dy0 * std::pow(t, 3) * std::pow(d, 3) + 20 * targetFootstep[1] * std::pow(t, 3) * std::pow(d, 2) - ddy0 * std::pow(t, 2) * std::pow(d, 5) - 10 * dy0 * std::pow(t, 2) * std::pow(d, 4) - 20 * y0 * std::pow(t, 2) * std::pow(d, 3) + 2 * dy0 * t * std::pow(d, 5) + 10 * y0 * t * std::pow(d, 4) - 2 * y0 * std::pow(d, 5)) / (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));

        targetFootstep_[0] = targetFootstep[0];
        targetFootstep_[1] = targetFootstep[1];
    }

    // Coefficients for z (deterministic)
    Vector4 Az;
    Az[0] = -maxHeight_ / (std::pow((d / 2), 3) * std::pow((d - d / 2), 3));
    Az[1] = (3 * d * maxHeight_) / (std::pow((d / 2), 3) * std::pow((d - d / 2), 3));
    Az[2] = -(3 * std::pow(d, 2) * maxHeight_) / (std::pow((d / 2), 3) * std::pow((d - d / 2), 3));
    Az[3] = (std::pow(d, 3) * maxHeight_) / (std::pow((d / 2), 3) * std::pow((d - d / 2), 3));

    // Get the next point
    double ev = t + dt;

    if (t < 0.0 || t > d)  // Just vertical motion
    {
        position_(0) = x0;
        position_(1) = y0;
        velocity_(0) = 0.0;
        velocity_(1) = 0.0;
        acceleration_(0) = 0.0;
        acceleration_(1) = 0.0;
    }
    else
    {
        position_(0) = Ax[5] + Ax[4] * ev + Ax[3] * std::pow(ev, 2) + Ax[2] * std::pow(ev, 3) + Ax[1] * std::pow(ev, 4) + Ax[0] * std::pow(ev, 5);
        position_(1) = Ay[5] + Ay[4] * ev + Ay[3] * std::pow(ev, 2) + Ay[2] * std::pow(ev, 3) + Ay[1] * std::pow(ev, 4) + Ay[0] * std::pow(ev, 5);
        velocity_(0) = Ax[4] + 2 * Ax[3] * ev + 3 * Ax[2] * std::pow(ev, 2) + 4 * Ax[1] * std::pow(ev, 3) + 5 * Ax[0] * std::pow(ev, 4);
        velocity_(1) = Ay[4] + 2 * Ay[3] * ev + 3 * Ay[2] * std::pow(ev, 2) + 4 * Ay[1] * std::pow(ev, 3) + 5 * Ay[0] * std::pow(ev, 4);
        acceleration_(0) = 2 * Ax[3] + 3 * 2 * Ax[2] * ev + 4 * 3 * Ax[1] * std::pow(ev, 2) + 5 * 4 * Ax[0] * std::pow(ev, 3);
        acceleration_(1) = 2 * Ay[3] + 3 * 2 * Ay[2] * ev + 4 * 3 * Ay[1] * std::pow(ev, 2) + 5 * 4 * Ay[0] * std::pow(ev, 3);
    }
    velocity_(2) = 3 * Az[3] * std::pow(ev, 2) + 4 * Az[2] * std::pow(ev, 3) + 5 * Az[1] * std::pow(ev, 4) + 6 * Az[0] * std::pow(ev, 5);
    acceleration_(2) = 2 * 3 * Az[3] * ev + 3 * 4 * Az[2] * std::pow(ev, 2) + 4 * 5 * Az[1] * std::pow(ev, 3) + 5 * 6 * Az[0] * std::pow(ev, 4);
    position_(2) = Az[3] * std::pow(ev, 3) + Az[2] * std::pow(ev, 4) + Az[1] * std::pow(ev, 5) + Az[0] * std::pow(ev, 6);
}
