#include "quadruped-reactive-walking/FootTrajectoryGenerator.h"

FootTrajectoryGenerator::FootTrajectoryGenerator(double maxHeightIn, double lockTimeIn, Vector3 targetPosition)
    : maxHeight_(maxHeightIn)
    , lockTime_(lockTimeIn)
    , targetPosition_(targetPosition)
    , lastAx_(Vector6::Zero())
    , lastAy_(Vector6::Zero())
    , lastAz_(Vector4::Zero())
    , nextFootPosition_(Vector3::Zero())
    , nextFootVelocity_(Vector3::Zero())
    , nextFootAcceleration_(Vector3::Zero())
{
    // Empty
}

void FootTrajectoryGenerator::updateFootPosition(Vector3 const& position,
                                                 Vector3 const& velocity,
                                                 Vector3 const& acceleration,
                                                 Vector3 const& targetPosition,
                                                 double const t,
                                                 double const d,
                                                 double const dt)
{
    using namespace std;

    Vector6 Ax = lastAx_;
    Vector6 Ay = lastAy_;
    Vector4 Az;

    if (t < d - lockTime_)
    {
        Ax[0] = (2 * targetPosition[0] * pow(t, 5) - acceleration[0] * pow(t, 4) * pow(d, 3) - 10 * targetPosition[0] * pow(t, 4) * d + 2 * acceleration[0] * pow(t, 3) * pow(d, 4) + 8 * velocity[0] * pow(t, 3) * pow(d, 3) + 20 * targetPosition[0] * pow(t, 3) * pow(d, 2) - acceleration[0] * pow(t, 2) * pow(d, 5) - 10 * velocity[0] * pow(t, 2) * pow(d, 4) - 20 * position[0] * pow(t, 2) * pow(d, 3) + 2 * velocity[0] * t * pow(d, 5) + 10 * position[0] * t * pow(d, 4) - 2 * position[0] * pow(d, 5)) / (2 * (pow(t, 2) - 2 * t * d + pow(d, 2)) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ax[1] = -(2 * pow(d, 5) * velocity[0] - 2 * t * pow(d, 5) * acceleration[0] - 10 * t * pow(d, 4) * velocity[0] + pow(t, 2) * pow(d, 4) * acceleration[0] + 4 * pow(t, 3) * pow(d, 3) * acceleration[0] - 3 * pow(t, 4) * pow(d, 2) * acceleration[0] - 16 * pow(t, 2) * pow(d, 3) * velocity[0] + 24 * pow(t, 3) * pow(d, 2) * velocity[0] - 60 * pow(t, 2) * pow(d, 2) * position[0] + 60 * pow(t, 2) * pow(d, 2) * targetPosition[0]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ax[2] = -(pow(d, 5) * acceleration[0] + 4 * t * pow(d, 4) * acceleration[0] + 3 * pow(t, 4) * d * acceleration[0] + 36 * t * pow(d, 3) * velocity[0] - 24 * pow(t, 3) * d * velocity[0] + 60 * t * pow(d, 2) * position[0] + 60 * pow(t, 2) * d * position[0] - 60 * t * pow(d, 2) * targetPosition[0] - 60 * pow(t, 2) * d * targetPosition[0] - 8 * pow(t, 2) * pow(d, 3) * acceleration[0] - 12 * pow(t, 2) * pow(d, 2) * velocity[0]) / (2 * (pow(t, 2) - 2 * t * d + pow(d, 2)) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ax[3] = (pow(t, 4) * acceleration[0] + 3 * pow(d, 4) * acceleration[0] - 8 * pow(t, 3) * velocity[0] + 12 * pow(d, 3) * velocity[0] + 20 * pow(t, 2) * position[0] - 20 * pow(t, 2) * targetPosition[0] + 20 * pow(d, 2) * position[0] - 20 * pow(d, 2) * targetPosition[0] + 80 * t * d * position[0] - 80 * t * d * targetPosition[0] + 4 * pow(t, 3) * d * acceleration[0] + 28 * t * pow(d, 2) * velocity[0] - 32 * pow(t, 2) * d * velocity[0] - 8 * pow(t, 2) * pow(d, 2) * acceleration[0]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ax[4] = (30 * t * targetPosition[0] - 30 * t * position[0] - 30 * d * position[0] + 30 * d * targetPosition[0] - 2 * pow(t, 3) * acceleration[0] - 3 * pow(d, 3) * acceleration[0] + 14 * pow(t, 2) * velocity[0] - 16 * pow(d, 2) * velocity[0] + 2 * t * d * velocity[0] + 4 * t * pow(d, 2) * acceleration[0] + pow(t, 2) * d * acceleration[0]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ax[5] = (acceleration[0] * pow(t, 2) - 2 * acceleration[0] * t * d - 6 * velocity[0] * t + acceleration[0] * pow(d, 2) + 6 * velocity[0] * d + 12 * position[0] - 12 * targetPosition[0]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));

        Ay[0] = (2 * targetPosition[1] * pow(t, 5) - acceleration[1] * pow(t, 4) * pow(d, 3) - 10 * targetPosition[1] * pow(t, 4) * d + 2 * acceleration[1] * pow(t, 3) * pow(d, 4) + 8 * velocity[1] * pow(t, 3) * pow(d, 3) + 20 * targetPosition[1] * pow(t, 3) * pow(d, 2) - acceleration[1] * pow(t, 2) * pow(d, 5) - 10 * velocity[1] * pow(t, 2) * pow(d, 4) - 20 * position[1] * pow(t, 2) * pow(d, 3) + 2 * velocity[1] * t * pow(d, 5) + 10 * position[1] * t * pow(d, 4) - 2 * position[1] * pow(d, 5)) / (2 * (pow(t, 2) - 2 * t * d + pow(d, 2)) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ay[1] = -(2 * pow(d, 5) * velocity[1] - 2 * t * pow(d, 5) * acceleration[1] - 10 * t * pow(d, 4) * velocity[1] + pow(t, 2) * pow(d, 4) * acceleration[1] + 4 * pow(t, 3) * pow(d, 3) * acceleration[1] - 3 * pow(t, 4) * pow(d, 2) * acceleration[1] - 16 * pow(t, 2) * pow(d, 3) * velocity[1] + 24 * pow(t, 3) * pow(d, 2) * velocity[1] - 60 * pow(t, 2) * pow(d, 2) * position[1] + 60 * pow(t, 2) * pow(d, 2) * targetPosition[1]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ay[2] = -(pow(d, 5) * acceleration[1] + 4 * t * pow(d, 4) * acceleration[1] + 3 * pow(t, 4) * d * acceleration[1] + 36 * t * pow(d, 3) * velocity[1] - 24 * pow(t, 3) * d * velocity[1] + 60 * t * pow(d, 2) * position[1] + 60 * pow(t, 2) * d * position[1] - 60 * t * pow(d, 2) * targetPosition[1] - 60 * pow(t, 2) * d * targetPosition[1] - 8 * pow(t, 2) * pow(d, 3) * acceleration[1] - 12 * pow(t, 2) * pow(d, 2) * velocity[1]) / (2 * (pow(t, 2) - 2 * t * d + pow(d, 2)) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ay[3] = (pow(t, 4) * acceleration[1] + 3 * pow(d, 4) * acceleration[1] - 8 * pow(t, 3) * velocity[1] + 12 * pow(d, 3) * velocity[1] + 20 * pow(t, 2) * position[1] - 20 * pow(t, 2) * targetPosition[1] + 20 * pow(d, 2) * position[1] - 20 * pow(d, 2) * targetPosition[1] + 80 * t * d * position[1] - 80 * t * d * targetPosition[1] + 4 * pow(t, 3) * d * acceleration[1] + 28 * t * pow(d, 2) * velocity[1] - 32 * pow(t, 2) * d * velocity[1] - 8 * pow(t, 2) * pow(d, 2) * acceleration[1]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ay[4] = (30 * t * targetPosition[1] - 30 * t * position[1] - 30 * d * position[1] + 30 * d * targetPosition[1] - 2 * pow(t, 3) * acceleration[1] - 3 * pow(d, 3) * acceleration[1] + 14 * pow(t, 2) * velocity[1] - 16 * pow(d, 2) * velocity[1] + 2 * t * d * velocity[1] + 4 * t * pow(d, 2) * acceleration[1] + pow(t, 2) * d * acceleration[1]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));
        Ay[5] = (acceleration[1] * pow(t, 2) - 2 * acceleration[1] * t * d - 6 * velocity[1] * t + acceleration[1] * pow(d, 2) + 6 * velocity[1] * d + 12 * position[1] - 12 * targetPosition[1]) / (2 * pow((t - d), 2) * (pow(t, 3) - 3 * pow(t, 2) * d + 3 * t * pow(d, 2) - pow(d, 3)));

        targetPosition_ = targetPosition;
    }

    // Coefficients for z (deterministic)
    Az[0] = (pow(d, 3) * maxHeight_) / (pow((d / 2), 3) * pow((d - d / 2), 3));
    Az[1] = -(3 * pow(d, 2) * maxHeight_) / (pow((d / 2), 3) * pow((d - d / 2), 3));
    Az[2] = (3 * d * maxHeight_) / (pow((d / 2), 3) * pow((d - d / 2), 3));
    Az[3] = -maxHeight_ / (pow((d / 2), 3) * pow((d - d / 2), 3));

    lastAx_ = Ax;
    lastAy_ = Ay;
    lastAz_ = Az;

    // Get the next point
    double ev = t + dt;

    nextFootPosition_[0] = Ax[0] + Ax[1] * ev + Ax[2] * pow(ev, 2) + Ax[3] * pow(ev, 3) + Ax[4] * pow(ev, 4) + Ax[5] * pow(ev, 5);
    nextFootPosition_[1] = Ay[0] + Ay[1] * ev + Ay[2] * pow(ev, 2) + Ay[3] * pow(ev, 3) + Ay[4] * pow(ev, 4) + Ay[5] * pow(ev, 5);
    nextFootPosition_[2] = Az[0] * pow(ev, 3) + Az[1] * pow(ev, 4) + Az[2] * pow(ev, 5) + Az[3] * pow(ev, 6);

    nextFootVelocity_[0] = Ax[1] + 2 * Ax[2] * ev + 3 * Ax[3] * pow(ev, 2) + 4 * Ax[4] * pow(ev, 3) + 5 * Ax[5] * pow(ev, 4);
    nextFootVelocity_[1] = Ay[1] + 2 * Ay[2] * ev + 3 * Ay[3] * pow(ev, 2) + 4 * Ay[4] * pow(ev, 3) + 5 * Ay[5] * pow(ev, 4);
    nextFootVelocity_[2] = 3 * Az[0] * pow(ev, 2) + 4 * Az[1] * pow(ev, 3) + 5 * Az[2] * pow(ev, 4) + 6 * Az[3] * pow(ev, 5);

    nextFootAcceleration_[0] = 2 * Ax[2] + 3 * 2 * Ax[3] * ev + 4 * 3 * Ax[4] * pow(ev, 2) + 5 * 4 * Ax[5] * pow(ev, 3);
    nextFootAcceleration_[1] = 2 * Ay[2] + 3 * 2 * Ay[3] * ev + 4 * 3 * Ay[4] * pow(ev, 2) + 5 * 4 * Ay[5] * pow(ev, 3);
    nextFootAcceleration_[2] = 2 * 3 * Az[0] * ev + 3 * 4 * Az[1] * pow(ev, 2) + 4 * 5 * Az[2] * pow(ev, 3) + 5 * 6 * Az[3] * pow(ev, 4);
}