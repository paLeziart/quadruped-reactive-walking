///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for Gait class
///
/// \details Planner that outputs current and future locations of footsteps, the reference
///          trajectory of the base and the position, velocity, acceleration commands for feet in
///          swing phase based on the reference velocity given by the user and the current
///          position/velocity of the base
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef GAIT_H_INCLUDED
#define GAIT_H_INCLUDED

#include "qrw/Types.h"

// Order of feet/legs: FL, FR, HL, HR

class Gait
{
public:
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Empty constructor
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    Gait();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Destructor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ~Gait() {}

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Initializer
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void initialize(double dt_in, double T_gait_in, double T_mpc_in, int N_gait);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Compute the remaining and total duration of a swing phase or a stance phase based
    ///        on the content of the gait matrix
    ///
    /// \param[in] i considered phase (row of the gait matrix)
    /// \param[in] j considered foot (col of the gait matrix)
    /// \param[in] value 0.0 for swing phase detection, 1.0 for stance phase detection
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    double getPhaseDuration(int i, int j, double value);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Handle the joystick code to trigger events (change of gait for instance)
    ///
    /// \param[in] code integer to trigger events with the joystick
    /// \param[in] q current position vector of the flying base in world frame (linear and angular stacked)
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool changeGait(int const code, VectorN const& q);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief  Move one step further in the gait cycle
    ///
    /// \details Decrease by 1 the number of remaining step for the current phase of the gait
    ///           Transfer current gait phase into past gait matrix
    ///           Insert future desired gait phase at the end of the gait matrix
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void updateGait(int const k, int const k_mpc, VectorN const& q, int const joystickCode);

    // TODO
    void rollGait();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Update the gait matrix externally (directly set the gait matrix)
    ///
    ///  \param[in] gaitMatrix  gait matrix that should be used for the incoming timesteps
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool setGait(MatrixN const& gaitMatrix);

    MatrixN getPastGait() { return pastGait_; }
    MatrixN getCurrentGait() { return currentGait_; }
    double getCurrentGaitCoeff(int i, int j) { return currentGait_(i, j); }
    MatrixN getDesiredGait() { return desiredGait_; }
    double getRemainingTime() { return remainingTime_; }
    bool getIsStatic() { return is_static_; }
    VectorN getQStatic() { return q_static_; }
    bool isNewPhase() { return newPhase_; }

private:
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief  Create a slow walking gait, raising and moving only one foot at a time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_walk();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a trot gait with diagonaly opposed legs moving at the same time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_trot();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a pacing gait with legs on the same side (left or right) moving at the same time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_pacing();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a bounding gait with legs on the same side (front or hind) moving at the same time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_bounding();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a static gait with all legs in stance phase
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_static();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Initialize content of the gait matrix based on the desired gait, the gait period and
    ///        the length of the prediciton horizon
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_gait_f();

    MatrixN pastGait_;     // Past gait
    MatrixN currentGait_;  // Current and future gait
    MatrixN desiredGait_;  // Future desired gait

    double dt_;      // Time step of the contact sequence (time step of the MPC)
    double T_gait_;  // Gait period
    double T_mpc_;   // MPC period (prediction horizon)
    int n_steps_;        // Number of time steps in the prediction horizon

    double remainingTime_;

    bool newPhase_;
    bool is_static_;
    VectorN q_static_;
};

#endif  // GAIT_H_INCLUDED
