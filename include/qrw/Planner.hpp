///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for Planner class
///
/// \details Planner that outputs current and future locations of footsteps, the reference
///          trajectory of the base and the position, velocity, acceleration commands for feet in
///          swing phase based on the reference velocity given by the user and the current
///          position/velocity of the base
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "pinocchio/math/rpy.hpp"
#include "qrw/FootTrajectoryGenerator.hpp"
#include "qrw/Gait.hpp"
#include "qrw/Types.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#define N0_gait 20
// Number of rows in the gait matrix. Arbitrary value that should be set high enough so that there is always at
// least one empty line at the end of the gait matrix

// Order of feet/legs: FL, FR, HL, HR

class Planner
{
public:
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Empty constructor
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    Planner();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Default constructor
    ///
    /// \param[in] dt_in
    /// \param[in] dt_tsid_in
    /// \param[in] T_gait_in
    /// \param[in] T_mpc_in
    /// \param[in] k_mpc_in
    /// \param[in] h_ref_in
    /// \param[in] fsteps_in
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    Planner(double dt_in,
            double dt_tsid_in,
            double T_gait_in,
            double T_mpc_in,
            int k_mpc_in,
            double h_ref_in,
            Matrix34 const& intialFootsteps,
            Matrix34 const& shouldersIn);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Destructor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ~Planner() {}


    void Print();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Compute a X by 13 matrix containing the remaining number of steps of each phase of the gait (first column)
    ///        and the [x, y, z]^T desired position of each foot for each phase of the gait (12 other columns).
    ///        For feet currently touching the ground the desired position is where they currently are.
    ///
    /// \param[in] q current position vector of the flying base in world frame(linear and angular stacked)
    /// \param[in] v current velocity vector of sthe flying base in world frame(linear and angular stacked)
    /// \param[in] vref desired velocity vector of the flying base in world frame(linear and angular stacked)
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void compute_footsteps(VectorN const& q, Vector6 const& v, Vector6 const& vref);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Compute the target location on the ground of a given foot for an upcoming stance phase
    ///
    /// \param[in] i considered phase (row of the gait matrix)
    /// \param[in] j considered foot (col of the gait matrix)
    ///
    /// \retval Matrix with the next footstep positions
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void compute_next_footstep(int i, int j);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Compute the reference trajectory of the CoM for each time step of the
    ///        predition horizon. The ouput is a matrix of size 12 by (N+1) with N the number
    ///        of time steps in the gait cycle (T_gait/dt) and 12 the position, orientation,
    ///        linear velocity and angular velocity vertically stacked. The first column contains
    ///        the current state while the remaining N columns contains the desired future states.
    ///
    /// \param[in] q current position vector of the flying base in world frame (linear and angular stacked)
    /// \param[in] v current velocity vector of the flying base in world frame (linear and angular stacked)
    /// \param[in] vref desired velocity vector of the flying base in world frame (linear and angular stacked)
    /// \param[in] z_average average height of feet currently in stance phase
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    int getRefStates(VectorN const& q, Vector6 const& v, Vector6 const& vref, double z_average);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Update desired location of footsteps using information coming from the footsteps planner
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void update_target_footsteps();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Update the 3D desired position for feet in swing phase by using a 5-th order polynomial that lead them
    ///        to the desired position on the ground (computed by the footstep planner)
    ///
    /// \param[in] k (int): number of time steps since the start of the simulation
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void update(int k);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Run the planner for one iteration of the main control loop
    ///
    ///  \param[in] k  number of time steps since the start of the simulation
    ///  \param[in] q  current position vector of the flying base in world frame (linear and angular stacked)
    ///  \param[in] v  current velocity vector of the flying base in world frame (linear and angular stacked)
    ///  \param[in] b_vref  desired velocity vector of the flying base in base frame (linear and angular stacked)
    ///  \param[in] h_estim  estimated height of the base
    ///  \param[in] z_average  average height of feet currently in stance phase
    ///  \param[in] joystick_code  integer to trigger events with the joystick
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void run_planner(int const k,
                        VectorN const& q,
                        Vector6 const& v,
                        Vector6 const& b_vref,
                        double const z_average,
                        int const joystickCode);

    // Accessors (to retrieve C data from Python)
    MatrixN get_xref();
    MatrixN get_fsteps();
    MatrixN get_gait();
    Matrix3N get_goals();
    Matrix3N get_vgoals();
    Matrix3N get_agoals();

private:
    MatrixN vectorToMatrix(std::array<Matrix34, N0_gait> const& array);

    // Inputs of the constructor
    double dt;      // Time step of the contact sequence (time step of the MPC)
    double T_gait;  // Gait period
    double T_mpc;   // MPC period (prediction horizon)
    double h_ref;   // Reference height for the trunk
    int k_mpc;      // Number of TSID iterations for one iteration of the MPC

    // Predefined quantities
    double k_feedback;  // Feedback gain for the feedback term of the planner
    double g;           // Value of the gravity acceleartion
    double L;           // Value of the maximum allowed deviation due to leg length

    // Number of time steps in the prediction horizon
    int n_steps;  // T_mpc / time step of the MPC

    // Constant sized matrices
    Matrix34 shoulders_;        // Position of shoulders in local frame
    Matrix34 currentFootstep_;  // Feet matrix in world frame
    Matrix34 nextFootstep_;     // Feet matrix in world frame
    Matrix34 targetFootstep_;
    std::array<Matrix34, N0_gait> footsteps_;

    Matrix3 Rz;  // Predefined matrices for compute_footstep function
    VectorN dt_cum;
    VectorN yaws;
    VectorN dx;
    VectorN dy;

    Vector3 q_tmp;
    Vector3 q_dxdy;
    Vector3 RPY;
    Vector3 b_v;
    Vector6 b_vref;

    Gait gait_;  // Gait object to hold the gait informations

    // Reference trajectory matrix of size 12 by (1 + N)  with the current state of
    // the robot in column 0 and the N steps of the prediction horizon in the others
    MatrixN xref;

    FootTrajectoryGenerator trajGen_;
};

#endif  // PLANNER_H_INCLUDED
