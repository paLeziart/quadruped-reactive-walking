#include "qrw/Planner.hpp"

Planner::Planner()
    : dt(0.0)
    , dt_tsid(0.0)
    , T_gait(0.0)
    , T_mpc(0.0)
    , h_ref(0.0)
    , k_mpc(0)
    , on_solo8(false)
    , k_feedback(0.03)
    , g(9.81)
    , L(0.155)
    , is_static(false)
    , n_steps(0)
    , feet()
    , t0s()
    , t_swing({0.0, 0.0, 0.0, 0.0})
    , fsteps(MatrixN::Zero(N0_gait, 13))
    , shoulders(Matrix34::Zero())
    , q_static(Vector19::Zero())
    , RPY_static(Vector3::Zero())
    , o_feet_contact(Vector12::Zero())
    , next_footstep(Matrix34::Zero())
    , R(Matrix3::Zero())
    , R_1(Matrix3::Zero())
    , R_2(Matrix3::Zero())
    , dt_cum(VectorN::Zero(N0_gait))
    , angle(VectorN::Zero(N0_gait))
    , dx(VectorN::Zero(N0_gait))
    , dy(VectorN::Zero(N0_gait))
    , q_tmp(Vector3::Zero())
    , q_dxdy(Vector3::Zero())
    , RPY(Vector3::Zero())
    , b_v_cur(Vector3::Zero())
    , b_v_ref(Vector6::Zero())
    , cross(Vector3::Zero())
    , vref_in(Vector6::Zero())
    , gait_()
    , xref()
    , maxHeight_(0.05)
    , lockTime_(0.07)
    , trajGens_()
    , targetFootstep_()
    , nextFootPosition_()
    , nextFootVelocity_()
    , nextFootAcceleration_()
{
    shoulders << 0.1946, 0.1946, -0.1946, -0.1946, 0.14695, -0.14695, 0.14695, -0.14695, 0.0, 0.0, 0.0, 0.0;

    // By default contacts are at the vertical of shoulders
    Eigen::Map<Vector12> v1(shoulders.data(), shoulders.size());
    o_feet_contact << v1;

    R(2, 2) = 1.0;
    R_1(2, 2) = 1.0;
}


Planner::Planner(double dt_in, double dt_tsid_in, double T_gait_in, double T_mpc_in, int k_mpc_in, bool on_solo8_in,
                 double h_ref_in, const MatrixN& fsteps_in)
    : Planner()
{
    // Parameters from the main controller
    dt = dt_in;
    dt_tsid = dt_tsid_in;
    T_gait = T_gait_in;
    T_mpc = T_mpc_in;
    k_mpc = k_mpc_in;
    on_solo8 = on_solo8_in;
    h_ref = h_ref_in;

    // Predefining quantities
    n_steps = (int)std::lround(T_mpc / dt);

    // Initialize xref matrix
    xref = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(12, 1 + n_steps);

    gait_.initialize(dt, T_gait, T_mpc, fsteps);

    // One foot trajectory generator per leg
    for (int i = 0; i < 4; i++)
    {
        nextFootPosition_.push_back(fsteps_in.col(i));
        nextFootVelocity_.push_back(Vector3::Zero());
        nextFootAcceleration_.push_back(Vector3::Zero());

        targetFootstep_.push_back({shoulders(0, i), shoulders(1, i), 0.0});
        trajGens_.push_back(FootTrajectoryGenerator(maxHeight_, lockTime_, targetFootstep_[i]));
    }
}


void Planner::Print()
{
    /* To print stuff for visualisation or debug */

    std::cout << "------" << std::endl;
    std::cout << gait_.getPastGait().block(0, 0, 6, 5) << std::endl;
    std::cout << "-" << std::endl;
    std::cout << gait_.getCurrentGait().block(0, 0, 6, 5) << std::endl;
    std::cout << "-" << std::endl;
    std::cout << gait_.getDesiredGait().block(0, 0, 6, 5) << std::endl;
}

int Planner::compute_footsteps(MatrixN q_cur, MatrixN v_cur, MatrixN v_ref)
{
    /* Compute a X by 13 matrix containing the remaining number of steps of each phase of the gait (first column)
  and the [x, y, z]^T desired position of each foot for each phase of the gait (12 other columns).
  For feet currently touching the ground the desired position is where they currently are.

  Args:
    q_cur (7x1 array): current position vector of the flying base in world frame (linear and angular stacked)
    v_cur (6x1 array): current velocity vector of the flying base in world frame (linear and angular stacked)
    v_ref (6x1 array): desired velocity vector of the flying base in world frame (linear and angular stacked)
  */

    fsteps = Eigen::Matrix<double, N0_gait, 13>::Zero();
    fsteps.col(0) = gait_.getCurrentGait().col(0);

    MatrixN gait = gait_.getCurrentGait();

    // Set current position of feet for feet in stance phase
    for (int j = 0; j < 4; j++)
    {
        if (gait(0, 1 + j) == 1.0)
        {
            fsteps.block(0, 1 + 3 * j, 1, 3) = o_feet_contact.segment<3>(3 * j);
        }
    }

    // Cumulative time by adding the terms in the first column (remaining number of timesteps)
    // Get future yaw angle compared to current position
    dt_cum(0, 0) = gait(0, 0) * dt;
    angle(0, 0) = v_ref(5, 0) * dt_cum(0, 0) + RPY(2, 0);
    for (int j = 1; j < N0_gait; j++)
    {
        dt_cum(j, 0) = dt_cum(j - 1, 0) + gait(j, 0) * dt;
        angle(j, 0) = v_ref(5, 0) * dt_cum(j, 0) + RPY(2, 0);
    }

    // Displacement following the reference velocity compared to current position
    if (v_ref(5, 0) != 0)
    {
        for (int j = 0; j < N0_gait; j++)
        {
            dx(j, 0) = (v_cur(0, 0) * std::sin(v_ref(5, 0) * dt_cum(j, 0)) + v_cur(1, 0) * (std::cos(v_ref(5, 0) * dt_cum(j, 0)) - 1.0)) / v_ref(5, 0);
            dy(j, 0) = (v_cur(1, 0) * std::sin(v_ref(5, 0) * dt_cum(j, 0)) - v_cur(0, 0) * (std::cos(v_ref(5, 0) * dt_cum(j, 0)) - 1.0)) / v_ref(5, 0);
        }
    }
    else
    {
        for (int j = 0; j < N0_gait; j++)
        {
            dx(j, 0) = v_cur(0, 0) * dt_cum(j, 0);
            dy(j, 0) = v_cur(1, 0) * dt_cum(j, 0);
        }
    }

    // Get current and reference velocities in base frame (rotated yaw)
    double c = std::cos(RPY(2, 0));
    double s = std::sin(RPY(2, 0));
    R_1.block(0, 0, 2, 2) << c, s, -s, c;  // already transposed here
    b_v_cur = R_1 * v_cur.block(0, 0, 3, 1);
    b_v_ref.block(0, 0, 3, 1) = R_1 * v_ref.block(0, 0, 3, 1);
    b_v_ref.block(3, 0, 3, 1) = R_1 * v_ref.block(3, 0, 3, 1);

    // Update the footstep matrix depending on the different phases of the gait (swing & stance)
    int i = 1;
    while (gait(i, 0) != 0)
    {
        // Feet that were in stance phase and are still in stance phase do not move
        for (int j = 0; j < 4; j++)
        {
            if (gait(i - 1, 1 + j) * gait(i, 1 + j) > 0)
            {
                fsteps.block(i, 1 + 3 * j, 1, 3) = fsteps.block(i - 1, 1 + 3 * j, 1, 3);
            }
        }

        // Current position without height
        q_tmp << q_cur(0, 0), q_cur(1, 0), 0.0;

        // Feet that were in swing phase and are now in stance phase need to be updated
        for (int j = 0; j < 4; j++)
        {
            if ((1 - gait(i - 1, 1 + j)) * gait(i, 1 + j) > 0)
            {
                // Offset to the future position
                q_dxdy << dx(i - 1, 0), dy(i - 1, 0), 0.0;

                // Get future desired position of footsteps
                compute_next_footstep(i, j);

                // Get desired position of footstep compared to current position
                double c = std::cos(angle(i - 1, 0));
                double s = std::sin(angle(i - 1, 0));
                R.block(0, 0, 2, 2) << c, -s, s, c;

                fsteps.block(i, 1 + 3 * j, 1, 3) = (R * next_footstep.col(j) + q_tmp + q_dxdy).transpose();
            }
        }

        i++;
    }

    return 0;
}

int Planner::compute_next_footstep(int i, int j)
{
    /* Compute the target location on the ground of a given foot for an upcoming stance phase

  Args:
    i (int): considered phase (row of the gait matrix)
    j (int): considered foot (col of the gait matrix)
  */

    double t_stance = gait_.get_stance_swing_duration(i, j, 1.0);  // 1.0 for stance phase

    // Add symmetry term
    next_footstep.col(j) = t_stance * 0.5 * b_v_cur;

    // Add feedback term
    next_footstep.col(j) += k_feedback * (b_v_cur - b_v_ref.block(0, 0, 3, 1));

    // Add centrifugal term
    cross << b_v_cur(1, 0) * b_v_ref(5, 0) - b_v_cur(2, 0) * b_v_ref(4, 0),
        b_v_cur(2, 0) * b_v_ref(3, 0) - b_v_cur(0, 0) * b_v_ref(5, 0), 0.0;
    next_footstep.col(j) += 0.5 * std::sqrt(h_ref / g) * cross;

    // Legs have a limited length so the deviation has to be limited
    if (next_footstep(0, j) > L)
    {
        next_footstep(0, j) = L;
    }
    else if (next_footstep(0, j) < -L)
    {
        next_footstep(0, j) = -L;
    }

    if (next_footstep(1, j) > L)
    {
        next_footstep(1, j) = L;
    }
    else if (next_footstep(1, j) < -L)
    {
        next_footstep(1, j) = -L;
    }

    // Add shoulders
    next_footstep.col(j) += shoulders.col(j);

    // Remove Z component (working on flat ground)
    next_footstep.row(2) = Eigen::Matrix<double, 1, 4>::Zero();

    return 0;
}

int Planner::getRefStates(MatrixN q, MatrixN v, MatrixN vref, double z_average)
{
    /* Compute the reference trajectory of the CoM for each time step of the
  predition horizon. The ouput is a matrix of size 12 by (N+1) with N the number
  of time steps in the gait cycle (T_gait/dt) and 12 the position, orientation,
  linear velocity and angular velocity vertically stacked. The first column contains
  the current state while the remaining N columns contains the desired future states.

  Args:
    q (7x1 array): current position vector of the flying base in world frame (linear and angular stacked)
    v (6x1 array): current velocity vector of the flying base in world frame (linear and angular stacked)
    vref (6x1 array): desired velocity vector of the flying base in world frame (linear and angular stacked)
    z_average (double): average height of feet currently in stance phase
  */

    VectorN dt_vector = VectorN::LinSpaced(n_steps, dt, T_mpc);

    // Update yaw and yaw velocity
    xref.block(5, 1, 1, n_steps) = vref(5, 0) * dt_vector.transpose();
    for (int i = 0; i < n_steps; i++)
    {
        xref(11, 1 + i) = vref(5, 0);
    }

    // Update x and y velocities taking into account the rotation of the base over the prediction horizon
    for (int i = 0; i < n_steps; i++)
    {
        xref(6, 1 + i) = vref(0, 0) * std::cos(xref(5, 1 + i)) - vref(1, 0) * std::sin(xref(5, 1 + i));
        xref(7, 1 + i) = vref(0, 0) * std::sin(xref(5, 1 + i)) + vref(1, 0) * std::cos(xref(5, 1 + i));
    }

    // Update x and y depending on x and y velocities (cumulative sum)
    if (vref(5, 0) != 0)
    {
        for (int i = 0; i < n_steps; i++)
        {
            xref(0, 1 + i) = (vref(0, 0) * std::sin(vref(5, 0) * dt_vector(i)) + vref(1, 0) * (std::cos(vref(5, 0) * dt_vector(i)) - 1.0)) / vref(5, 0);
            xref(1, 1 + i) = (vref(1, 0) * std::sin(vref(5, 0) * dt_vector(i)) - vref(0, 0) * (std::cos(vref(5, 0) * dt_vector(i)) - 1.0)) / vref(5, 0);
        }
    }
    else
    {
        for (int i = 0; i < n_steps; i++)
        {
            xref(0, 1 + i) = vref(0, 0) * dt_vector(i);
            xref(1, 1 + i) = vref(1, 0) * dt_vector(i);
        }
    }

    for (int i = 0; i < n_steps; i++)
    {
        xref(5, 1 + i) += RPY(2, 0);
        xref(2, 1 + i) = h_ref + z_average;
        xref(8, 1 + i) = 0.0;
    }

    // No need to update Z velocity as the reference is always 0
    // No need to update roll and roll velocity as the reference is always 0 for those
    // No need to update pitch and pitch velocity as the reference is always 0 for those

    // Update the current state
    xref.block(0, 0, 3, 1) = q.block(0, 0, 3, 1);
    xref.block(3, 0, 3, 1) = RPY;
    xref.block(6, 0, 3, 1) = v.block(0, 0, 3, 1);
    xref.block(9, 0, 3, 1) = v.block(3, 0, 3, 1);

    for (int i = 0; i < n_steps; i++)
    {
        xref(0, 1 + i) += xref(0, 0);
        xref(1, 1 + i) += xref(1, 0);
    }

    if (is_static)
    {
        Vector3 RPY;
        Eigen::Quaterniond quat(q_static(6, 0), q_static(3, 0), q_static(4, 0), q_static(5, 0));  // w, x, y, z
        RPY << pinocchio::rpy::matrixToRpy(quat.toRotationMatrix());
        for (int i = 0; i < n_steps; i++)
        {
            xref.block(0, 1 + i, 3, 1) = q_static.block(0, 0, 3, 1);
            xref.block(3, 1 + i, 3, 1) = RPY;
        }
    }

    return 0;
}

void Planner::update_target_footsteps()
{
    for (int i = 0; i < 4; i++)
    {
        // Index of the first non-empty line
        int index = 0;
        while (fsteps(index, 1 + 3 * i) == 0.0)
        {
            index++;
        }
        targetFootstep_[i] << fsteps(index, 1 + 3 * i), fsteps(index, 2 + 3 * i), 0.0;
    }
}

int Planner::update_trajectory_generator(int k, double h_estim)
{
    /* Update the 3D desired position for feet in swing phase by using a 5-th order polynomial that lead them
  to the desired position on the ground (computed by the footstep planner)

  Args:
    k (int): number of time steps since the start of the simulation
    h_estim (double): estimated height of the base
  */

    if ((k % k_mpc) == 0)
    {
        // Indexes of feet in swing phase
        feet.clear();
        for (int i = 0; i < 4; i++)
        {
            if (gait_.getCurrentGait()(0, 1 + i) == 0)
            {
                feet.push_back(i);
            }
        }
        // If no foot in swing phase
        if (feet.size() == 0)
        {
            return 0;
        }

        // For each foot in swing phase get remaining duration of the swing phase
        t0s.clear();
        for (int j = 0; j < (int)feet.size(); j++)
        {
            int i = feet[j];

            t_swing[i] = gait_.get_stance_swing_duration(0, feet[j], 0.0);  // 0.0 for swing phase

            double value = t_swing[i] - (gait_.getRemainingTime() * k_mpc - ((k + 1) % k_mpc)) * dt_tsid - dt_tsid;

            if (value > 0.0)
            {
                t0s.push_back(value);
            }
            else
            {
                t0s.push_back(0.0);
            }
        }
    }
    else
    {
        // If no foot in swing phase
        if (feet.size() == 0)
        {
            return 0;
        }

        // Increment of one time step for feet in swing phase
        for (int i = 0; i < (int)feet.size(); i++)
        {
            double value = t0s[i] + dt_tsid;
            if (value > 0.0)
            {
                t0s[i] = value;
            }
            else
            {
                t0s[i] = 0.0;
            }
        }
    }

    // Get position, velocity and acceleration commands for feet in swing phase
    for (int i = 0; i < (int)feet.size(); i++)
    {
        int i_foot = feet[i];

        trajGens_[i_foot].updateFootPosition(nextFootPosition_[i_foot],
                                             nextFootVelocity_[i_foot],
                                             nextFootAcceleration_[i_foot],
                                             targetFootstep_[i_foot],
                                             t0s[i],
                                             t_swing[i_foot],
                                             dt_tsid);

        nextFootPosition_[i_foot] = trajGens_[i_foot].getFootPosition();
        nextFootVelocity_[i_foot] = trajGens_[i_foot].getFootVelocity();
        nextFootAcceleration_[i_foot] = trajGens_[i_foot].getFootAcceleration();
    }

    return 0;
}

int Planner::run_planner(int k, const MatrixN& q, const MatrixN& v, const MatrixN& b_vref_in,
                         double h_estim, double z_average, int joystick_code)
{
    // Get the reference velocity in world frame (given in base frame)
    Eigen::Quaterniond quat(q(6, 0), q(3, 0), q(4, 0), q(5, 0));  // w, x, y, z
    RPY << pinocchio::rpy::matrixToRpy(quat.toRotationMatrix());
    double c = std::cos(RPY(2, 0));
    double s = std::sin(RPY(2, 0));
    R_2.block(0, 0, 2, 2) << c, -s, s, c;
    R_2(2, 2) = 1.0;
    vref_in.block(0, 0, 3, 1) = R_2 * b_vref_in.block(0, 0, 3, 1);
    vref_in.block(3, 0, 3, 1) = b_vref_in.block(3, 0, 3, 1);

    // Handle joystick events
    is_static = gait_.handle_joystick(joystick_code, q, q_static, fsteps);

    // Move one step further in the gait
    if (k % k_mpc == 0)
    {
        gait_.roll(k, fsteps, o_feet_contact);
    }

    // Compute the desired location of footsteps over the prediction horizon
    compute_footsteps(q, v, vref_in);

    // Get the reference trajectory for the MPC
    getRefStates(q, v, vref_in, z_average);

    // Update desired location of footsteps on the ground
    update_target_footsteps();

    // Update trajectory generator (3D pos, vel, acc)
    update_trajectory_generator(k, h_estim);

    return 0;
}

MatrixN Planner::get_xref() { return xref; }
MatrixN Planner::get_fsteps() { return fsteps; }
MatrixN Planner::get_gait() { return gait_.getCurrentGait(); }
Matrix3N Planner::get_goals() { return vectorToMatrix(nextFootPosition_); }
Matrix3N Planner::get_vgoals() { return vectorToMatrix(nextFootVelocity_); }
Matrix3N Planner::get_agoals() { return vectorToMatrix(nextFootAcceleration_); }

Matrix3N Planner::vectorToMatrix(std::vector<Vector3> const& vector)
{
    Matrix3N M;
    for (int i = 0; i < vector.size(); i++)
    {
        M.conservativeResize(M.rows(), M.cols() + 1);
        M.col(M.cols() - 1) = vector[i];
    }
    return M;
}
