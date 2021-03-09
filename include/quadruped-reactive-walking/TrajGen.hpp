///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for FootTrajectoryGenerator class
///
/// \details This class generates a reference trajectory for the swing foot, in position, velocity
///           and acceleration
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRAJGEN_H_INCLUDED
#define TRAJGEN_H_INCLUDED

#include "quadruped-reactive-walking/Types.h"

// class FootTrajectoryGenerator
// {
// public:
//     ////////////////////////////////////////////////////////////////////////////////////////////////
//     ///
//     /// \brief Default constructor
//     ///
//     /// \param[in] maxHeightIn Apex height of the swinging trajectory
//     /// \param[in] lockTimeIn Target lock before the touchdown
//     /// \param[in] target desired target location at the end of the swing phase
//     ///
//     ////////////////////////////////////////////////////////////////////////////////////////////////
//     FootTrajectoryGenerator(double maxHeightIn, double lockTimeIn, Vector3 targetPosition);

//     ////////////////////////////////////////////////////////////////////////////////////////////////
//     ///
//     /// \brief Destructor.
//     ///
//     ////////////////////////////////////////////////////////////////////////////////////////////////
//     ~FootTrajectoryGenerator();  // Empty constructor

//     ////////////////////////////////////////////////////////////////////////////////////////////////
//     ///
//     /// \brief updates the nex foot position, velocity and acceleration, and the foot goal position
//     ///
//     /// \param[in] x current position of the foot
//     /// \param[in] v current velocity of the foot
//     /// \param[in] a current acceleration of the foot
//     /// \param[in] target desired target location at the end of the swing phase
//     /// \param[in] t time elapsed since the start of the swing phase
//     /// \param[in] duration duration of the swing phase
//     /// \param[in] dt time step of the control
//     ///
//     ////////////////////////////////////////////////////////////////////////////////////////////////
//     void updateFootPosition(Vector3 const& position,
//                             Vector3 const& velocity,
//                             Vector3 const& acceleration,
//                             Vector3 const& targetPosition,
//                             double const t,
//                             double const duration,
//                             double const dt);

//     Vector3 getTargetPosition();    ///< Get the foot goal position
//     Vector3 getFootPosition();      ///< Get the next foot position if updated
//     Vector3 getFootVelocity();      ///< Get the next foot velocity if updated
//     Vector3 getFootAcceleration();  ///< Get the next foot acceleration if updated

// private:
//     double maxHeight_;  // Apex height of the swinging trajectory
//     double lockTime_;   // Target lock before the touchdown

//     Vector3 targetPosition_;  // desired target location at the end of the swing phase

//     Vector6 lastAx_;  // Last X coefficients
//     Vector6 lastAy_;  // Last Y coefficients
//     Vector4 lastAz_;  // Last Z coefficients

//     Vector3 nextFootPosition_;      // next Foot position computed in updateFootPosition
//     Vector3 nextFootVelocity_;      // next Foot velocity computed in updateFootPosition
//     Vector3 nextFootAcceleration_;  // next Foot acceleration computed in updateFootPosition

// }

class TrajGen
{
    /* Class that generates a reference trajectory in position, velocity and acceleration that feet it swing phase
     should follow */
public:
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Default constructor
    ///
    /// \param[in] maxHeightIn Apex height of the swinging trajectory
    /// \param[in] lockTimeIn Target lock before the touchdown
    /// \param[in] target desired target location at the end of the swing phase
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    TrajGen(double const maxHeightIn, double const lockTimeIn, Vector3 const& targetFootstepIn);  // Default constructor

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Destructor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ~TrajGen() {}  // Empty constructor

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief updates the nex foot position, velocity and acceleration, and the foot goal position
    ///
    /// \param[in] a current acceleration of the foot
    /// \param[in] v current velocity of the foot
    /// \param[in] x current position of the foot
    /// \param[in] target desired target location at the end of the swing phase
    /// \param[in] t time elapsed since the start of the swing phase
    /// \param[in] d duration of the swing phase
    /// \param[in] dt time step of the control
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    Vector11 get_next_foot(Vector3 const& position,
                           Vector3 const& velocity,
                           Vector3 const& acceleration,
                           Vector3 const& targetFootstep,
                           double const t,
                           double const d,
                           double const dt);

    Vector3 getTargetPosition() { return targetFootstep_; }  ///< Get the foot goal position
    Vector3 getFootPosition() { return position_; }          ///< Get the next foot position
    Vector3 getFootVelocity() { return velocity_; }          ///< Get the next foot velocity
    Vector3 getFootAcceleration() { return acceleration_; }  ///< Get the next foot acceleration

private:
    Vector6 Ax;  ///< Coefficients for the X component
    Vector6 Ay;  ///< Coefficients for the Y component

    double maxHeight_;  ///< Apex height of the swinging trajectory
    double lockTime_;   ///< Target lock before the touchdown

    Vector3 targetFootstep_;  // Target for the X component

    Vector3 position_;      // position computed in updateFootPosition
    Vector3 velocity_;      // velocity computed in updateFootPosition
    Vector3 acceleration_;  // acceleration computed in updateFootPosition
};
#endif  // PLANNER_H_INCLUDED
