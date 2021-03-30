#include "qrw/gepadd.hpp"
#include "qrw/Gait.hpp"
#include "qrw/InvKin.hpp"
#include "qrw/MPC.hpp"
#include "qrw/Planner.hpp"
#include "qrw/QPWBC.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

template <typename MPC>
struct MPCPythonVisitor : public bp::def_visitor<MPCPythonVisitor<MPC>>
{
    template <class PyClassMPC>
    void visit(PyClassMPC& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def(bp::init<double, int, double>(bp::args("dt_in", "n_steps_in", "T_gait_in"),
                                               "Constructor with parameters."))

            // Run MPC from Python
            .def("run", &MPC::run, bp::args("num_iter", "xref_in", "fsteps_in"), "Run MPC from Python.\n")
            .def("get_latest_result", &MPC::get_latest_result,
                 "Get latest result (predicted trajectory  forces to apply).\n")
            .def("get_gait", &MPC::get_gait, "Get gait matrix.\n")
            .def("get_Sgait", &MPC::get_Sgait, "Get S_gait matrix.\n");
    }

    static void expose()
    {
        bp::class_<MPC>("MPC", bp::no_init).def(MPCPythonVisitor<MPC>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};

void exposeMPC() { MPCPythonVisitor<MPC>::expose(); }

// -------- FOOT TRAJECTORY GENERATOR --------------------------------------------------------------

// template <typename FootTrajectoryGenerator>
// struct FootTrajectoryGeneratorVisitor : public bp::def_visitor<FootTrajectoryGeneratorVisitor<FootTrajectoryGenerator>>
// {
//     template <class PyClassFootTrajectoryGenerator>
//     void visit(PyClassFootTrajectoryGenerator& cl) const
//     {
//         cl.def(bp::init<>(bp::arg(""), "Default constructor."))

//             .def("initialize", &FootTrajectoryGenerator::initialize, bp::args("maxHeight", "lockTime", "targetFootstep", "initialFootPosition", "dt", "k", "gait"),
//                  "Initialize Gait class.\n")

//             .def("update_foot_position", &FootTrajectoryGenerator::updateFootPosition, bp::args("foot_id", "targetFootstep"),
//                  "Compute the next position, velocity and acceleration of the foot.\n")

//             .def("update_foot_position", &FootTrajectoryGenerator::update, bp::args("k", "targetFootstep"),
//                  "Compute the next position, velocity and acceleration of the foot.\n")

//             .def("get_target_footstep", &FootTrajectoryGenerator::getTargetPosition, "Get the target foot position.\n")
//             .def("get_position", &FootTrajectoryGenerator::getFootPosition, "Get computed foot position matrix.\n")
//             .def("get_velocity", &FootTrajectoryGenerator::getFootVelocity, "Get computed foot velocity matrix.\n")
//             .def("get_acceleration", &FootTrajectoryGenerator::getFootAcceleration, "Get computed foot acceleration matrix.\n");
//     }

//     static void expose()
//     {
//         bp::class_<FootTrajectoryGenerator>("FootTrajectoryGenerator", bp::no_init).def(FootTrajectoryGeneratorVisitor<FootTrajectoryGenerator>());

//         ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
//     }
// };
// void exposeFootTrajectoryGenerator() { FootTrajectoryGeneratorVisitor<FootTrajectoryGenerator>::expose(); }

// -------- GAIT -----------------------------------------------------------------------------------

template <typename Gait>
struct GaitVisitor : public bp::def_visitor<GaitVisitor<Gait>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def("initialize", &Gait::initialize, bp::args("dt_in", "T_gait_in", "T_mpc_in"),
                 "Initialize Gait class.\n")

            .def("get_phase_duration", &Gait::getPhaseDuration, bp::args("i", "j", "value"),
                 "Get the duration of the swing or stance phase for a given foot")

            .def("roll", &Gait::roll, bp::args("k", "footstep", "currentFootstep"),
                 "Get the duration of the swing or stance phase for a given foot")

            .def("change_gait", &Gait::changeGait, bp::args("code", "q"),
                 "Use the code to change the gait")

            .def("get_current_gait", &Gait::getCurrentGait, "Get the gait.\n")
            .def("get_desired_gait", &Gait::getDesiredGait, "Get get desired future gait.\n")
            .def("get_past_gait", &Gait::getPastGait, "Get previous gait.\n")
            .def("get_remaining_time", &Gait::getRemainingTime, "Get remaining time.\n")
            .def("get_is_static", &Gait::getIsStatic, "Get is_static.\n")
            .def("get_q_static", &Gait::getQStatic, "Get q_static.\n");
    }

    static void expose()
    {
        bp::class_<Gait>("Gait", bp::no_init).def(GaitVisitor<Gait>());
        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeGait() { GaitVisitor<Gait>::expose(); }


template <typename Planner>
struct PlannerPythonVisitor : public bp::def_visitor<PlannerPythonVisitor<Planner>>
{
    template <class PyClassPlanner>
    void visit(PyClassPlanner& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def(bp::init<double, double, double, double, int, double, const MatrixN&, const MatrixN&, Gait&>(
                bp::args("dt_in", "dt_tsid_in", "T_gait_in", "T_mpc_in", "k_mpc_in", "h_ref_in",
                         "fsteps_in", "shoulders positions", "gait"),
                "Constructor with parameters."))

            .def("get_xref", &Planner::get_xref, "Get xref matrix.\n")
            .def("get_fsteps", &Planner::get_fsteps, "Get fsteps matrix.\n")
            .def("get_goals", &Planner::get_goals, "Get position goals matrix.\n")
            .def("get_vgoals", &Planner::get_vgoals, "Get velocity goals matrix.\n")
            .def("get_agoals", &Planner::get_agoals, "Get acceleration goals matrix.\n")

            // Run Planner from Python
            .def("run_planner", &Planner::run_planner, bp::args("k", "q", "v", "b_vref", "h_estim", "z_average", "joystick_code"),
                 "Run Planner from Python.\n");
    }

    static void expose()
    {
        bp::class_<Planner>("Planner", bp::no_init).def(PlannerPythonVisitor<Planner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposePlanner() { PlannerPythonVisitor<Planner>::expose(); }


template <typename InvKin>
struct InvKinPythonVisitor : public bp::def_visitor<InvKinPythonVisitor<InvKin>>
{
    template <class PyClassInvKin>
    void visit(PyClassInvKin& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def(bp::init<double>(bp::args("dt_in"), "Constructor with parameters."))

            .def("get_q_step", &InvKin::get_q_step, "Get velocity goals matrix.\n")
            .def("get_dq_cmd", &InvKin::get_dq_cmd, "Get acceleration goals matrix.\n")

            // Run InvKin from Python
            .def("refreshAndCompute", &InvKin::refreshAndCompute,
                 bp::args("x_cmd", "contacts", "goals", "vgoals", "agoals", "posf", "vf", "wf", "af", "Jf",
                          "posb", "rotb", "vb", "ab", "Jb"),
                 "Run InvKin from Python.\n");
    }

    static void expose()
    {
        bp::class_<InvKin>("InvKin", bp::no_init).def(InvKinPythonVisitor<InvKin>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};

void exposeInvKin() { InvKinPythonVisitor<InvKin>::expose(); }

template <typename QPWBC>
struct QPWBCPythonVisitor : public bp::def_visitor<QPWBCPythonVisitor<QPWBC>>
{
    template <class PyClassQPWBC>
    void visit(PyClassQPWBC& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("get_f_res", &QPWBC::get_f_res, "Get velocity goals matrix.\n")
            .def("get_ddq_res", &QPWBC::get_ddq_res, "Get acceleration goals matrix.\n")
            .def("get_H", &QPWBC::get_H, "Get H weight matrix.\n")

            // Run QPWBC from Python
            .def("run", &QPWBC::run, bp::args("M", "Jc", "f_cmd", "RNEA", "k_contacts"), "Run QPWBC from Python.\n");
    }

    static void expose()
    {
        bp::class_<QPWBC>("QPWBC", bp::no_init).def(QPWBCPythonVisitor<QPWBC>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};

void exposeQPWBC() { QPWBCPythonVisitor<QPWBC>::expose(); }

BOOST_PYTHON_MODULE(libquadruped_reactive_walking)
{
    boost::python::def("add", gepetto::example::add);
    boost::python::def("sub", gepetto::example::sub);

    eigenpy::enableEigenPy();

    exposeMPC();
    exposeGait();
    exposePlanner();
    exposeInvKin();
    exposeQPWBC();
}