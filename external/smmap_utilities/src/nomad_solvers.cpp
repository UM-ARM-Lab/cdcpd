#include "smmap_utilities/nomad_solvers.h"

#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/math_helpers.hpp>

namespace smmap
{
    ///////////////////////////////////////////////////////////////////
    // Straight robot DOF
    ///////////////////////////////////////////////////////////////////

    Eigen::VectorXd minFunctionPointerDirectRobotDOF(
            const std::string& log_file_path,
            const bool fix_step,
            const int max_count,
            const ssize_t num_dof,
            const Eigen::VectorXd& x_lower_bound,
            const Eigen::VectorXd& x_upper_bound,
            std::mt19937_64& generator,
            std::uniform_real_distribution<double>& uniform_unit_distribution,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& eval_error_cost_fn,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& collision_constraint_fn,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& stretching_constraint_fn,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& robot_motion_constraint_fn)
    {
        assert(num_dof > 0);
        assert(num_dof < std::numeric_limits<int>::max());
        assert(x_lower_bound.size() == num_dof);
        assert(x_upper_bound.size() == num_dof);

        Eigen::VectorXd optimal_robot_command;

        // TODO: figure out a way to deal with logging, for now leave extra code here for reference
        std::ofstream out(log_file_path.c_str(), std::ios::out);
        // NOMAD::Display out (std::cout);
        out.precision(NOMAD::DISPLAY_PRECISION_STD);

        try
        {
            // NOMAD initializations:
            NOMAD::begin(0, nullptr);

            // parameters creation:
            NOMAD::Parameters p(out);
            p.set_DIMENSION((int)num_dof);  // number of variables

            // Set the output types for each black box function evaluation
            {
                std::vector<NOMAD::bb_output_type> bbot (4); // definition of
                bbot[0] = NOMAD::OBJ;                   // output types
                // TODO: might need to decide which kind of constraint to use
                bbot[1] = NOMAD::PB;
                bbot[2] = NOMAD::PB;
                bbot[3] = NOMAD::PB;

                if (fix_step)
                {
                    bbot.push_back(NOMAD::EB);
                }

                p.set_BB_OUTPUT_TYPE(bbot);
            }

            // Create the initial samples for NOMAD
            {
                const int size_of_initial_batch = 5;
                for (int sample_ind = 0; sample_ind < size_of_initial_batch; sample_ind++)
                {
                    NOMAD::Point x0 = NOMAD::Point((int)num_dof, 0.0);
                    for (int coord_ind = 0; coord_ind < num_dof; coord_ind++)
                    {
                        const double sample = uniform_unit_distribution(generator);
                        const double value_in_limits = EigenHelpers::Interpolate(x_lower_bound[coord_ind], x_upper_bound[coord_ind], sample);
                        x0.set_coord(coord_ind, value_in_limits);
                    }
                    p.set_X0(x0);
                }
            }

            // Set the upper and lower bound for the decision variables
            {
                for (int ind = 0; ind < num_dof; ++ind)
                {
                    p.set_LOWER_BOUND(ind, x_lower_bound(ind));
                    p.set_UPPER_BOUND(ind, x_upper_bound(ind));
                }
            }

            // Set the parameters for the solver itself
            {
                p.set_MAX_BB_EVAL(max_count);     // the algorithm terminates after max_count_ black-box evaluations
                p.set_DISPLAY_DEGREE(2);
                //p.set_SGTELIB_MODEL_DISPLAY("");
                p.set_SOLUTION_FILE("sol.txt");
            }

            // parameters validation:
            p.check();

            // custom evaluator creation:
            RobotMotionNomadEvaluator ev(p,
                                         num_dof,
                                         eval_error_cost_fn,
                                         collision_constraint_fn,
                                         stretching_constraint_fn,
                                         robot_motion_constraint_fn,
                                         fix_step);

            // algorithm creation and execution:
            NOMAD::Mads mads(p, &ev);
            mads.run();

            const NOMAD::Eval_Point* best_x = mads.get_best_feasible();
            if (best_x != nullptr)
            {
                optimal_robot_command = ev.evalPointToRobotDOFDelta(*best_x);
                assert(optimal_robot_command.size() == num_dof);
            }
            else
            {
                std::cerr << "   ---------  No output from NOMAD evaluator ------------, setting motion to zero" << std::endl;
                optimal_robot_command = Eigen::VectorXd::Zero(num_dof);
            }
        }
        catch (std::exception& e)
        {
            std::cerr << "\nNOMAD has been interrupted (" << e.what() << ")\n\n";
            assert(false);
        }

        NOMAD::Slave::stop_slaves(out);
        NOMAD::end();

        return optimal_robot_command;
    }

    RobotMotionNomadEvaluator::RobotMotionNomadEvaluator(
            const NOMAD::Parameters& p,
            const ssize_t num_dof,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& eval_error_cost_fn,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& collision_constraint_fn,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& stretching_constraint_fn,
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& robot_motion_constraint_fn,
            const bool fix_step_size)
        : NOMAD::Evaluator(p)
        , num_dof_(num_dof)
        , eval_error_cost_fn_(eval_error_cost_fn)
        , collision_constraint_fn_(collision_constraint_fn)
        , stretching_constraint_fn_(stretching_constraint_fn)
        , robot_motion_constraint_fn_(robot_motion_constraint_fn)
        , fix_step_size_(fix_step_size)
    {}

    bool RobotMotionNomadEvaluator::eval_x(
            NOMAD::Eval_Point& x,
            const NOMAD::Double& h_max,
            bool& count_eval)
    {
        UNUSED(h_max); // TODO: Why don't we use h_max?

        // count a black-box evaluation
        count_eval = true;

        // Convert NOMAD points into
        const Eigen::VectorXd test_robot_motion = evalPointToRobotDOFDelta(x);

        std::cout << "\ntest motion: " << test_robot_motion.transpose() << std::endl;

        NOMAD::Double c1_error_cost = eval_error_cost_fn_(test_robot_motion);
        NOMAD::Double c2_collision_constraint = collision_constraint_fn_(test_robot_motion);
        NOMAD::Double c3_stretching_constraint = stretching_constraint_fn_(test_robot_motion);
        NOMAD::Double c4_robot_motion_constraint = robot_motion_constraint_fn_(test_robot_motion);

        std::cout << "Cost:                    " << c1_error_cost.value() << std::endl;
        std::cout << "collision constraint:    " << c2_collision_constraint.value() << std::endl;
        std::cout << "Stretching constraint:   " << c3_stretching_constraint.value() << std::endl;
        std::cout << "Robot motion constraint: " << c4_robot_motion_constraint.value() << std::endl;

        // objective value
        x.set_bb_output(0, c1_error_cost);

        // constraints
        x.set_bb_output(1, c2_collision_constraint);
        x.set_bb_output(2, c3_stretching_constraint);
        x.set_bb_output(3, c4_robot_motion_constraint);

        if (fix_step_size_)
        {
            if (x.get_bb_outputs().size() < 5)
            {
                assert(false && "size of x not match due to the fix step size constraint");
            }
            x.set_bb_output(4, -c4_robot_motion_constraint);
        }

        return count_eval;
    }

    Eigen::VectorXd RobotMotionNomadEvaluator::evalPointToRobotDOFDelta(
            const NOMAD::Eval_Point& x)
    {
        if (x.size() != num_dof_)
        {
            assert(false && "num_dof and eval_point x have different size");
        }

        Eigen::VectorXd robot_motion(num_dof_);
        for (int ind = 0; ind < num_dof_; ind ++)
        {
            robot_motion(ind) = x[ind].value();
        }

        return robot_motion;
    }

    ///////////////////////////////////////////////////////////////////
    // Gripper motion (tangent SE(3))
    ///////////////////////////////////////////////////////////////////

    AllGrippersSinglePoseDelta minFunctionPointerSE3Delta(
            const std::string& log_file_path,
            const bool fix_step,
            const int max_count,
            const ssize_t num_grippers,
            const double max_step_size,
            std::mt19937_64& generator,
            std::uniform_real_distribution<double>& uniform_unit_distribution,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& eval_error_cost_fn,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& collision_constraint_fn,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& stretching_constraint_fn,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& gripper_motion_constraint_fn)
    {
        AllGrippersSinglePoseDelta optimal_gripper_command;
        const int x_dim = (int)(6 * num_grippers);

        // TODO: figure out a way to deal with logging, for now leave extra code here for reference
        std::ofstream out(log_file_path.c_str(), std::ios::out);
        // NOMAD::Display out (std::cout);
        out.precision(NOMAD::DISPLAY_PRECISION_STD);

        try
        {
            // NOMAD initializations:
            NOMAD::begin(0, nullptr);

            // parameters creation:
            NOMAD::Parameters p(out);
            p.set_DIMENSION((int)(6 * num_grippers));  // number of variables

            // Set the output types for each black box function evaluation
            {
                std::vector<NOMAD::bb_output_type> bbot (4); // definition of
                bbot[0] = NOMAD::OBJ;                   // output types
                // TODO: might need to decide which kind of constraint to use
                bbot[1] = NOMAD::PB;
                bbot[2] = NOMAD::PB;
                bbot[3] = NOMAD::PB;

                if (fix_step)
                {
                    bbot.push_back(NOMAD::EB);
                }

                p.set_BB_OUTPUT_TYPE(bbot);
            }

            // Create the initial samples for NOMAD
            {
                const int size_of_initial_batch = 5;
                for (int sample_ind = 0; sample_ind < size_of_initial_batch; sample_ind++)
                {
                    NOMAD::Point x0 = NOMAD::Point(x_dim, 0.0);
                    for (int coord_ind = 0; coord_ind < x_dim; coord_ind++)
                    {
                        x0.set_coord(coord_ind, EigenHelpers::Interpolate(-max_step_size, max_step_size, uniform_unit_distribution(generator)));
                    }
                    p.set_X0(x0);
                }
            }

            // Set the upper and lower bound for the decision variables
            {
                p.set_LOWER_BOUND(NOMAD::Point((int)(6 * num_grippers), -max_step_size));
                p.set_UPPER_BOUND(NOMAD::Point((int)(6 * num_grippers), max_step_size));
            }

            // Set the parameters for the solver itself
            {
                p.set_MAX_BB_EVAL(max_count);     // the algorithm terminates after max_count_ black-box evaluations
                p.set_DISPLAY_DEGREE(2);
                //p.set_SGTELIB_MODEL_DISPLAY("");
                p.set_SOLUTION_FILE("sol.txt");
            }

            // parameters validation:
            p.check();

            // custom evaluator creation:
            GripperMotionNomadEvaluator ev(p,
                                           num_grippers,
                                           eval_error_cost_fn,
                                           collision_constraint_fn,
                                           stretching_constraint_fn,
                                           gripper_motion_constraint_fn,
                                           fix_step);

            // algorithm creation and execution:
            NOMAD::Mads mads(p, &ev);
            mads.run();

            const NOMAD::Eval_Point* best_x = mads.get_best_feasible();
            if (best_x != nullptr)
            {
                optimal_gripper_command = ev.evalPointToGripperPoseDelta(*best_x);
                assert((ssize_t)(optimal_gripper_command.size()) == num_grippers);
            }
            else
            {
                std::cerr << "   ---------  No output from NOMAD evaluator ------------, setting motion to zero" << std::endl;
                optimal_gripper_command = AllGrippersSinglePoseDelta(num_grippers, kinematics::Vector6d::Zero());
            }
        }
        catch (std::exception& e)
        {
            std::cerr << "\nNOMAD has been interrupted (" << e.what() << ")\n\n";
            assert(false);
        }

        NOMAD::Slave::stop_slaves(out);
        NOMAD::end();

        return optimal_gripper_command;
    }

    GripperMotionNomadEvaluator::GripperMotionNomadEvaluator(
            const NOMAD::Parameters & p,
            const ssize_t num_grippers,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& eval_error_cost_fn,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& collision_constraint_fn,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& stretching_constraint_fn,
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& gripper_motion_constraint_fn,
            const bool fix_step_size)
        : NOMAD::Evaluator(p)
        , num_grippers_(num_grippers)
        , eval_error_cost_fn_(eval_error_cost_fn)
        , collision_constraint_fn_(collision_constraint_fn)
        , stretching_constraint_fn_(stretching_constraint_fn)
        , gripper_motion_constraint_fn_(gripper_motion_constraint_fn)
        , fix_step_size_(fix_step_size)
    {}

    bool GripperMotionNomadEvaluator::eval_x(
            NOMAD::Eval_Point& x,
            const NOMAD::Double& h_max,
            bool& count_eval)
    {
        UNUSED(h_max); // TODO: Why don't we use h_max?

        // count a black-box evaluation
        count_eval = true;

        // Convert NOMAD points into
        const AllGrippersSinglePoseDelta test_grippers_motions = evalPointToGripperPoseDelta(x);

        NOMAD::Double c1_error_cost = eval_error_cost_fn_(test_grippers_motions);
        NOMAD::Double c2_collision_constraint = collision_constraint_fn_(test_grippers_motions);
        NOMAD::Double c3_stretching_constraint = stretching_constraint_fn_(test_grippers_motions);
        NOMAD::Double c4_gripper_motion_constraint = gripper_motion_constraint_fn_(test_grippers_motions);

        // objective value
        x.set_bb_output(0, c1_error_cost);

        // constraints
        x.set_bb_output(1, c2_collision_constraint);
        x.set_bb_output(2, c3_stretching_constraint);
        x.set_bb_output(3, c4_gripper_motion_constraint);

        if (fix_step_size_)
        {
            if (x.get_bb_outputs().size() < 5)
            {
                assert(false && "size of x not match due to the fix step size constraint");
            }
            x.set_bb_output(4, -c4_gripper_motion_constraint);
        }

        return count_eval;
    }

    AllGrippersSinglePoseDelta GripperMotionNomadEvaluator::evalPointToGripperPoseDelta(
            const NOMAD::Eval_Point& x)
    {
        const int single_gripper_dimension = 6;
        if (x.size() != num_grippers_ * single_gripper_dimension)
        {
            assert(false && "grippers data and eval_point x have different size");
        }

        AllGrippersSinglePoseDelta grippers_motion(num_grippers_);
        for (int gripper_ind = 0; gripper_ind < num_grippers_; gripper_ind ++)
        {
            kinematics::Vector6d& single_gripper_delta = grippers_motion[gripper_ind];

            single_gripper_delta(0) = x[gripper_ind * single_gripper_dimension].value();
            single_gripper_delta(1) = x[gripper_ind * single_gripper_dimension + 1].value();
            single_gripper_delta(2) = x[gripper_ind * single_gripper_dimension + 2].value();

            single_gripper_delta(3) = x[gripper_ind * single_gripper_dimension + 3].value();
            single_gripper_delta(4) = x[gripper_ind * single_gripper_dimension + 4].value();
            single_gripper_delta(5) = x[gripper_ind * single_gripper_dimension + 5].value();
        }

        return grippers_motion;
    }
}
