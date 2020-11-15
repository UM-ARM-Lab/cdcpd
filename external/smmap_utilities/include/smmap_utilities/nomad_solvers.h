#ifndef NOMAD_SOLVERS_H
#define NOMAD_SOLVERS_H

#include <random>
#include <nomad.hpp>
#include <Eigen/Dense>
#include <kinematics_toolbox/kinematics.h>

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
            const std::function<double(const Eigen::VectorXd& test_robot_motion)>& robot_motion_constraint_fn);

    class RobotMotionNomadEvaluator : public NOMAD::Evaluator
    {
    public:
        RobotMotionNomadEvaluator(
                const NOMAD::Parameters& p,
                const ssize_t num_dof,
                const std::function<double(const Eigen::VectorXd& test_robot_motion)>& eval_error_cost_fn,
                const std::function<double(const Eigen::VectorXd& test_robot_motion)>& collision_constraint_fn,
                const std::function<double(const Eigen::VectorXd& test_robot_motion)>& stretching_constraint_fn,
                const std::function<double(const Eigen::VectorXd& test_robot_motion)>& robot_motion_constraint_fn,
                const bool fix_step_size = false);

        bool eval_x(
                NOMAD::Eval_Point& x,
                const NOMAD::Double& h_max,
                bool& count_eval);

        Eigen::VectorXd evalPointToRobotDOFDelta(
                const NOMAD::Eval_Point& x);

    private:
        const ssize_t num_dof_;

        const std::function<double(const Eigen::VectorXd& test_robot_motion)> eval_error_cost_fn_;
        const std::function<double(const Eigen::VectorXd& test_robot_motion)> collision_constraint_fn_;
        const std::function<double(const Eigen::VectorXd& test_robot_motion)> stretching_constraint_fn_;
        const std::function<double(const Eigen::VectorXd& test_robot_motion)> robot_motion_constraint_fn_;
        const bool fix_step_size_;
    };

    ///////////////////////////////////////////////////////////////////
    // Gripper motion (tangent SE(3))
    ///////////////////////////////////////////////////////////////////

    // TODO: find a better place for this
    typedef kinematics::VectorVector6d AllGrippersSinglePoseDelta;

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
            const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& gripper_motion_constraint_fn);

    class GripperMotionNomadEvaluator : public NOMAD::Evaluator
    {
    public:
        GripperMotionNomadEvaluator(
                const NOMAD::Parameters& p,
                const ssize_t num_grippers,
                const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& eval_error_cost_fn,
                const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& collision_constraint_fn,
                const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& stretching_constraint_fn,
                const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)>& gripper_motion_constraint_fn,
                const bool fix_step_size = false);

        bool eval_x(
                NOMAD::Eval_Point& x,
                const NOMAD::Double& h_max,
                bool& count_eval);

        AllGrippersSinglePoseDelta evalPointToGripperPoseDelta(
                const NOMAD::Eval_Point& x);

    private:
        const ssize_t num_grippers_;

        const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)> eval_error_cost_fn_;
        const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)> collision_constraint_fn_;
        const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)> stretching_constraint_fn_;
        const std::function<double(const AllGrippersSinglePoseDelta& test_gripper_motion)> gripper_motion_constraint_fn_;
        const bool fix_step_size_;
    };
}

#endif // NOMAD_SOLVERS_H
