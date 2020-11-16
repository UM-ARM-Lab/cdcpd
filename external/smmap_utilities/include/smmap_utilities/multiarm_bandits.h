#ifndef MULTIARM_BANDITS_H
#define MULTIARM_BANDITS_H

#include <assert.h>
#include <vector>
#include <random>
#include <utility>
#include <Eigen/Dense>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>

namespace smmap
{
    class MABBase
    {
        public:
            MABBase(size_t num_arms);

            virtual ssize_t selectArmToPull(std::mt19937_64& generator) const = 0;

            virtual bool generateAllModelActions() const = 0;

            virtual void updateArms(const ssize_t arm_pulled, const double reward);

            virtual void updateArms(
                    const Eigen::VectorXd& transition_variance,
                    const ssize_t arm_pulled,
                    const double observed_reward,
                    const double observation_variance);

            virtual void updateArms(
                    const Eigen::MatrixXd& transition_covariance,
                    const Eigen::MatrixXd& observation_matrix,
                    const Eigen::VectorXd& observed_reward,
                    const Eigen::MatrixXd& observation_covariance);

            virtual Eigen::VectorXd getMean() const = 0;

            virtual Eigen::MatrixXd getSecondStat() const = 0;

        protected:
            size_t num_arms_;
    };

    class UCB1NormalBandit : public MABBase
    {
        public:
            UCB1NormalBandit(size_t num_arms);

            virtual ssize_t selectArmToPull(std::mt19937_64& generator) const override final;

            virtual bool generateAllModelActions() const override final;

            virtual void updateArms(const ssize_t arm_pulled, const double reward) override final;

            virtual Eigen::VectorXd getMean() const override final;

            virtual Eigen::MatrixXd getSecondStat() const override final;

            Eigen::VectorXd getUCB() const;

        private:
            std::vector<double> total_reward_;
            std::vector<double> sum_of_squared_reward_;
            std::vector<size_t> num_pulls_;
            size_t total_pulls_;
    };

    class KalmanFilterMANB : public MABBase
    {
        public:
            KalmanFilterMANB(
                    const Eigen::VectorXd& prior_mean,
                    const Eigen::VectorXd& prior_var);

            /**
             * @brief selectArmToPull Perform Thompson sampling on the bandits,
             *                        and select the bandit with the largest sample.
             * @param generator
             * @return
             */
            virtual ssize_t selectArmToPull(std::mt19937_64& generator) const override final;

            virtual bool generateAllModelActions() const;

            /**
             * @brief updateArms
             * @param transition_variance
             * @param arm_pulled
             * @param observed_reward
             * @param observation_variance
             */
            virtual void updateArms(
                    const Eigen::VectorXd& transition_variance,
                    const ssize_t arm_pulled,
                    const double observed_reward,
                    const double observation_variance) override final;

            virtual Eigen::VectorXd getMean() const override final;

            virtual Eigen::MatrixXd getSecondStat() const override final;

            Eigen::MatrixXd getVariance() const;

        private:
            Eigen::VectorXd arm_mean_;
            Eigen::VectorXd arm_var_;
    };

    class KalmanFilterMANDB : public MABBase
    {
        public:
            KalmanFilterMANDB(
                    const Eigen::VectorXd& prior_mean,
                    const Eigen::MatrixXd& prior_covar);

            /**
             * @brief selectArmToPull Perform Thompson sampling on the bandits,
             *                        and select the bandit with the largest sample.
             * @param generator
             * @return
             */
            virtual ssize_t selectArmToPull(std::mt19937_64& generator) const override final;

            virtual bool generateAllModelActions() const override final;

            /**
             * @brief updateArms
             * @param transition_covariance
             * @param arm_pulled
             * @param obs_reward
             * @param obs_var
             */
            virtual void updateArms(
                    const Eigen::MatrixXd& transition_covariance,
                    const Eigen::MatrixXd& observation_matrix,
                    const Eigen::VectorXd& observed_reward,
                    const Eigen::MatrixXd& observation_covariance) override final;

            virtual Eigen::VectorXd getMean() const override final;

            virtual Eigen::MatrixXd getSecondStat() const override final;

            const Eigen::MatrixXd& getCovariance() const;

        private:
            Eigen::VectorXd arm_mean_;
            Eigen::MatrixXd arm_covar_;
    };
}

#endif // MULTIARM_BANDITS_H
