#pragma once

#include <sdm/utils/config.hpp>
#include <sdm/algorithms/planning/value_iteration.hpp>
#include <sdm/algorithms/planning/pbvi.hpp>

#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/algorithms/planning/dfsvi.hpp>
#include <sdm/algorithms/planning/perseus.hpp>

#include <sdm/algorithms/rl/q_learning.hpp>
#include <sdm/algorithms/rl/sarsa.hpp>
#include <sdm/algorithms/planning/alpha_star.hpp>
#include <sdm/algorithms/planning/backward_induction.hpp>
#include <sdm/algorithms/planning/bayesian_game_solver.hpp>

namespace sdm
{
    /**
     * @brief Namespace grouping functions to manipulate algorithms.
     */
    namespace algo
    {

        std::shared_ptr<ActionSelectionInterface> makeActionSawtoothLP(std::shared_ptr<SolvableByDP> problem,
                                                                       std::string value_name,
                                                                       std::string type_of_resolution_name);

        std::shared_ptr<ValueFunction> makeValueFunction(std::shared_ptr<SolvableByDP> problem,
                                                         std::string value_name,
                                                         std::string init_name,
                                                         bool store_state,
                                                         bool pessimistic,
                                                         std::string type_of_resolution_name,
                                                         std::string type_of_pruning,
                                                         int freq_pruning);
        /**
         * @brief Build HSVI algorithm.
         */
        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem,
                                            double error = 0.01,
                                            int trials = 10000,
                                            bool store_state = true,
                                            bool store_action = true,
                                            std::string name = "hsvi",
                                            double time_max = 3600 /* 1h */ * 10,
                                            std::string lower_bound_name = "tabular",
                                            std::string upper_bound_name = "tabular",
                                            std::string lb_init_name = "Min",
                                            std::string ub_init_name = "Max",
                                            number lb_freq_update = 1,
                                            number ub_freq_update = 1,
                                            std::string lb_type_of_resolution_name = "",
                                            std::string ub_type_of_resolution_name = "",
                                            int lb_freq_pruning = 1,
                                            int ub_freq_pruning = 1,
                                            std::string lb_type_of_pruning = "none",
                                            std::string ub_type_of_pruning = "none");

        /**
         * @brief Build the ValueIteration version that use TabularValueFunction Representation
         */
        std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem,
                                                                std::string value_function_name, std::string vf_init_name, double error,
                                                                bool store_state, std::string name, double time_max,
                                                                std::string vf_type_of_resolution_name,
                                                                int vf_freq_pruning, std::string vf_type_of_pruning);

        std::shared_ptr<sdm::PBVI> makePBVI(std::shared_ptr<SolvableByHSVI> problem,
                                            std::string value_function_name, std::string vf_init_name, unsigned long long num_samples, std::string type_sampling, double error,
                                            bool store_state, std::string name, double time_max,
                                            std::string vf_type_of_resolution_name,
                                            int vf_freq_pruning, std::string vf_type_of_pruning);
        /**
         * @brief Build Q-Value Function.
         */
        std::shared_ptr<QValueFunction> makeQValueFunction(std::shared_ptr<SolvableByDP> problem, std::string qvalue_name, std::string q_init_name);

        /**
         * @brief Build QLearning algorithm.
         */
        std::shared_ptr<sdm::QLearning> makeRL(std::shared_ptr<SolvableByDP> problem,
                                                      std::string algo_name = "qlearning",
                                                      std::string qvalue_name = "tabular",
                                                      std::string q_init_name = "Zero",
                                                      number horizon = 0,
                                                      double discount = 0.9,
                                                      double rate_start = 1.0, double rate_end = 0.001, double rate_decay = 10000,
                                                      double eps_start = 1.0, double eps_end = 0.001, double eps_decay = 10000,
                                                      double batch_size = 1,
                                                      unsigned long long num_episodes = 10000,
                                                      std::string name = "qlearning");

        /**
         * @brief Build a problem.
         */
        std::shared_ptr<SolvableByHSVI> makeFormalism(Config config);

        std::shared_ptr<SolvableByHSVI> makeFormalism(std::string problem_path,
                                                      std::string formalism,
                                                      double discount,
                                                      number horizon,
                                                      int memory,
                                                      StateType state_type,
                                                      bool store_state,
                                                      bool store_action,
                                                      number batch_size);

        /**
         * @brief Build an algorithm.
         */
        std::shared_ptr<Algorithm> makeAlgorithm(std::string algo_name, std::shared_ptr<SolvableByHSVI> formalism, double discount,
                                                 double error, int trials, bool store_state, bool store_action, std::string name, double time_max, unsigned long long num_samples, std::string type_sampling,
                                                 double rate_start, double rate_end, double rate_decay, double eps_start, double eps_end, double eps_decay,
                                                 std::string value_function_1, std::string init_v1, number freq_update_v1, std::string type_of_resolution_v1, int freq_pruning_v1, std::string type_of_pruning_v1,
                                                 std::string value_function_2, std::string init_v2, number freq_update_v2, std::string type_of_resolution_v2, int freq_pruning_v2, std::string type_of_pruning_v2);

        /**
         * @brief Build an algorithm given his name and the configurations required.
         *
         * @param algo_name the name of the algorithm to be built
         * @param problem_path the path to the problem to be solved
         * @param formalism_name the name of the formalism to consider to solve the problem
         * @param upper_bound a string describing the upper bound
         * @param upper_bound a string describing the lower bound
         * @param ub_init a string describing the way to initialize the upper bound
         * @param lb_init a string describing the way to initialize the lower bound
         * @param discount the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @param trials the maximum number of trials
         * @param memory if greater than 0, specify the max size of the history
         * @param batch_size for learning algorithms only
         * @return pointer on algorithm instance
         */
        std::shared_ptr<Algorithm> make(std::string algo_name,
                                        std::string problem_path,
                                        number horizon = 10,
                                        double discount = 1.,
                                        double error = 0.01,
                                        int trials = 10000,
                                        double time_max = 3600 /* 1h */ * 10,
                                        std::string name = "",
                                        int memory = -1,
                                        StateType state_type = COMPRESSED,
                                        bool store_state = true,
                                        bool store_action = true,
                                        number batch_size = 0,
                                        unsigned long long num_samples = 10,
                                        std::string type_sampling = "",
                                        double rate_start = 1.0, double rate_end = 0.001, double rate_decay = 1000,
                                        double eps_start = 1.0, double eps_end = 0.001, double eps_decay = 10000,
                                        std::string value_function_1 = "tabular",
                                        std::string init_v1 = "Min",
                                        number freq_update_v1 = 1,
                                        std::string type_of_resolution_v1 = "",
                                        int freq_pruning_v1 = 1,
                                        std::string type_of_pruning_v1 = "none",
                                        std::string value_function_2 = "tabular",
                                        std::string init_v2 = "Max",
                                        number freq_update_v2 = 1,
                                        std::string type_of_resolution_v2 = "",
                                        int freq_pruning_v2 = 1,
                                        std::string type_of_pruning_v2 = "none");

        /**
         * @brief Get the list of available algorithms.
         *
         * Usage:
         * ```cpp
         * #include <sdm/algorithms.hpp>
         *
         * int main()
         * {
         *     std::cout << sdm::algo::available() << std::endl;
         *     return 0;
         * }
         * ```
         *
         * @return the list of available algorithms.
         *
         */
        std::vector<std::string> available();

    } // namespace algo
} // namespace sdm
