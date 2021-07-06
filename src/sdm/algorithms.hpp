#pragma once

#include <sdm/algorithms/hsvi.hpp>
#include <sdm/algorithms/value_iteration.hpp>

namespace sdm
{
    namespace algo
    {
        /**
         * @brief Build the HSVI version that use TabularValueFunction Representation. 
         * 
         * @tparam TState Type of the state.
         * @tparam TAction Type of the action.
         * @param problem the problem to be solved
         * @param discount the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @return pointer on HSVI instance
         */
        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem,
                                            std::string upper_bound_name,
                                            std::string lower_bound_name,
                                            std::string ub_init_name,
                                            std::string lb_init_name,
                                            double discount = 0.99,
                                            double error = 0.01,
                                            number horizon = 0,
                                            int trials = 1000,
                                            std::string name = "",
                                            std::string current_type_of_resolution,
                                            number BigM,
                                            std::string type_sawtooth_linear_programming);


        /**
         * @brief Build the ValueIteration version. 
         * 
         * @tparam TState Type of the state.
         * @tparam TAction Type of the action.
         * @param problem the problem to be solved
         * @param discount the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @return pointer on HSVI instance
         */
        std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem,
                                                                double discount,
                                                                double error,
                                                                number horizon);


        /**
         * @brief Build an algorithm given his name and the configurations required. 
         * 
         * @tparam TState Type of the state.
         * @tparam TAction Type of the action.
         * @param algo_name the name of the algorithm to be built* 
         * @param problem the problem to be solved
         * @param discount the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @param trials the maximum number of trials 
         * @return auto pointer on algorithm instance
         */
        std::shared_ptr<Algorithm> make(std::string algo_name,
                                        std::string problem_path,
                                        std::string formalism,
                                        std::string upper_bound,
                                        std::string lower_bound,
                                        std::string ub_init,
                                        std::string lb_init,
                                        double discount = 0.99,
                                        double error = 0.001,
                                        number horizon = 0,
                                        int trials = 1000,
                                        int truncation = -1,
                                        std::string name = "",
                                        std::string current_type_of_resolution = "BigM",
                                        number BigM = 100,
                                        std::string type_sawtooth_linear_programming = "Full");

        // std::shared_ptr<sdm::QLearning<TObservation, TAction>> makeQLearning(std::shared_ptr<GymInterface<TObservation, TAction>> problem,
        //                                                                 std::string,
        //                                                                 std::string,
        //                                                                 number horizon = 0,
        //                                                                 double discount = 0.9,
        //                                                                 double lr = 0.01,
        //                                                                 double batch_size = 1,
        //                                                                 unsigned long num_max_steps = 100000,
        //                                                                 std::string name = "qlearning")
    }
}















// #include <random>

// #include <sdm/exception.hpp>
// #include <sdm/tools.hpp>
// #include <sdm/worlds.hpp>
// #include <sdm/algorithms/hsvi.hpp>
// #include <sdm/algorithms/q_learning.hpp>
// #include <sdm/algorithms/value_iteration.hpp>

// #include <sdm/public/algorithm.hpp>
// #include <sdm/core/states.hpp>
// #include <sdm/core/actions.hpp>

// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/utils/value_function/tabular_qvalue_function.hpp>

// #include <sdm/utils/value_function/sawtooth_vf.hpp>
// #include <sdm/utils/value_function/sawtooth_vf_with_lp.hpp>

// #include <sdm/utils/value_function/initializer/initializers.hpp>
// #include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

// #include <sdm/utils/value_function/max_plan_vf.hpp>
// #include <sdm/utils/value_function/max_plan_vf_with_lp.hpp>
// #include <sdm/utils/value_function/max_plane_vf_serialized.hpp>

// #include <sdm/utils/rl/exploration.hpp>

// namespace sdm
// {
//     namespace algo
//     {
//         /**
//          * @brief Build the HSVI version that use TabularValueFunction Representation. 
//          * 
//          * @tparam TState Type of the state.
//          * @tparam TAction Type of the action.
//          * @param problem the problem to be solved
//          * @param discount the discount factor
//          * @param error the accuracy
//          * @param horizon the planning horizon
//          * @return pointer on HSVI instance
//          */
//         std::shared_ptr<sdm::HSV> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound_name, std::string lower_bound_name, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution, number BigM, std::string type_sawtooth_linear_programming)
//         {
//             // assert(((discount < 1) || (horizon > 0)));

//             // // Set params in the environment
//             // problem->getUnderlyingProblem()->setDiscount(discount);
//             // problem->getUnderlyingProblem()->setPlanningHorizon(horizon);

//             // // Increase the horizon for the value function if the problem is serialized
//             // if (problem->isSerialized())
//             // {
//             //     horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
//             // }

//             // // Instanciate initializers
//             // // auto lb_init = sdm::makeInitializer(lb_init_name);
//             // // auto ub_init = sdm::makeInitializer(ub_init_name);

//             // // Instanciate bounds
//             // std::shared_ptr<sdm::ValueFunction> lower_bound;
//             // if (lower_bound_name == "maxplan")
//             // {
//             //     // lower_bound = std::make_shared<sdm::MaxPlanValueFunction>(problem, horizon, lb_init);
//             // }
//             // else if (lower_bound_name == "maxplan_serial")
//             // {
//             //     // lower_bound = std::make_shared<sdm::MaxPlanValueFunctionSerialized>(problem, horizon, lb_init);
//             // }
//             // else if (lower_bound_name == "maxplan_lp")
//             // {
//             //     // lower_bound = std::make_shared<sdm::MaxPlanValueFunctionLP>(problem, horizon, lb_init);
//             // }
//             // else
//             // {
//             //     // lower_bound = std::make_shared<sdm::MappedValueFunction>(problem, horizon, lb_init);
//             // }

//             // std::shared_ptr<sdm::ValueFunction> upper_bound;
//             // if (upper_bound_name == "sawtooth")
//             // {
//             //     // upper_bound = std::make_shared<sdm::SawtoothValueFunction>(problem, horizon, ub_init);
//             // }
//             // else if (upper_bound_name == "sawtooth_lp")
//             // {
//             //     if (type_sawtooth_linear_programming == "Full")
//             //     {
//             //         if (current_type_of_resolution == "BigM")
//             //         {
//             //             // upper_bound = std::make_shared<sdm::SawtoothValueFunctionLP>(problem, horizon, ub_init, TypeOfResolution::BigM, BigM, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);
//             //         }
//             //         else
//             //         {
//             //             // upper_bound = std::make_shared<sdm::SawtoothValueFunctionLP>(problem, horizon, ub_init, TypeOfResolution::IloIfThenResolution, BigM, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);
//             //         }
//             //     }
//             //     else
//             //     {
//             //         // upper_bound = std::make_shared<sdm::SawtoothValueFunctionLP>(problem, horizon, ub_init, TypeOfResolution::BigM, BigM, TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING);
//             //     }
//             // }
//             // else
//             // {
//             //     // upper_bound = std::make_shared<sdm::MappedValueFunction>(problem, horizon, ub_init);
//             // }

//             // return std::make_shared<HSVI>(problem, lower_bound, upper_bound, horizon, error, trials, name);
//         }

//         // /**
//         //  * @brief Build the ValueIteration version. 
//         //  * 
//         //  * @tparam TState Type of the state.
//         //  * @tparam TAction Type of the action.
//         //  * @param problem the problem to be solved
//         //  * @param discount the discount factor
//         //  * @param error the accuracy
//         //  * @param horizon the planning horizon
//         //  * @return pointer on HSVI instance
//         //  */
//         // std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem, double discount, double error, number horizon)
//         // {
//         //     // Set params in the environment
//         //     problem->getUnderlyingProblem()->setDiscount(discount);
//         //     problem->getUnderlyingProblem()->setPlanningHorizon(horizon);

//         //     // Increase the horizon for the value function if the problem is serialized
//         //     if (problem->isSerialized())
//         //     {
//         //         horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
//         //     }
//         //     return std::make_shared<ValueIteration>(problem, error, horizon);
//         // }

//         /**
//          * @brief Build an algorithm given his name and the configurations required. 
//          * 
//          * @tparam TState Type of the state.
//          * @tparam TAction Type of the action.
//          * @param algo_name the name of the algorithm to be built* 
//          * @param problem the problem to be solved
//          * @param discount the discount factor
//          * @param error the accuracy
//          * @param horizon the planning horizon
//          * @param trials the maximum number of trials 
//          * @return auto pointer on algorithm instance
//          */
//         std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, std::string formalism, std::string upper_bound, std::string lower_bound, std::string ub_init, std::string lb_init, double discount = 0.99, double error = 0.001, number horizon = 0, int trials = 1000, int truncation = -1, std::string name = "", std::string current_type_of_resolution = "BigM", number BigM = 100, std::string type_sawtooth_linear_programming = "Full")
//         {
//             std::shared_ptr<Algorithm> p_algo;

//             // if ((algo_name == "hsvi"))
//             // {
//             //     if ((formalism == "mdp") || (formalism == "MDP"))
//             //     {
//             //         // Attention, il faut créer un autre constructeur pour que le problème créer le MDP Interface avec seulement le problem path ?
//             //         auto base_mdp  = std::make_shared<BaseMDP>();


//             //         auto mdp = std::make_shared<MDP>(problem_path);
//             //         p_algo = makeHSVI(mdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_mdphsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     }
//             //     else if ((formalism == "pomdp") || (formalism == "POMDP"))
//             //     {
//             //         auto beliefMDP = std::make_shared<BeliefMDP>(problem_path);
//             //         p_algo = makeHSVI(beliefMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     // else if ((formalism == "pomdp_b") || (formalism == "POMDP_b"))
//             //     // {
//             //     //     using TAction = number;
//             //     //     using TObservation = number;
//             //     //     using TState = BeliefStateGraph_p<TAction, TObservation>;

//             //     //     auto pomdp = std::make_shared<DiscretePOMDP>(problem_path);
//             //     //     auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction, TObservation>>(pomdp);

//             //     //     p_algo = makeHSVI<TState, TAction>(beliefMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     // }
//             //     }
//             //     else if ((formalism == "decpomdp_b") || (formalism == "DecPOMDP_b") || (formalism == "dpomdp_b") || (formalism == "DPOMDP_b"))
//             //     {
//             //         // using TObservation = number;

//             //         // using TActionDescriptor = number;
//             //         // using TStateDescriptor = HistoryTree_p<TObservation>;

//             //         // // using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
//             //         // using TActionPrescriptor = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
//             //         // using TStatePrescriptor = OccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>;
//             //         // // using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

//             //         // auto oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(problem_path, (truncation > 0) ? truncation : horizon);
//             //         // p_algo = makeHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ohsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     }
//             //     else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
//             //     {
//             //         auto oMDP = std::make_shared<OccupancyMDP>(problem_path, (truncation > 0) ? truncation : horizon);
//             //         p_algo = makeHSVI(oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ohsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     }
//             //     else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP"))
//             //     {
//             //         auto serialized_mdp = std::make_shared<SerializedMMDP>(problem_path);
//             //         p_algo = makeHSVI(serialized_mdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ext_mdphsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     }
//             //     else if ((formalism == "extensive-pomdp") || (formalism == "Extensive-POMDP"))
//             //     {
//             //         auto serialized_pomdp = std::make_shared<SerializedBeliefMDP>(problem_path);
//             //         p_algo = makeHSVI(serialized_pomdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     }
//             //     else if ((formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") || (formalism == "Extensive-DPOMDP"))
//             //     {
//             //         auto serialized_oMDP = std::make_shared<OccupancyMDP>(problem_path, horizon);
//             //         p_algo = makeHSVI(serialized_oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ext_ohsvi" : name, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
//             //     }
//             // }
//             // else
//             // {
//             //     throw sdm::exception::Exception("Undefined algorithm type : " + algo_name);
//             // }

//             return p_algo;
//         }

//         // std::shared_ptr<sdm::QLearning<TObservation, TAction>> makeQLearning(std::shared_ptr<GymInterface<TObservation, TAction>> problem,
//         //                                                                      std::string,
//         //                                                                      std::string,
//         //                                                                      number horizon = 0,
//         //                                                                      double discount = 0.9,
//         //                                                                      double lr = 0.01,
//         //                                                                      double batch_size = 1,
//         //                                                                      unsigned long num_max_steps = 100000,
//         //                                                                      std::string name = "qlearning")
//         // {
//         //     assert(((discount < 1) || (horizon > 0)));

//         //     // Instanciate initializers and qvalue functions
//         //     auto initializer = std::make_shared<sdm::ZeroInitializer<TObservation, TAction>>();
//         //     auto qvalue = std::make_shared<sdm::MappedQValueFunction<TObservation, TAction>>(horizon, lr, initializer);

//         //     auto initializer_target = std::make_shared<sdm::ZeroInitializer<TObservation, TAction>>();
//         //     auto target_qvalue = std::make_shared<sdm::MappedQValueFunction<TObservation, TAction>>(horizon, lr, initializer_target);

//         //     // Instanciate exploration process
//         //     auto exploration_process = std::make_shared<sdm::EpsGreedy<TObservation, TAction>>();

//         //     return std::make_shared<QLearning<TObservation, TAction>>(problem, qvalue, target_qvalue, exploration_process, horizon, discount, lr, batch_size, num_max_steps, name);
//         // }

//         // /**
//         //  * @brief Build an algorithm given his name and the configurations required. 
//         //  * 
//         //  * @tparam TState Type of the state.
//         //  * @tparam TAction Type of the action.
//         //  * @param algo_name the name of the algorithm to be built* 
//         //  * @param problem the problem to be solved
//         //  * @param discount the discount factor
//         //  * @param error the accuracy
//         //  * @param horizon the planning horizon
//         //  * @param trials the maximum number of trials 
//         //  * @return auto pointer on algorithm instance
//         //  */
//         // std::shared_ptr<Algorithm> make(std::string algo_name,
//         //                                 std::string problem_path,
//         //                                 std::string formalism,
//         //                                 std::string qvalue_name,
//         //                                 std::string initializer_name,
//         //                                 number horizon = 0,
//         //                                 double discount = 0.9,
//         //                                 double lr = 0.01,
//         //                                 double batch_size = 1,
//         //                                 unsigned long num_max_steps = 100000,
//         //                                 std::string name = "qlearning")
//         // {
//         //     std::shared_ptr<Algorithm> p_algo = nullptr;

//         //     if ((algo_name == "qlearning"))
//         //     {
//         //         if ((formalism == "mdp") || (formalism == "MDP"))
//         //         {
//         //             using env_type = DiscreteMDP;
//         //             auto problem = std::make_shared<env_type>(problem_path);
//         //             problem->setDiscount(discount);
//         //             problem->setPlanningHorizon(horizon);
//         //             problem->setupDynamicsGenerator();
//         //             p_algo = makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount, lr, batch_size, num_max_steps, name);
//         //         }
//         //         else if ((formalism == "pomdp") || (formalism == "POMDP"))
//         //         {
//         //             using env_type = DiscretePOMDP;
//         //             auto problem = std::make_shared<env_type>(problem_path);
//         //             problem->setDiscount(discount);
//         //             problem->setPlanningHorizon(horizon);
//         //             problem->setupDynamicsGenerator();
//         //             p_algo = makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount, lr, batch_size, num_max_steps, name);
//         //         }
//         //         else if ((formalism == "beliefmdp") || (formalism == "BeliefMDP"))
//         //         {
//         //             using env_type = BeliefMDP<>;
//         //             auto problem = std::make_shared<env_type>(problem_path);
//         //             problem->getUnderlyingProblem()->setDiscount(discount);
//         //             problem->getUnderlyingProblem()->setPlanningHorizon(horizon);
//         //             problem->getUnderlyingProblem()->setupDynamicsGenerator();
//         //             p_algo = makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount, lr, batch_size, num_max_steps, name);
//         //         }
//         //         else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
//         //         {
//         //             using env_type = DiscreteDecPOMDP;
//         //             auto problem = std::make_shared<env_type>(problem_path);
//         //             problem->setDiscount(discount);
//         //             problem->setPlanningHorizon(horizon);
//         //             problem->setupDynamicsGenerator();
//         //             p_algo = makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount, lr, batch_size, num_max_steps, name);
//         //         }
//         //         else if ((formalism == "occupancymdp") || (formalism == "OccupancyMDP"))
//         //         {
//         //             using TObservation = number;
//         //             using TState = number;

//         //             using TActionDescriptor = number;
//         //             using TStateDescriptor = HistoryTree_p<TObservation>;

//         //             using TActionPrescriptor = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
//         //             using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

//         //             using env_type = OccupancyMDP<TStatePrescriptor, TActionPrescriptor>;
//         //             auto problem = std::make_shared<env_type>(problem_path, horizon);

//         //             problem->getUnderlyingProblem()->setDiscount(discount);
//         //             problem->getUnderlyingProblem()->setPlanningHorizon(horizon);
//         //             problem->getUnderlyingProblem()->setupDynamicsGenerator();
//         //             p_algo = makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount, lr, batch_size, num_max_steps, name);
//         //         }
//         //     }
//         //     else
//         //     {
//         //         //throw sdm::exception::Exception("Undefined algorithm type : " + algo_name);
//         //     }

//         //     return p_algo;
//         // }

//         /**
//          * @brief Get all available algorithms.
//          * 
//          * @return the list of available algorithms.
//          */
//         std::vector<std::string> available()
//         {
//             return {"hsvi", "qlearning"};
//         }

//     } // namespace algo
// } // namespace sdm
