#pragma once

#include <random>

#include <sdm/exception.hpp>
#include <sdm/tools.hpp>
#include <sdm/worlds.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/algorithms/q_learning.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/core/states.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <sdm/utils/value_function/initializers.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/rl/exploration.hpp>

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
         * @param discount_factor the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @return pointer on HSVI instance
         */
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, std::string upper_bound_name, std::string lower_bound_name, std::string ub_init_name, std::string lb_init_name, double discount_factor, double error, number horizon, int trials, std::string name)
        {
            assert(((discount_factor < 1) || (horizon > 0)));

            // Set params in the environment
            problem->getUnderlyingProblem()->setDiscount(discount_factor);
            problem->getUnderlyingProblem()->setPlanningHorizon(horizon);

            // Increase the horizon for the value function if the problem is serialized
            if (problem->isSerialized())
            {
                horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
            }

            // Instanciate initializers
            auto lb_init = sdm::makeInitializer<TState, TAction>(lb_init_name); //std::make_shared<sdm::MinInitializer<TState, TAction>>();
            auto ub_init = sdm::makeInitializer<TState, TAction>(ub_init_name);

            // Instanciate bounds
            // std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound;
            // if (lower_bound_name == "maxplan")
            // {
            //     lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TState, TAction>>(problem, horizon, lb_init);
            // }
            // else
            // {
            auto lower_bound = std::make_shared<sdm::MappedValueFunction<TState, TAction>>(problem, horizon, lb_init);
            // }
            // std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound = std::make_shared<sdm::MappedValueFunction<TState, TAction>>(problem, horizon, lb_init);
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, ub_init));

            return std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error, trials, name);
        }

        /**
         * @brief Build an algorithm given his name and the configurations required. 
         * 
         * @tparam TState Type of the state.
         * @tparam TAction Type of the action.
         * @param algo_name the name of the algorithm to be built* 
         * @param problem the problem to be solved
         * @param discount_factor the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @param trials the maximum number of trials 
         * @return auto pointer on algorithm instance
         */
        std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, std::string formalism, std::string upper_bound, std::string lower_bound, std::string ub_init, std::string lb_init, double discount_factor = 0.99, double error = 0.001, number horizon = 0, int trials = 1000, std::string name = "")
        {
            if ((algo_name == "hsvi"))
            {
                if ((formalism == "mdp") || (formalism == "MDP"))
                {
                    auto mdp = std::make_shared<DiscreteMDP>(problem_path);
                    mdp->getUnderlyingProblem()->setInternalState(0);

                    return makeHSVI<number, number>(mdp, upper_bound, lower_bound, ub_init, lb_init, discount_factor, error, horizon, trials, (name == "") ? "tab_mdphsvi" : name);
                }
                else if ((formalism == "pomdp") || (formalism == "POMDP"))
                {
                    using TState = BeliefState;
                    using TAction = number;
                    using TObservation = number;

                    auto pomdp = std::make_shared<DiscretePOMDP>(problem_path);
                    auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction, TObservation>>(pomdp);

                    return makeHSVI<TState, TAction>(beliefMDP, upper_bound, lower_bound, ub_init, lb_init, discount_factor, error, horizon, trials, (name == "") ? "tab_hsvi" : name);
                }
                else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
                {
                    using TObservation = number;
                    using TState = number;

                    using TActionDescriptor = number;
                    using TStateDescriptor = HistoryTree_p<TObservation>;

                    using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
                    using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

                    auto oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(problem_path, horizon);
                    return makeHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, upper_bound, lower_bound, ub_init, lb_init, discount_factor, error, horizon, trials, (name == "") ? "tab_ohsvi" : name);
                }
                else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP"))
                {
                    using TState = SerializedState; //<number, number>;
                    using TAction = number;

                    auto mmdp = std::make_shared<DiscreteMMDP>(problem_path);
                    auto serialized_mdp = std::make_shared<SerializedMDP<TState, TAction>>(mmdp);
                    serialized_mdp->getUnderlyingProblem()->setInternalState(0);

                    return makeHSVI<TState, TAction>(serialized_mdp, upper_bound, lower_bound, ub_init, lb_init, discount_factor, error, horizon, trials, (name == "") ? "tab_ext_mdphsvi" : name);
                }
                else if ((formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") || (formalism == "Extensive-DPOMDP"))
                {
                    using TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>;
                    using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

                    auto serialized_oMDP = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(problem_path, horizon);

                    return makeHSVI<TState, TAction>(serialized_oMDP, upper_bound, lower_bound, ub_init, lb_init, discount_factor, error, horizon, trials, (name == "") ? "tab_ext_ohsvi" : name);
                }
            }
            else
            {
                throw sdm::exception::Exception("Undefined algorithm type : " + algo_name);
            }
        }

        template <typename TObservation, typename TAction>
        std::shared_ptr<sdm::QLearning<TObservation, TAction>> makeQLearning(std::shared_ptr<GymInterface<TObservation, TAction>> problem,
                                                                             std::string qvalue_name,
                                                                             std::string initializer_name,
                                                                             number horizon = 0,
                                                                             double discount_factor = 0.9,
                                                                             double lr = 0.01,
                                                                             double batch_size = 1,
                                                                             unsigned long num_max_steps = 100000,
                                                                             std::string name = "qlearning")
        {
            assert(((discount_factor < 1) || (horizon > 0)));

            // Instanciate initializers and qvalue functions
            auto initializer = std::make_shared<sdm::ZeroInitializer<TObservation, TAction>>();
            auto qvalue = std::make_shared<sdm::MappedQValueFunction<TObservation, TAction>>(horizon, lr, initializer);

            auto initializer_target = std::make_shared<sdm::ZeroInitializer<TObservation, TAction>>();
            auto target_qvalue = std::make_shared<sdm::MappedQValueFunction<TObservation, TAction>>(horizon, lr, initializer_target);

            // Instanciate exploration process
            auto exploration_process = std::make_shared<sdm::EpsGreedy<TObservation, TAction>>();

            return std::make_shared<QLearning<TObservation, TAction>>(
                problem, qvalue, target_qvalue, exploration_process, horizon, discount_factor, lr, batch_size, num_max_steps, name
            );
        }

        /**
         * @brief Build an algorithm given his name and the configurations required. 
         * 
         * @tparam TState Type of the state.
         * @tparam TAction Type of the action.
         * @param algo_name the name of the algorithm to be built* 
         * @param problem the problem to be solved
         * @param discount_factor the discount factor
         * @param error the accuracy
         * @param horizon the planning horizon
         * @param trials the maximum number of trials 
         * @return auto pointer on algorithm instance
         */
        std::shared_ptr<Algorithm> make(std::string algo_name,
                                        std::string problem_path,
                                        std::string formalism,
                                        std::string qvalue_name,
                                        std::string initializer_name,
                                        number horizon = 0,
                                        double discount_factor = 0.9,
                                        double lr = 0.01,
                                        double batch_size = 1,
                                        unsigned long num_max_steps = 100000,
                                        std::string name = "qlearning")
        {
            if ((algo_name == "qlearning"))
            {
                if ((formalism == "mdp") || (formalism == "MDP"))
                {
                    using env_type = DiscreteMDP;
                    auto problem = std::make_shared<env_type>(problem_path);
                    problem->setDiscount(discount_factor);
                    problem->setPlanningHorizon(horizon);
                    problem->setupDynamicsGenerator();
                    return makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount_factor, lr, batch_size, num_max_steps, name);
                }
                else if ((formalism == "pomdp") || (formalism == "POMDP"))
                {
                    using env_type = DiscretePOMDP;
                    auto problem = std::make_shared<env_type>(problem_path);
                    problem->setDiscount(discount_factor);
                    problem->setPlanningHorizon(horizon);
                    problem->setupDynamicsGenerator();
                    return makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount_factor, lr, batch_size, num_max_steps, name);
                }
                else if ((formalism == "beliefmdp") || (formalism == "BeliefMDP"))
                {
                    using env_type = BeliefMDP<>;
                    auto problem = std::make_shared<env_type>(problem_path);
                    problem->getUnderlyingProblem()->setDiscount(discount_factor);
                    problem->getUnderlyingProblem()->setPlanningHorizon(horizon);
                    problem->getUnderlyingProblem()->setupDynamicsGenerator();
                    return makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount_factor, lr, batch_size, num_max_steps, name);
                }
                else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
                {
                    using env_type = DiscreteDecPOMDP;
                    auto problem = std::make_shared<env_type>(problem_path);
                    problem->setDiscount(discount_factor);
                    problem->setPlanningHorizon(horizon);
                    problem->setupDynamicsGenerator();
                    return makeQLearning<env_type::observation_type, env_type::action_type>(problem, qvalue_name, initializer_name, horizon, discount_factor, lr, batch_size, num_max_steps, name);
                }
                else if ((formalism == "occupancymdp") || (formalism == "OccupancyMDP"))
                {
                    using TObservation = number;
                    using TState = number;

                    using TActionDescriptor = number;
                    using TStateDescriptor = HistoryTree_p<TObservation>;

                    using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
                    using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

                    using env_type = OccupancyMDP<TStatePrescriptor, TActionPrescriptor>;
                    auto problem = std::make_shared<env_type>(problem_path, horizon);

                    problem->getUnderlyingProblem()->setDiscount(discount_factor);
                    problem->getUnderlyingProblem()->setPlanningHorizon(horizon);
                    problem->getUnderlyingProblem()->setupDynamicsGenerator();
                    return makeQLearning<env_type::observation_type, env_type::action_type>(
                        problem, qvalue_name, initializer_name, horizon, discount_factor, lr, batch_size, num_max_steps, name
                    );
                }
                else if ((formalism == "hisoccupancymdp") || (formalism == "HISOccupancyMDP"))
                {
                    using TObservation = number;
                    using TState = number;

                    using TActionDescriptor = number;
                    using TStateDescriptor = HistoryTree_p<TObservation>;

                    using TActionPrescriptor = Pair<Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>, TActionDescriptor>;
                    using TStatePrescriptor = PrivateOccupancyState<TState, JointHistoryTree_p<TObservation>>;

                    using env_type = PrivateOccupancyMDP<TStatePrescriptor, TActionPrescriptor>;
                    // auto problem = std::make_shared<env_type>(problem_path, horizon);

                //     problem->getUnderlyingProblem()->setDiscount(discount_factor);
                //     problem->getUnderlyingProblem()->setPlanningHorizon(horizon);
                //     problem->getUnderlyingProblem()->setupDynamicsGenerator();
                //     return makeQLearning<env_type::observation_type, env_type::action_type>(
                //         problem, qvalue_name, initializer_name, horizon, discount_factor, lr, batch_size, num_max_steps, name
                //     );
                }
            }
            else
            {
                throw sdm::exception::Exception("Undefined algorithm type : " + algo_name);
            }
        }

        /**
         * @brief Get all available algorithms.
         * 
         * @return the list of available algorithms.
         */
        std::vector<std::string> available()
        {
            return {"hsvi", "qlearning"};
        }

    } // namespace algo
} // namespace sdm
