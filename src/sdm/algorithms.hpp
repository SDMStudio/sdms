#pragma once

#include <random>

#include <sdm/exception.hpp>
#include <sdm/tools.hpp>
#include <sdm/worlds.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/core/states.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <sdm/utils/value_function/initializers.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/value_iteration.hpp>

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
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, std::string upper_bound_name, std::string lower_bound_name, std::string ub_init_name, std::string lb_init_name, double discount, double error, int horizon, int trials, std::string name)
        {
            assert(((discount < 1) || (horizon > 0)));

            // Set params in the environment
            problem->getUnderlyingProblem()->setDiscount(discount);
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
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound;
            if (lower_bound_name == "maxplan")
            {
                lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TState, TAction>>(problem, horizon, lb_init);
            }
            else
            {
                lower_bound = std::make_shared<sdm::MappedValueFunction<TState, TAction>>(problem, horizon, lb_init);
            }
            // std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound = std::make_shared<sdm::MappedValueFunction<TState, TAction>>(problem, horizon, lb_init);
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, ub_init));

            return std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error, trials, name);
        }

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
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::ValueIteration<TState, TAction>> makeValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double discount, double error, int horizon)
        {
            // Set params in the environment
            problem->getUnderlyingProblem()->setDiscount(discount);
            problem->getUnderlyingProblem()->setPlanningHorizon(horizon);

            // Increase the horizon for the value function if the problem is serialized
            if (problem->isSerialized())
            {
                horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
            }

            return std::make_shared<ValueIteration<TState, TAction>>(problem,discount, horizon, error);
        }

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
        std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, std::string formalism, std::string upper_bound, std::string lower_bound, std::string ub_init, std::string lb_init, double discount = 0.99, double error = 0.001, int horizon = 0, int trials = 1000, std::string name = "")
        {
            if ((algo_name == "hsvi"))
            {
                if ((formalism == "mdp") || (formalism == "MDP"))
                {
                    auto mdp = std::make_shared<DiscreteMDP>(problem_path);
                    mdp->getUnderlyingProblem()->setInternalState(0);

                    return makeHSVI<number, number>(mdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_mdphsvi" : name);
                }
                else if ((formalism == "pomdp") || (formalism == "POMDP"))
                {
                    using TState = BeliefState;
                    using TAction = number;
                    using TObservation = number;

                    auto pomdp = std::make_shared<DiscretePOMDP>(problem_path);
                    auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction, TObservation>>(pomdp);

                    return makeHSVI<TState, TAction>(beliefMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name);
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
                    return makeHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ohsvi" : name);
                }
                else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP"))
                {
                    using TState = SerializedState; //<number, number>;
                    using TAction = number;

                    auto mmdp = std::make_shared<DiscreteMMDP>(problem_path);
                    auto serialized_mdp = std::make_shared<SerializedMDP<TState, TAction>>(mmdp);
                    serialized_mdp->getUnderlyingProblem()->setInternalState(0);

                    return makeHSVI<TState, TAction>(serialized_mdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ext_mdphsvi" : name);
                }
                else if ((formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") || (formalism == "Extensive-DPOMDP"))
                {
                    using TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>;
                    using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

                    auto serialized_oMDP = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(problem_path, horizon);

                    return makeHSVI<TState, TAction>(serialized_oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ext_ohsvi" : name);
                }
            }
        }

        /**
         * @brief Get all available algorithms.
         * 
         * @return the list of available algorithms.
         */
        std::vector<std::string> available()
        {
            return {"hsvi"};
        }

    } // namespace algo
} // namespace sdm
