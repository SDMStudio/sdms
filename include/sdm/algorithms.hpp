#pragma once

#include <random>

#include <sdm/algorithms/hsvi.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/worlds.hpp>
#include <sdm/tools.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/initializer.hpp>

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
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeMappedHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double discount = 0.99, double error = 0.001, int horizon = 0, int trials = 1000)
        {
            assert(((discount < 1) || (horizon > 0)));

            problem->setDiscount(discount);

            auto lb_init = std::make_shared<sdm::MinInitializer<TState, TAction>>(problem->getReward().getMinReward(), discount);
            auto ub_init = std::make_shared<sdm::MaxInitializer<TState, TAction>>(problem->getReward().getMaxReward(), discount);

            std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, ub_init));
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, lb_init));

            return std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error, trials);
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
        std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, double discount = 0.99, double error = 0.001, int horizon = 0, int trials = 1000)
        {
            if (algo_name == "mapped_hsvi" || algo_name == "tabular_hsvi" || algo_name == "hsvi")
            {
                // if (sdm::tools::hasExtension(problem_path, ".mdp") || sdm::tools::hasExtension(problem_path, ".MDP"))
                // {
                //     std::shared_ptr<SolvableByHSVI<number, number>> mdp = std::make_shared<DiscreteMDP>(problem_path);
                //     return makeMappedHSVI<number, number>(mdp, discount, error, horizon, trials);
                // }
                if (sdm::tools::hasExtension(problem_path, ".pomdp") || sdm::tools::hasExtension(problem_path, ".POMDP"))
                {
                    using TState = BeliefState;
                    using TAction = number;
                    using TObservation = number;

                    std::shared_ptr<SolvableByHSVI<TState, TAction>> beliefMDP = std::make_shared<BeliefMDP<TState, TAction, TObservation>>(problem_path);
                    return makeMappedHSVI<TState, TAction>(beliefMDP, discount, error, horizon, trials);
                }
                else if (sdm::tools::hasExtension(problem_path, ".dpomdp") || sdm::tools::hasExtension(problem_path, ".DPOMDP"))
                {
                    using TObservation = number;
                    using TState = number;

                    using TActionDescriptor = number;
                    using TStateDescriptor = HistoryTree_p<TObservation>;

                    using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
                    using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;
                    std::shared_ptr<SolvableByHSVI<TStatePrescriptor, TActionPrescriptor>> oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(problem_path, horizon);
                    return makeMappedHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, discount, error, horizon, trials);
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
            return {"mapped_hsvi", "tabular_hsvi"};
        }

    } // namespace algo
} // namespace sdm
