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
#include <sdm/utils/value_function/initializers.hpp>

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
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeMappedHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double discount = 0.99, double error = 0.001, int horizon = 0, int trials = 10000, std::string name = "tab_hsvi")
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
            //auto lb = std::make_shared<sdm::MMDPInitializer<TState, TAction>>();
            //std::shared_ptr<sdm::ValueFunction<TState, TAction>> lb_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, lb));
            //lb_bound->initialize();

            // Instanciate initializers
            auto lb_init = sdm::makeInitializer<TState, TAction>("MinInitializer");//std::make_shared<sdm::MinInitializer<TState, TAction>>();
            auto ub_init = sdm::makeInitializer<TState, TAction>("MaxInitializer");//std::make_shared<sdm::MaxInitializer<TState, TAction>>();

            // Instanciate bounds
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, ub_init));
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, lb_init));

            std::cout<<lower_bound->str()<<"\n";
            std::cout<<upper_bound->str()<<"\n";

            return std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error, trials, name);
        }

        // /**
        //  * @brief
        //  *
        //  * @tparam TState
        //  * @tparam TAction
        //  * @param problem
        //  * @param discount
        //  * @param error
        //  * @param horizon
        //  * @param trials
        //  * @return std::shared_ptr<sdm::HSVI<TState, TAction>>
        //  */
        // template <typename TState, typename TAction>
        // std::shared_ptr<sdm::HSVI<TState, TAction>> makeHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double discount = 0.99, double error = 0.001, int horizon = 0, int trials = 1000)
        // {
        //     assert(((discount < 1) || (horizon > 0)));

        //     problem->setDiscount(discount);

        //     auto lb_init = std::make_shared<sdm::MinInitializer<TState, TAction>>(problem->getReward().getMinReward(), discount);
        //     auto ub_init = std::make_shared<sdm::MaxInitializer<TState, TAction>>(problem->getReward().getMaxReward(), discount);

        //     std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MaxPlanValueFunction<TState, TAction>(problem, horizon, ub_init));
        //     std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound(new sdm::MaxPlanValueFunction<TState, TAction>(problem, horizon, lb_init));

        //     return std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error, trials);
        // }

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
        std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, std::string formalism, double discount = 0.99, double error = 0.001, int horizon = 0, int trials = 1000, std::string name = "")
        {
            if ((algo_name == "tabular_hsvi") || (algo_name == "mapped_hsvi"))
            {
                if ((formalism == "mdp") || (formalism == "MDP"))
                {
                    auto mdp = std::make_shared<DiscreteMDP>(problem_path);
                    mdp->getUnderlyingProblem()->setInternalState(0);

                    return makeMappedHSVI<number, number>(mdp, discount, error, horizon, trials, (name == "") ? "tab_mdphsvi" : name);
                }
                else if ((formalism == "pomdp") || (formalism == "POMDP"))
                {
                    using TState = BeliefState;
                    using TAction = number;
                    using TObservation = number;

                    auto pomdp = std::make_shared<DiscretePOMDP>(problem_path);
                    auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction, TObservation>>(pomdp);

                    return makeMappedHSVI<TState, TAction>(beliefMDP, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name);
                }
                else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
                {
                    using TObservation = number;
                    using TState = number;

                    using TActionDescriptor = number;
                    using TStateDescriptor = HistoryTree_p<TObservation>;

                    using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
                    using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;
                    // auto dpomdp = std::make_shared<DiscreteDecPOMDP>(problem_path);
                    auto oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(problem_path, horizon);
                    return makeMappedHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, discount, error, horizon, trials, (name == "") ? "tab_ohsvi" : name);
                }
                else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP"))
                {
                    using TState = SerializedState<number,number>;
                    using TAction = number;

                    auto serialized_mdp = std::make_shared<SerializedMDP<TState, TAction>>(problem_path);
                    serialized_mdp->getUnderlyingProblem()->setInternalState(0);

                    return makeMappedHSVI<TState, TAction>(serialized_mdp, discount, error, horizon, trials, (name == "") ? "tab_ext_mdphsvi" : name);
                }
                else if ((formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") || (formalism == "Extensive-DPOMDP"))
                {
                    using TState = SerializedOccupancyState<SerializedState<number,number>,JointHistoryTree_p<number>>;
                    using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

                    auto serialized_oMDP = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(problem_path, horizon);

                    return makeMappedHSVI<TState, TAction>(serialized_oMDP, discount, error, horizon, trials, (name == "") ? "tab_ext_ohsvi" : name);
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
            return {"tabular_hsvi"};
        }

    } // namespace algo
} // namespace sdm
