#pragma once

#include <sdm/types.hpp>
#include <sdm/algorithms/planning/pbvi.hpp>

namespace sdm
{
    /**
     * @brief [Perseus](https://arxiv.org/pdf/1109.2145.pdf) algorithm
     * 
     * This class contains the general algorithmic scheme of Perseus. 
     * By redefining the way the value function or the world are represented, we will be 
     * able to solve different class of problems with this algorithm (MDP, POMDP, DecPOMDP)..
     * 
     */
    class Perseus : public PBVI
    {
    public:
        /**
         * @brief Construct the Perseus algorithm.
         * 
         * @param world the world to be solved
         * @param value_function the value function representation
         * @param error the error 
         * @param horizon the planning horizon
         * 
         */
        Perseus(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, number num_samples, double max_time, std::string name = "pbvi");

    protected:

        /**
         * @brief Select the list of actions to explore.
         * 
         * This function can be inherited to build algorithms with different 
         * exploration heuristics
         * 
         * @return a list of actions 
         */
        std::shared_ptr<Space> selectActions(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Select the list of observations to explore.
         * 
         * This function can be inherited to build algorithms with different 
         * exploration heuristics
         * 
         * @return a list of observations 
         */
        std::shared_ptr<Space> selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Compute the next state.
         * 
         * @return the next state 
         */
        std::shared_ptr<Space> selectNextStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action,
                                                const std::shared_ptr<Observation> &observation, number t);

        /** @brief The number of decision rules to sample and explore */
        number num_samples;
    };
}