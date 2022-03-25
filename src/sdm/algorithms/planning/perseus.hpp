#pragma once

#include <sdm/types.hpp>
#include <sdm/algorithms/planning/dfsvi.hpp>

namespace sdm
{
    /**
     * @brief Random search value iteration algorithm
     * 
     * This algorithm is a tree search algorithm selecting randomly the branches 
     * to be explored (branches consists of next actons and observations).
     * By redefining the world, we will be able to solve different class of 
     * problems such as MDP, POMDP, DecPOMDP, and so on.
     * 
     */
    class Perseus : public DFSVI
    {
    public:
        /**
         * @brief Construct the RandomSearchValueIteration algorithm.
         * 
         * @param world the world to be solved
         * @param value_function the value function representation
         * @param error the error 
         * @param horizon the planning horizon
         * 
         */
        Perseus(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, number num_samples, double max_time, std::string name = "RandomSearchVI");

        /**
         * @brief Get the name of the algorithm as a string. 
         * 
         * This function will return the name of the algorithm as a string. 
         * It does not return the name of a specific instance (`name` attribute) 
         * but those of the general algorithm used (i.e. HSVI, QLearning, etc).
         * 
         * @return the algorithm name 
         */
        std::string getAlgorithmName();

    protected:
        /**
         * @brief Select the list of actions to explore.
         * 
         * This function can be inherited to build algorithms with different 
         * exploration heuristics
         * 
         * @return a list of actions 
         */
        std::vector<std::shared_ptr<Action>> selectActions(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Select the list of observations to explore.
         * 
         * This function can be inherited to build algorithms with different 
         * exploration heuristics
         * 
         * @return a list of observations 
         */
        std::vector<std::shared_ptr<Observation>> selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /** @brief The number of decision rules to sample and explore */
        number num_samples;
    };
}