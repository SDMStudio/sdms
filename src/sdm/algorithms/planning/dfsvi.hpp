#pragma once

#include <math.h>
#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/algorithms/planning/tsvi.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

namespace sdm
{
    /**
     * @brief Depth-first search value iteration.
     * 
     * This algorithm is comparable to the depth-first search algorithm applied to the tree
     * of state, action, observation transitions.
     * 
     */
    class DFSVI : public TSVI
    {
    public:
        /**
         * @brief Construct the PBVI algorithm.
         * 
         * @param world the world to be solved
         * @param value_function the value function representation
         * @param error the error 
         * @param horizon the planning horizon
         * 
         */
        DFSVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max = 1800, std::string name = "pbvi");

        /**
    	 * @brief Check the end of HSVI algo.
    	 * 
    	 * @param state the current state
    	 * @param cost_so_far the cost so far
    	 * @param h the current timestep
    	 * @return true if optimal is reached or number of trials is bigger than maximal number of trials
    	 * @return false elsewhere
    	 */
        bool stop(const std::shared_ptr<State> &state, double cost_so_far, number h);

        /**
		 * @brief Log execution variables in output streams.
		 */
        void logging();

        /**
         * @brief Update the value function at a specific state and time step.
         * 
         * This function will make an update on the value function using the 
         * bellman operator. this update is done for a particular state and time step.
         * 
         * @param state the state 
         * @param t the time step
         */
        void updateValue(const std::shared_ptr<State> &state, number t);

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
         * @brief Initialize the loggers.
         * 
         * The loggers are objects allowing to write the statistics relative to the execution of the 
         * algorithm in files. The files in question can have different formats (text, XML, CSV, etc.) 
         * depending on the desired processing afterwards.
         * 
         */
        void initLogger();

        /**
         * @brief Initialize a trial.
         * 
         * This function will initialize arguments required in a trial.
         * 
         */
        void initTrial();

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

        /** @brief The allowed error */
        double max_error;

        bool was_updated;
    };

    using DepthFirstSearchVI = DFSVI;
    using DepthFirstSearchValueIteration = DFSVI;
}