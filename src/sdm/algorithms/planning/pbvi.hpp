#pragma once

#include <math.h>
#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/algorithms/planning/vi.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    /**
     * @brief [Point-Based Value Iteration](https://www.ijcai.org/Proceedings/03/Papers/147.pdf)
     * and its extensions (ValueIteration, PBVI for DecPOMDP).
     * 
     * This class contains the general algorithmic scheme of Point-Based Value Iteration. 
     * By redefining the way the value function or the world are represented, we will be 
     * able to obtain different state-of-the-art algorithms such as: Value Iteration, Point-based
     * Value Iteration, etc.
     * 
     */
    class PBVI : public ValueIteration
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
        PBVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max = 1800, std::string name = "pbvi");

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
    };
}