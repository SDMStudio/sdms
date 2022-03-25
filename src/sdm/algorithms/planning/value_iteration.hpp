#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/algorithms/planning/dp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    /**
     * @brief [Point-Based Value Iteration](https://www.ijcai.org/Proceedings/03/Papers/147.pdf)
     * and its extensions (ValueIteration, ValueIteration for DecPOMDP).
     *
     * This class contains the general algorithmic scheme of Point-Based Value Iteration.
     * By redefining the way the value function or the world are represented, we will be
     * able to obtain different state-of-the-art algorithms such as: Value Iteration, Point-based
     * Value Iteration, etc.
     *
     */
    class ValueIteration : public DynamicProgramming
    {
    public:
        /**
         * @brief Construct the ValueIteration algorithm.
         *
         * @param world the world to be solved
         * @param value_function the value function representation
         * @param error the error
         * @param horizon the planning horizon
         *
         */
        ValueIteration(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max, std::string name);

        /**
         * @brief Initialize the algorithm.
         *
         * Initialize the algorithm. This method must be called before `solve()`.
         * In fact, this will initialize the value function and other components required
         * by the algorithm.
         *
         */
        virtual void initialize();

        /**
         * @brief Planning procedure. Will attempt to solve the problem.
         *
         * This method will solve a problem using point-based value iteration.
         * Before calling `solve()`, the algorithm must be initialize with
         * `initialize()` function.
         *
         */
        virtual void solve();

        /**
         * @brief Explore a state.
         *
         * @param state the state to explore
         * @param cost_so_far the cost so far
         * @param t the time step of the exploration
         */
        virtual void doTrial();
        virtual void doOneStepTrial(number t);

        /**
         * @brief Check if the stop criterion is reached or not.
         *
         * In value iteration algorithms, the stop criterion is reached when the
         * distance between old value and new value is lower than the error.
         *
         * @return true the stop criterion is reached
         * @return false the stop criterion is not reached yet
         */
        virtual bool stop();

        /**
         * @brief Log execution variables in output streams.
         */
        void logging();

        /**
         * @brief Test the current policy and display the reward obtained.
         */
        virtual void test();

        /**
         * @brief Save the value function.
         */
        virtual void save();

        /**
         * @brief Get the value function.
         */
        virtual std::shared_ptr<ValueFunction> getValueFunction();

        /**
         * @brief Update the value function at a specific state and time step.
         *
         * This function will make an update on the value function using the
         * bellman operator. this update is done for a particular state and time step.
         *
         * @param state the state
         * @param t the time step
         */
        virtual void updateValue(const std::shared_ptr<State> &state, number t);

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

        std::shared_ptr<ValueFunction> getTmpValueFunction();

        void setTmpValueFunction(const std::shared_ptr<ValueFunction> &tmp_vf);

    protected:
        /**
         * @brief Select the states that wil be used to update the value function.
         *
         * @param h the horizon
         * @return the state spaces
         */
        virtual std::shared_ptr<Space> selectStates(number h);

        /**
         * @brief Initialize a trial.
         *
         * This function will initialize arguments required in a trial.
         *
         */
        virtual void initTrial();

        /**
         * @brief Initialize the logger
         */
        void initLogger();

        /** @brief The value function */
        std::shared_ptr<ValueFunction> value_function, tmp_value_function;

        /** @brief The time max before leaving execution */
        double time_max, max_error;

        bool was_updated;
    };
}