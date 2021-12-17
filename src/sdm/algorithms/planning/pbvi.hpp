#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/algorithms/planning/value_iteration.hpp>

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
        PBVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, unsigned long long num_sample_states, double error, double time_max, std::string name, std::string type_sampling = "");

        void initialize();

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
        std::shared_ptr<State> sampleNextState(const std::shared_ptr<State> &state, number t);

        void initStateSpace();
        void initStateSpace2();

        /**
         * @brief Select the states that wil be used to update the value function.
         * 
         * @param h the horizon
         * @return the state spaces
         */
        std::shared_ptr<Space> selectStates(number h);

        /**
         * @brief Select one state.
         * 
         * @param t the time step
         * @return select one state
         */
        std::shared_ptr<State> selectOneState(number t);

        unsigned long long num_sample_states;

        std::vector<std::shared_ptr<Space>> sampled_state_space;

        std::string type_sampling;
    };
}