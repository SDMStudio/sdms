/**
 * @file stochastic_process2.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief  Generic Stochastic process class
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <random>
#include <vector>
#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

/**
 * @brief this namespace groups all tools for sdms
 * 
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TStateSpace 
     * @tparam TDistrib 
     */
    template <typename TStateSpace, typename TDistrib>
    class StochasticProcess
    {
    public:
        using value_type = typename TStateSpace::value_type;

        StochasticProcess();
        StochasticProcess(TStateSpace);
        StochasticProcess(TStateSpace, TDistrib);

        value_type getInternalState() const;

        void setInternalState(value_type new_i_state);

        /**
         * @brief Init the process and return the initial internal state. This initial state is sampled from the "start" distribution.
         * 
         * @return value_type the sampled initial state
         */
        value_type init();

        TDistrib getStartDistrib() const;

        TStateSpace getStateSpace() const;

        void setStartDistrib(TDistrib);

        void setStateSpace() const;

        virtual TDistrib getProbaNextState(value_type)
        {
            throw sdm::exception::NotImplementedException();
        };

        /**
         * @brief Execute a step and return the next sampled state
         * 
         * @return value_type 
         */
        virtual value_type nextState()
        {
            throw sdm::exception::NotImplementedException();
        };

    private:
        /**
         * @brief The internal state
         */
        value_type internal_state_ = 0;

    protected:
        /**
         * @brief The state space
         */
        TStateSpace state_space_;

        /**
         * @brief The initial state distribution
         */
        TDistrib start_distrib_;
    };

    /**
     * @brief 
     * 
     * @tparam TInteger 
     */
    template <typename TInteger>
    class DiscreteStochasticProcess : public StochasticProcess<DiscreteSpace<TInteger>, std::discrete_distribution<TInteger>>
    {
    public:
        DiscreteStochasticProcess();
        DiscreteStochasticProcess(TInteger num_states);
        DiscreteStochasticProcess(std::initializer_list<double> start_distrib);
        DiscreteStochasticProcess(DiscreteSpace<TInteger> state_sp, std::discrete_distribution<TInteger> start_distrib);
        int getNumStates() const;
    };
} // namespace sdm
#include <sdm/world/stochastic_process.tpp>