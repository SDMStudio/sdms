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
        using state_space_type = TStateSpace;
        using state_type = typename state_space_type::value_type;

        StochasticProcess();
        StochasticProcess(TStateSpace);
        StochasticProcess(TStateSpace, TDistrib);

        state_type getInternalState() const;

        void setInternalState(state_type new_i_state);

        /**
         * @brief Init the process and return the initial internal state. This initial state is sampled from the "start" distribution.
         * 
         * @return state_type the sampled initial state
         */
        state_type init();

        TDistrib getStartDistrib() const;

        TStateSpace getStateSpace() const;

        void setStartDistrib(TDistrib);

        void setStateSpace(TStateSpace) const;

        virtual TDistrib getProbaNextState(state_type)
        {
            throw sdm::exception::NotImplementedException();
        }

        /**
         * @brief Execute a step and return the next sampled state
         * 
         * @return state_type 
         */
        virtual state_type nextState()
        {
            return this->getProbaNextState(this->getInternalState())(sdm::common::global_urng());
        }

        state_type step() {
            this->internal_state_ = this->nextState();
            return this->getInternalState();
        }

    private:
        /**
         * @brief The internal state
         */
        state_type internal_state_ = 0;

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
} // namespace sdm
#include <sdm/world/stochastic_process.tpp>