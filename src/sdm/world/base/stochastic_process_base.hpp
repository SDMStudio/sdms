/**
 * @file stochastic_process_base.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains the implementation of the Generic Stochastic process class. 
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/discrete_space.hpp>

/**
 * @brief this namespace groups all tools for sdms
 */
namespace sdm
{
    /**
     * @brief This class is the base class for stochastic processes including StochasticProcess class and DecisionProcess class. 
     * This class is usually used to have a base interface that provide required data structure as StateSpace.
     * 
     * @tparam TStateSpace the type of StateSpace to be used
     * @tparam TDistrib the type of start distribution to be used
     */
    template <typename TStateSpace, typename TDistrib>
    class StochasticProcessBase
    {
    public:
        using state_space_type = TStateSpace;
        using state_type = typename TStateSpace::value_type;

        StochasticProcessBase();
        StochasticProcessBase(std::shared_ptr<TStateSpace>);
        StochasticProcessBase(std::shared_ptr<TStateSpace>, TDistrib);

        state_type getInternalState() const;
        void setInternalState(state_type new_i_state);

        TDistrib getStartDistrib() const;
        void setStartDistrib(TDistrib);

        std::shared_ptr<TStateSpace> getStateSpace() const;
        void setStateSpace(std::shared_ptr<TStateSpace>);

    private:
        /**
         * @brief The internal state
         */
        state_type internal_state_;

    protected:
        /**
         * @brief The state space
         */
        std::shared_ptr<TStateSpace> state_space_;

        /**
         * @brief The initial state distribution
         */
        TDistrib start_distrib_;
    };
} // namespace sdm
#include <sdm/world/base/stochastic_process_base.tpp>