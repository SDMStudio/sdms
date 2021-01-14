#pragma once

#include <random>

#include <sdm/algorithms/hsvi.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/initializer.hpp>

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
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeMappedHSVI(std::shared_ptr<POSG> problem, double discount = 0.99, double error = 0.001, int horizon = 0)
        {
            assert(((discount < 1) || (horizon > 0)));

            problem->setDiscount(discount);

            auto lb_init = std::make_shared<sdm::MinInitializer<TState, TAction>>();
            auto ub_init = std::make_shared<sdm::MaxInitializer<TState, TAction>>();
            ;
            std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, ub_init));

            std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound(new sdm::MappedValueFunction<TState, TAction>(problem, horizon, lb_init));

            return std::make_shared<HSVI<TState, TAction>>(problem, lower_bound, upper_bound, horizon, error);
        }

    } // namespace algo
} // namespace sdm
