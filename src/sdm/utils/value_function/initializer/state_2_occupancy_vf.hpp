#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    class State2OccupancyValueFunction : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> mdp_vf_;

    public:
        State2OccupancyValueFunction(std::shared_ptr<ValueFunction> );

        double operatorState(const std::shared_ptr<State> &, const number &);
        double operatorBelief(const std::shared_ptr<State> &, const number &);
        double operatorOccupancy(const std::shared_ptr<State> &, const number &);

        double operator()(const std::shared_ptr<State> &, const number &);

        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action> > &, const number &);
        double operatorQTableState(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>>  &state_AND_action, const number &tau);
        double operatorQTableBelief(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>>  &state_AND_action, const number &tau);


        bool isPomdpAvailable();
        bool isMdpAvailable();

        std::shared_ptr<ValueFunction> getRelaxation();

    };
} // namespace sdm