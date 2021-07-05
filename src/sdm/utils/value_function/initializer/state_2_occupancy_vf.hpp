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

        bool isPomdpAvailable();
        bool isMdpAvailable();
    };

    // template <typename TState, typename TAction, typename TObservation>
    // class State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>
    //     : public RelaxedValueFunction<TState,OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>
    // {
    // protected:
    //     std::shared_ptr<ValueFunction<TState, TAction>> mdp_vf_;

    // public:
    //     State2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf);
    //     double operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &ostate, const number &tau);
        
    //     double operator()(const Pair<TState, number> &ostate, const number &tau);

    //     bool isPomdpAvailable();
    //     bool isMdpAvailable();

    // };
} // namespace sdm