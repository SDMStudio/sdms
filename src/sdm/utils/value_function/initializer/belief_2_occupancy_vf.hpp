#pragma once

#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>


namespace sdm
{
    class Belief2OccupancyValueFunction : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> pomdp_vf_;

    public:
        Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction> pomdp_vf);

        double operatorMPOMDP(const std::shared_ptr<State> &, const number &);

        double operatorNotMPOMDP(const std::shared_ptr<State> &, const number &);
        
        double operator()(const std::shared_ptr<State> &, const number &);

        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &, const number &);

        bool isPomdpAvailable();
        bool isMdpAvailable();

    };

    // template <typename TState, typename TAction, typename TObservation>
    // class Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>
    //     : public RelaxedValueFunction<TState,OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>
    // {
    // protected:
    //     std::shared_ptr<ValueFunction<TState, TAction>> pomdp_vf_;

    // public:
    //     Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf);

    //     double operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &ostate, const number &tau);
    //     double operator()(const Pair<TState, number> &ostate, const number &tau);

    //     bool isPomdpAvailable();
    //     bool isMdpAvailable();
    // };

} // namespace sdm
