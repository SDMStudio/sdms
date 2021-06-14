#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/struct/pair.hpp>


namespace sdm
{
    class State2OccupancyValueFunction : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> mdp_vf_;

    public:
        State2OccupancyValueFunction(std::shared_ptr<ValueFunction> vf);

        // template <bool is_mdp = std::is_any<typename WorldType<TOccupancyState>::type, DiscreteMDP, SerializedMMDP>::value>
        // std::enable_if_t<is_mdp, double>
        // operator()(const TOccupancyState &ostate, const number &tau);

        // template <bool is_mdp = std::is_any<typename WorldType<TOccupancyState>::type, DiscreteMDP, SerializedMMDP>::value>
        // std::enable_if_t<!is_mdp, double>
        // operator()(const TOccupancyState &ostate, const number &tau);

        double operator()(const std::shared_ptr<State> &ostate, const number &tau);

        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action> > &ostate, const number &tau);

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

#include <sdm/utils/value_function/state_2_occupancy_vf.tpp>
