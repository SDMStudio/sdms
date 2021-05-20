#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/world/world_type.hpp>
#include <sdm/utils/struct/pair.hpp>



namespace sdm
{
    template <typename TBelief, typename TOccupancyState>
    class Belief2OccupancyValueFunction : public RelaxedValueFunction<TBelief,TOccupancyState>
    {
    protected:
        std::shared_ptr<ValueFunction<TBelief, number>> pomdp_vf_;

    public:
        Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TBelief, number>> pomdp_vf);

        template <bool is_solving_dpomdp = std::is_any<typename WorldType<TOccupancyState>::type, OccupancyMDP<>,SerializedOccupancyMDP<>>::value>
        std::enable_if_t<is_solving_dpomdp, double>
        sawtooth(const TBelief &bstate, const number &tau);

        template <bool is_solving_dpomdp = std::is_any<typename WorldType<TOccupancyState>::type, OccupancyMDP<>,SerializedOccupancyMDP<>>::value>
        std::enable_if_t<is_solving_dpomdp, double>
        operator()(const TOccupancyState &ostate, const number &tau);
        

        template <bool is_solving_dpomdp = std::is_any<typename WorldType<TOccupancyState>::type, OccupancyMDP<>,SerializedOccupancyMDP<>>::value>
        std::enable_if_t<!is_solving_dpomdp, double>
        operator()(const TOccupancyState &ostate, const number &tau);
        
        double operator()(const TOccupancyState &ostate, const number &tau);

        double operator()(const Pair<TBelief, number> &ostate, const number &tau);

        bool isPomdpAvailable();
        bool isMdpAvailable();

    };

    template <typename TState, typename TAction, typename TObservation>
    class Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>
        : public RelaxedValueFunction<TState,OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>
    {
    protected:
        std::shared_ptr<ValueFunction<TState, TAction>> pomdp_vf_;

    public:
        Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf);

        double operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &ostate, const number &tau);
        double operator()(const Pair<TState, number> &ostate, const number &tau);

        bool isPomdpAvailable();
        bool isMdpAvailable();
    };

} // namespace sdm

#include <sdm/utils/value_function/belief_2_occupancy_vf.tpp>
