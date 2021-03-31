#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>

namespace sdm
{
    template <typename TBelief, typename TOccupancyState>
    class Belief2OccupancyValueFunction : public BinaryFunction<TOccupancyState, number, double>
    {
    protected:
        std::shared_ptr<BinaryFunction<TBelief, number, double>> pomdp_vf_;

        //J'aimerai bien créer cette variable avec la condition que c'est bien un SerializedOccupancyState
        // Je sais pas si cela est possible mais ça permettrai d'éviter de la faire passer de fonction en fonction
        //std::set<typename TOccupancyState::state_type> getAllStates;

        //De façon temporaire je vais garder cette variable
        std::set<TBelief> getAllBelief;


    public:

        template <bool is_mdp = std::is_same<TOccupancyState,SerializedOccupancyState<>>::value>
        std::enable_if_t<is_mdp, double>
        sawtooth(const TBelief &bstate, const number &tau);

        Belief2OccupancyValueFunction(std::shared_ptr<BinaryFunction<TBelief, number, double>> pomdp_vf);

        template <bool is_mdp = std::is_same<TOccupancyState,SerializedOccupancyState<>>::value>
        std::enable_if_t<is_mdp, double>
        operator()(const TOccupancyState &ostate, const number &tau);

        template <bool is_mdp = std::is_same<TOccupancyState,SerializedOccupancyState<>>::value>
        std::enable_if_t<!is_mdp, double>
        operator()(const TOccupancyState &ostate, const number &tau);
        
        double operator()(const TOccupancyState &ostate, const number &tau);

        MappedVector<TBelief,double> getMappedBelief(const number &tau);
    };

} // namespace sdm

#include <sdm/utils/value_function/belief_2_occupancy_vf.tpp>
