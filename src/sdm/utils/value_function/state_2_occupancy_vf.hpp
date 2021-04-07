#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/world/world_type.hpp>


namespace sdm
{
    template <typename TState, typename TOccupancyState>
    class State2OccupancyValueFunction : public BinaryFunction<TOccupancyState, number, double>
    {
    protected:
        std::shared_ptr<BinaryFunction<TState, number, double>> mdp_vf_;

    public:
        State2OccupancyValueFunction(std::shared_ptr<BinaryFunction<TState, number, double>> vf);

        template <bool is_mdp = std::is_any<typename WorldType<TOccupancyState>::type, DiscreteMDP,SerializedMDP<SerializedState, number>>::value>
        std::enable_if_t<is_mdp, double>
        operator()(const TOccupancyState &ostate, const number &tau);

        // template <bool is_serial_mdp = std::is_any<typename WorldType<TOccupancyState>::type, SerializedMDP<SerializedState, number>>::value>
        // std::enable_if_t<is_serial_mdp, double>
        // getValue(const TOccupancyState &ostate, const number &tau);

        // template <bool is_serial_mdp = std::is_any<typename WorldType<TOccupancyState>::type, SerializedMDP<SerializedState, number>>::value>
        // std::enable_if_t<!is_serial_mdp, double>
        // getValue(const TOccupancyState &ostate, const number &tau);

        template <bool is_mdp = std::is_any<typename WorldType<TOccupancyState>::type, DiscreteMDP,SerializedMDP<SerializedState, number>>::value>
        std::enable_if_t<!is_mdp, double>
        operator()(const TOccupancyState &ostate, const number &tau);
        
        double operator()(const TOccupancyState &ostate, const number &tau);
    };
} // namespace sdm

#include <sdm/utils/value_function/state_2_occupancy_vf.tpp>
