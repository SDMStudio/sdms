#include <sdm/utils/value_function/state_2_occupancy_vf.hpp>

namespace sdm
{

    template <typename TState, typename TOccupancyState>
    State2OccupancyValueFunction<TState, TOccupancyState>::State2OccupancyValueFunction(std::shared_ptr<BinaryFunction<TState, number, double>> vf) : mdp_vf_(vf)
    {
    }

    template <typename TState, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<is_mdp, double>
    State2OccupancyValueFunction<TState, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        return this->mdp_vf_->operator()(ostate, tau);
    }

    template <typename TState, typename TOccupancyState>
    template <bool is_mdp>
    std::enable_if_t<!is_mdp, double>
    State2OccupancyValueFunction<TState, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        double value = 0;
        for (auto &ost : ostate)
        {
            auto state = ostate.getState(ost.first);
            auto proba = ost.second;
            value += proba * this->mdp_vf_->operator()(state, tau);
        }
        return value;
    }

    template <typename TState, typename TOccupancyState>
    double State2OccupancyValueFunction<TState, TOccupancyState>::operator()(const TOccupancyState &ostate, const number &tau)
    {
        return this->operator()<>(ostate, tau);
    }

    template <typename TState, typename TOccupancyState>
    double State2OccupancyValueFunction<TState, TOccupancyState>::getValue(const TState &state, const number &tau)
    {
        return this->mdp_vf_->operator()(state, tau);
    }


} // namespace sdm
