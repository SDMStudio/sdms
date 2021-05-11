#include <sdm/utils/value_function/state_2_occupancy_vf.hpp>

namespace sdm
{

    template <typename TState, typename TOccupancyState>
    State2OccupancyValueFunction<TState, TOccupancyState>::State2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, number>> vf) : mdp_vf_(vf)
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
    double State2OccupancyValueFunction<TState, TOccupancyState>::getQValueAt(const TState &state,const number &action, const number &tau)
    {
        return this->mdp_vf_->getQValueAt(state,action,tau);
    }

    template <typename TState, typename TAction, typename TObservation>
    State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::State2OccupancyValueFunction(std::shared_ptr<BinaryFunction<TState, number, double>> vf) : mdp_vf_(vf)
    {
    }

    template <typename TState, typename TAction, typename TObservation>
    double State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &occupancy_state, const number &t)
    {
        double value = 0;
        for (auto &pair_belief_history_proba : occupancy_state)
        {
            auto belief = occupancy_state.getState(pair_belief_history_proba.first);
            auto proba = pair_belief_history_proba.second;
            for (number state = 0; state < belief->getData().size(); state++)
            {
                value += proba * belief->getData()[state] * (*this->mdp_vf_)(state, t);
            }
        }
        return value;
    }

} // namespace sdm
