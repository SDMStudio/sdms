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
    double State2OccupancyValueFunction<TState, TOccupancyState>::operator()(const Pair<TState, number> &state_AND_action, const number &tau)
    {
        auto state = state_AND_action.first;
        auto action = state_AND_action.second;
        
        return this->mdp_vf_->getQValueAt(state, action, tau);
    }

    template <typename TState, typename TOccupancyState>
    bool State2OccupancyValueFunction<TState, TOccupancyState>::isPomdpAvailable()
    {
        return false;
    }

    template <typename TState, typename TOccupancyState>
    bool State2OccupancyValueFunction<TState, TOccupancyState>::isMdpAvailable()
    {
        return true;
    }


    // Belief Graph

    template <typename TState, typename TAction, typename TObservation>
    State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::State2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf) : mdp_vf_(vf)
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

    template <typename TState, typename TAction, typename TObservation>
    double State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const Pair<TState, number> &, const number &)
    {
        // auto ostate = ostate_AND_action.first;
        // auto action = ostate_AND_action.second;
        
        // return this->mdp_vf_->getQValueAt(ostate, action, tau);
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TObservation>
    bool State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isPomdpAvailable()
    {
        return false;
    }

    template <typename TState, typename TAction, typename TObservation>
    bool State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isMdpAvailable()
    {
        return true;
    }

} // namespace sdm
