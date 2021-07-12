#include <sdm/utils/value_function/initializer/state_2_occupancy_vf.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{
    State2OccupancyValueFunction::State2OccupancyValueFunction(std::shared_ptr<ValueFunction> vf) : mdp_vf_(vf)
    {
    }

    double State2OccupancyValueFunction::operatorState(const std::shared_ptr<State> &state, const number &tau)
    {
        return this->mdp_vf_->operator()(state, tau);
    }

    double State2OccupancyValueFunction::operatorBelief(const std::shared_ptr<State> &state, const number &tau)
    {
        double value = 0;
        auto ostate = state->toBelief();

        for (auto &ost : ostate->getStates())
        {
            value += ostate->getProbability(ost) * this->mdp_vf_->operator()(ost, tau);
        }
        return value;
    }

    double State2OccupancyValueFunction::operatorOccupancy(const std::shared_ptr<State> &state, const number &tau)
    {
        double value = 0;
        auto ostate = state->toOccupancyState();

        for (auto &jhistory : ostate->getJointHistories())
        {
            double belief_value = this->operator()(ostate->getBeliefAt(jhistory), tau);
            value += ostate->getProbability(jhistory) * belief_value;
        }
        return value;
    }

    double State2OccupancyValueFunction::operator()(const std::shared_ptr<State> &state, const number &tau)
    {
        switch (state->getTypeState())
        {
        case TypeState::STATE:
            return operatorState(state, tau);
            break;

        case TypeState::BELIEF_STATE:
            return operatorBelief(state, tau);
            break;

        case TypeState::OCCUPANCY_STATE:
            return operatorOccupancy(state, tau);
            break;

        default:
            return operatorState(state, tau);
            break;
        }
        // return this->operator()<>(ostate, tau);
    }

    double State2OccupancyValueFunction::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, const number &tau)
    {
        auto state = state_AND_action.first;
        auto action = state_AND_action.second;

        return 0;
        // return this->mdp_vf_->getQValueAt(state, action, tau);
    }
    bool State2OccupancyValueFunction::isPomdpAvailable()
    {
        return false;
    }
    bool State2OccupancyValueFunction::isMdpAvailable()
    {
        return true;
    }

    // Belief Graph

    // template <typename TState, typename TAction, typename TObservation>
    // State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::State2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf) : mdp_vf_(vf)
    // {
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // double State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &occupancy_state, const number &t)
    // {
    //     double value = 0;
    //     for (auto &pair_belief_history_proba : occupancy_state)
    //     {
    //         auto belief = occupancy_state.getState(pair_belief_history_proba.first);
    //         auto proba = pair_belief_history_proba.second;
    //         for (number state = 0; state < belief->getData().size(); state++)
    //         {
    //             value += proba * belief->getData()[state] * (*this->mdp_vf_)(state, t);
    //         }
    //     }
    //     return value;
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // double State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const Pair<TState, number> &ostate_AND_action, const number &tau)
    // {
    //     auto ostate = ostate_AND_action.first;
    //     auto action = ostate_AND_action.second;

    //     // return this->mdp_vf_->getQValueAt(ostate, action, tau);
    //     throw sdm::exception::NotImplementedException();
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // bool State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isPomdpAvailable()
    // {
    //     return false;
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // bool State2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isMdpAvailable()
    // {
    //     return true;
    // }

} // namespace sdm
