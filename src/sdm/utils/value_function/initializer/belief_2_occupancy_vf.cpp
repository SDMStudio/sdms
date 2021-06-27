#include <sdm/utils/value_function/initializer/belief_2_occupancy_vf.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{

    Belief2OccupancyValueFunction::Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction> pomdp_vf) : pomdp_vf_(pomdp_vf)
    {
    }

    double Belief2OccupancyValueFunction::operatorMPOMDP(const std::shared_ptr<State> &state, const number &tau)
    {
        double value = 0;
        auto ostate = state->toOccupancyState();
        // $sum_{o_{\tau}} p(o_{\tau} \mid s_{\tau} v_{\tau}^{pomdp}\left( x_{\tau} \mid o_{\tau} \right))$

        for (const auto &joint_history : ostate->getJointHistories())
        {
            // std::shared_ptr<BeliefInterface> belief = ostate->createBeliefWeighted(joint_history);
            // value += ostate->getProbabilityOverJointHistory(joint_history) * this->pomdp_vf_->getValueAt(belief, tau);
            for(const auto &belief : ostate->getBeliefsAt(joint_history))
            {
                value += ostate->getProbability(joint_history,belief) * this->pomdp_vf_->getValueAt(belief, tau);
            }

        }
        return value;
    }

    double Belief2OccupancyValueFunction::operatorNotMPOMDP(const std::shared_ptr<State> &, const number &)
    {
        throw sdm::exception::Exception("The initializer used is not available for this formalism !");
    }

    double Belief2OccupancyValueFunction::operator()(const std::shared_ptr<State> &state, const number &tau)
    {
        switch (state->getTypeState())
        {
        case TypeState::STATE :
            return operatorNotMPOMDP(state,tau);
            break;

        case TypeState::BELIEF_STATE :
            return operatorNotMPOMDP(state,tau);
            break;

        case TypeState::OCCUPANCY_STATE :
            return operatorMPOMDP(state,tau);
            break;

        default:
            return operatorNotMPOMDP(state,tau);
            break;
        }
    }

    double Belief2OccupancyValueFunction::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &belief_AND_action, const number &)
    {
        auto belief = belief_AND_action.first;
        auto action = belief_AND_action.second;

        // return this->pomdp_vf_->getQValueAt(belief, action, tau);
    }

    bool Belief2OccupancyValueFunction::isPomdpAvailable()
    {
        return true;
    }

    bool Belief2OccupancyValueFunction::isMdpAvailable()
    {
        return false;
    }

    // ****** Belief Graph  ****** A faire ********* // 
    // template <typename TState, typename TAction, typename TObservation>
    // Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction<TState, TAction>> vf) : pomdp_vf_(vf)
    // {
    //     throw sdm::exception::NotImplementedException();
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // double Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>> &, const number &)
    // {
    //     throw sdm::exception::NotImplementedException();
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // double Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::operator()(const Pair<TState,number> &, const number &)
    // {
    //     throw sdm::exception::NotImplementedException();
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // bool Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isPomdpAvailable()
    // {
    //     return true;
    // }

    // template <typename TState, typename TAction, typename TObservation>
    // bool Belief2OccupancyValueFunction<TState, OccupancyState<BeliefStateGraph_p<TAction, TObservation>, JointHistoryTree_p<TObservation>>>::isMdpAvailable()
    // {
    //     return false;
    // }
} // namespace sdm
