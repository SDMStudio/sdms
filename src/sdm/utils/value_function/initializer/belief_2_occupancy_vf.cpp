#include <sdm/utils/value_function/initializer/belief_2_occupancy_vf.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>

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
            auto belief = ostate->getBeliefAt(joint_history);
            value += ostate->getProbability(joint_history) * this->pomdp_vf_->getValueAt(belief, tau);
        }
        return value;
    }

    double Belief2OccupancyValueFunction::operator()(const std::shared_ptr<State> &state, const number &tau)
    {
        switch (state->getTypeState())
        {
        case TypeState::OCCUPANCY_STATE:
            return operatorMPOMDP(state, tau);
            break;

        default:
            throw sdm::exception::Exception("The initializer used is not available for this formalism !");
            break;
        }
    }

    double Belief2OccupancyValueFunction::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &belief_AND_action, const number &tau)
    {
        auto belief = belief_AND_action.first;
        auto action = belief_AND_action.second;

        return this->pomdp_vf_->template backup<double>(belief, action, tau);
    }

    bool Belief2OccupancyValueFunction::isPomdpAvailable()
    {
        return true;
    }

    bool Belief2OccupancyValueFunction::isMdpAvailable()
    {
        return false;
    }
} // namespace sdm
