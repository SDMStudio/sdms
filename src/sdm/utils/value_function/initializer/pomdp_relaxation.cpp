#include <sdm/utils/value_function/initializer/pomdp_relaxation.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{

    POMDPRelaxation::POMDPRelaxation(std::shared_ptr<ValueFunction> pomdp_value_function)
        : pomdp_value_function(pomdp_value_function)
    {
    }

    double POMDPRelaxation::getValueAt(const std::shared_ptr<State> &state, const number &t)
    {
        return operator()(state, t);
    }

    double POMDPRelaxation::operator()(const std::shared_ptr<State> &state, const number &t)
    {
        if (state == nullptr)
        {
            return getRelaxation()->operator()(state, t);
        }
        else if (auto ostate = sdm::isInstanceOf<OccupancyStateInterface>(state))
        {
            return getValueAtOccupancy(ostate, t);
        }
        else if (auto bstate = sdm::isInstanceOf<BeliefInterface>(state))
        {
            return getValueAtBelief(bstate, t);
        }
        else
        {
            return getValueAtState(state, t);
        }
    }

    double POMDPRelaxation::getValueAtState(const std::shared_ptr<State> &, const number &)
    {
        throw sdm::exception::Exception("The initializer used is not available for this formalism !");
    }

    double POMDPRelaxation::getValueAtBelief(const std::shared_ptr<BeliefInterface> &belief_state, const number &t)
    {
        return this->getPOMDPValueFunction()->getValueAt(belief_state, t);
    }

    double POMDPRelaxation::getValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t)
    {
        double value = 0.0;

        // $sum_{o_{t}} p(o_{t} \mid s_{t} v_{t}^{pomdp}\left( x_{t} \mid o_{t} \right))$
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            auto belief = occupancy_state->getBeliefAt(joint_history);
            value += occupancy_state->getProbability(joint_history) * getValueAtBelief(belief, t);
        }
        return value;
    }

    double POMDPRelaxation::getValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t, bool display)
    {
        double value = 0.0;

        // $sum_{o_{t}} p(o_{t} \mid s_{t} v_{t}^{pomdp}\left( x_{t} \mid o_{t} \right))$
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            auto belief = occupancy_state->getBeliefAt(joint_history);
            value += occupancy_state->getProbability(joint_history) * getValueAtBelief(belief, t);
        }
        return value;
    }

    double POMDPRelaxation::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, const number &t)
    {
        return this->getQValueAt(state_AND_action.first, state_AND_action.second, t);
    }

    double POMDPRelaxation::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &t)
    {
        if (state == nullptr)
        {
            return this->getQValueAtState(state, action, t);
        }
        else if (auto ostate = sdm::isInstanceOf<OccupancyStateInterface>(state))
        {
            return getQValueAtOccupancy(ostate, action->toDecisionRule(), t);
        }
        else if (auto bstate = sdm::isInstanceOf<BeliefInterface>(state))
        {
            return getQValueAtBelief(bstate, action, t);
        }
        else
        {
            return getQValueAtState(state, action, t);
        }
    }

    double POMDPRelaxation::getQValueAtState(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, const number &)
    {
        throw sdm::exception::Exception("The initializer used is not available for this formalism !");
    }

    double POMDPRelaxation::getQValueAtBelief(const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const number &t)
    {
        return this->getPOMDPValueFunction()->getQValueAt(belief, action, t);
    }

    double POMDPRelaxation::getQValueAtBelief(const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const number &t, bool display)
    {
        double r_b = this->getPOMDPValueFunction()->getWorld()->getReward(belief, action, t);

        double exp_next_v = 0.0;
        // For all observations from the controller point of view
        auto accessible_observation_space = this->getPOMDPValueFunction()->getWorld()->getObservationSpaceAt(belief, action, t);
        for (const auto &observation : *accessible_observation_space)
        {
            // Compute next state
            auto [next_state, state_transition_proba] = this->getPOMDPValueFunction()->getWorld()->getNextStateAndProba(belief, action, observation->toObservation(), t);

            // Update the next expected value at the next state
            exp_next_v += state_transition_proba * this->getPOMDPValueFunction()->getValueAt(next_state, t + 1);
        }
        return r_b + this->getPOMDPValueFunction()->getWorld()->getDiscount(t) * exp_next_v;
    }

    double POMDPRelaxation::getQValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, const number &t)
    {
        double value = 0.0;
        for (auto &jhistory : occupancy_state->getJointHistories())
        {
            auto action = occupancy_state->applyDR(decision_rule, jhistory);
            value += occupancy_state->getProbability(jhistory) * this->getQValueAtBelief(occupancy_state->getBeliefAt(jhistory), action, t);
        }
        return value;
    }

    bool POMDPRelaxation::isPomdpAvailable()
    {
        return true;
    }

    bool POMDPRelaxation::isMdpAvailable()
    {
        return false;
    }

    std::shared_ptr<ValueFunction> POMDPRelaxation::getRelaxation()
    {
        return this->pomdp_value_function;
    }

    std::shared_ptr<ValueFunction> POMDPRelaxation::getPOMDPValueFunction()
    {
        return this->pomdp_value_function;
    }

} // namespace sdm
