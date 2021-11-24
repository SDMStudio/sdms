#include <sdm/utils/value_function/initializer/mdp_relaxation.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{
    MDPRelaxation::MDPRelaxation(std::shared_ptr<ValueFunction> vf) : mdp_value_function(vf)
    {
    }

    double MDPRelaxation::getValueAt(const std::shared_ptr<State> &state, const number &t)
    {
        return operator()(state, t);
    }

    double MDPRelaxation::operator()(const std::shared_ptr<State> &state, const number &t)
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

    double MDPRelaxation::getValueAtState(const std::shared_ptr<State> &state, const number &t)
    {
        return this->getMDPValueFunction()->operator()(state, t);
    }

    double MDPRelaxation::getValueAtBelief(const std::shared_ptr<BeliefInterface> &belief_state, const number &t)
    {
        double value = 0.0;

        for (auto &state : belief_state->getStates())
        {
            value += belief_state->getProbability(state) * getValueAtState(state, t);
        }
        return value;
    }

    double MDPRelaxation::getValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t)
    {
        double value = 0.0;
        for (auto &jhistory : occupancy_state->getJointHistories())
        {
            double belief_value = this->getValueAtBelief(occupancy_state->getBeliefAt(jhistory), t);
            value += occupancy_state->getProbability(jhistory) * belief_value;
        }
        return value;
    }

    double MDPRelaxation::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, const number &t)
    {
        return this->getQValueAt(state_AND_action.first, state_AND_action.second, t);
    }

    double MDPRelaxation::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &t)
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

    double MDPRelaxation::getQValueAtState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &t)
    {
        return this->getMDPValueFunction()->getQValueAt(state, action, t);
    }

    double MDPRelaxation::getQValueAtBelief(const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const number &t)
    {
        double value = 0.0;
        for (auto &state : belief->getStates())
        {
            value += belief->getProbability(state) * this->getMDPValueFunction()->getQValueAt(state, action, t);
        }
        return value;
    }

    double MDPRelaxation::getQValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, const number &t)
    {
        double value = 0.0;
        for (auto &jhistory : occupancy_state->getJointHistories())
        {
            auto action = decision_rule->act(jhistory);
            double belief_value = this->getQValueAtBelief(occupancy_state->getBeliefAt(jhistory), action, t);
            value += occupancy_state->getProbability(jhistory) * belief_value;
        }
        return value;
    }

    bool MDPRelaxation::isPomdpAvailable()
    {
        return false;
    }
    bool MDPRelaxation::isMdpAvailable()
    {
        return true;
    }

    std::shared_ptr<ValueFunction> MDPRelaxation::getRelaxation()
    {
        return this->mdp_value_function;
    }

    std::shared_ptr<ValueFunction> MDPRelaxation::getMDPValueFunction()
    {
        return this->mdp_value_function;
    }

} // namespace sdm
