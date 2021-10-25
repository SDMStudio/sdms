#include <sdm/utils/value_function/initializer/state_2_occupancy_vf.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{
    State2OccupancyValueFunction::State2OccupancyValueFunction(std::shared_ptr<ValueFunction> vf) : mdp_vf_(vf)
    {
    }

    double State2OccupancyValueFunction::operator()(const std::shared_ptr<State> &state, number t)
    {
        if (state == nullptr)
        {
            return getRelaxation()->operator()(state, t);
        }
        else if (sdm::isInstanceOf<OccupancyStateInterface>(state))
        {
            return operatorOccupancyState(state->toOccupancyState(), t);
        }
        else if (sdm::isInstanceOf<BeliefInterface>(state))
        {
            return operatorBeliefState(state->toBelief(), t);
        }
        else
        {
            return operatorState(state, t);
        }
    }

    double State2OccupancyValueFunction::operatorState(const std::shared_ptr<State> &state, number t)
    {
        return this->getMDPValueFunction()->operator()(state, t);
    }

    double State2OccupancyValueFunction::operatorBeliefState(const std::shared_ptr<BeliefInterface> &belief_state, number t)
    {
        double value = 0.0;

        for (auto &state : belief_state->getStates())
        {
            value += belief_state->getProbability(state) * this->getMDPValueFunction()->getValueAt(state, t);
        }
        return value;
    }

    double State2OccupancyValueFunction::operatorOccupancyState(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
    {
        double value = 0.0;
        for (auto &jhistory : occupancy_state->getJointHistories())
        {
            double belief_value = this->operatorBeliefState(occupancy_state->getBeliefAt(jhistory), t);
            value += occupancy_state->getProbability(jhistory) * belief_value;
        }
        return value;
    }

    double State2OccupancyValueFunction::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t)
    {
        switch (state_AND_action.first->getTypeState())
        {
        case TypeState::STATE:
            return this->operatorQTableState(state_AND_action, t);
            break;

        case TypeState::BELIEF_STATE:
            return operatorQTableBelief(state_AND_action, t);
            break;
        default:
            return this->operatorQTableState(state_AND_action, t);
            break;
        }
    }

    double State2OccupancyValueFunction::operatorQTableState(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t)
    {
        auto state = state_AND_action.first;
        auto action = state_AND_action.second;

        return this->getMDPValueFunction()->getQValueAt(state, action, t);
    }
    double State2OccupancyValueFunction::operatorQTableBelief(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t)
    {
        auto state = state_AND_action.first->toBelief();
        auto action = state_AND_action.second;

        double value = 0.0;
        for (auto &ost : state->getStates())
        {
            value += state->getProbability(ost) * this->getMDPValueFunction()->getQValueAt(ost, action, t);
        }
        return value;
    }

    bool State2OccupancyValueFunction::isPomdpAvailable()
    {
        return false;
    }
    bool State2OccupancyValueFunction::isMdpAvailable()
    {
        return true;
    }

    std::shared_ptr<ValueFunction> State2OccupancyValueFunction::getRelaxation()
    {
        return this->mdp_vf_;
    }

    std::shared_ptr<ValueFunction> State2OccupancyValueFunction::getMDPValueFunction()
    {
        return this->mdp_vf_;
    }

} // namespace sdm
