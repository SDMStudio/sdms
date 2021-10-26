#include <sdm/utils/value_function/initializer/belief_2_occupancy_vf.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{

    Belief2OccupancyValueFunction::Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction> pomdp_value_function)
        : pomdp_value_function(pomdp_value_function)
    {
    }

    double Belief2OccupancyValueFunction::operator()(const std::shared_ptr<State> &state, const number &t)
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

    double Belief2OccupancyValueFunction::operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &belief_AND_action, const number &t)
    {
        auto belief = belief_AND_action.first;
        auto action = belief_AND_action.second;

        return getPOMDPValueFunction()->getQValueAt(belief, action, t);
    }

    double Belief2OccupancyValueFunction::operatorState(const std::shared_ptr<State> &, const number &)
    {
        throw sdm::exception::Exception("The initializer used is not available for this formalism !");
    }

    double Belief2OccupancyValueFunction::operatorBeliefState(const std::shared_ptr<BeliefInterface> &belief_state, const number &t)
    {
        return this->getPOMDPValueFunction()->getValueAt(belief_state, t);
    }

    double Belief2OccupancyValueFunction::operatorOccupancyState(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t)
    {
        double value = 0.0;

        // $sum_{o_{t}} p(o_{t} \mid s_{t} v_{t}^{pomdp}\left( x_{t} \mid o_{t} \right))$
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            auto belief = occupancy_state->getBeliefAt(joint_history);
            value += occupancy_state->getProbability(joint_history) * operatorBeliefState(belief, t);
        }
        return value;
    }

    bool Belief2OccupancyValueFunction::isPomdpAvailable()
    {
        return true;
    }

    bool Belief2OccupancyValueFunction::isMdpAvailable()
    {
        return false;
    }

    std::shared_ptr<ValueFunction> Belief2OccupancyValueFunction::getRelaxation()
    {
        return this->pomdp_value_function;
    }

    std::shared_ptr<ValueFunction> Belief2OccupancyValueFunction::getPOMDPValueFunction()
    {
        return this->pomdp_value_function;
    }

} // namespace sdm
