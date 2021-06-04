/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <sdm/types.hpp>
#include <sdm/world/serialized_mmdp.hpp>

namespace sdm
{
    SerializedMMDP::SerializedMMDP(const std::shared_ptr<MDPInterface> &mmdp) : mmdp_(mmdp)
    {
    }

    number SerializedMMDP::getAgentId(number t)
    {
        return (t % this->getNumAgents());
    }

    bool SerializedMMDP::isLastAgent(number t)
    {
        return (this->getAgentId(t) != (this->getNumAgents() - 1));
    }

    double SerializedMMDP::getDiscount(number t) const
    {
        return (((t + 1) % this->getNumAgents()) == 0) ? this->mmdp_->getDiscount(t / this->getNumAgents()) : 1.0;
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> SerializedMMDP::getStartDistribution() const
    {
        return this->mmdp_->getStartDistribution();
    }

    std::set<std::shared_ptr<State>> SerializedMMDP::getAllStates(number t) const
    {
    }

    std::set<std::shared_ptr<State>> SerializedMMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
    }

    std::vector<std::shared_ptr<Action>> SerializedMMDP::getAllActions(number t) const
    {
        return std::static_pointer_cast<Joint<std::shared_ptr<Space<Action>>>>(this->mmdp_->getActionSpace(t))->get(this->getAgentId(t))->getAll();
    }

    double SerializedMMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        if (!this->isLastAgent(t))
        {
            return 0;
        }
        else
        {
            std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
            auto joint_action = serialized_state->getAction();
            joint_action.push_back(serial_action);

            return this->mmdp_->getReward(serialized_state.getState(), joint_action, t);
        }
    }

    double SerializedMMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::static_pointer_cast<SerializedState>(next_state);

        Joint<std::shared_ptr<Action>> all_action = serialized_state->getAction();
        all_action.push_back(action);

        if (!this->isLastAgent(t))
        {
            // If the next serialized_state and the current serialized_state don't have the same hidden or it's not the same player to act, then the dynamics is impossible
            return !(((serialized_state->getCurrentAgentId() + 1) != next_serialized_state->getCurrentAgentId()) ||
                     (serialized_state->getState() != next_serialized_state->getState()) ||
                     (next_serialized_state->getAction() != all_action));
        }
        else
        {
            return this->mmdp_->getTransitionProbability(serialized_state->getState(), all_action, next_serialized_state->getState());
        }
    }

    double SerializedMMDP::getMinReward(number t) const
    {
    }

    double SerializedMMDP::getMaxReward(number t) const
    {
    }

} // namespace sdm