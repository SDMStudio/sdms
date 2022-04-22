#include <sdm/world/mmdp.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{
    MMDP::MMDP() {}

    MMDP::MMDP(const std::shared_ptr<StateSpace> &state_space,
               const std::shared_ptr<ActionSpace> &action_space,
               const std::shared_ptr<RewardModel> &reward,
               const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
               const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
               number horizon,
               double discount,
               Criterion criterion)
        : MDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion)
    {
        this->num_agents_ = std::static_pointer_cast<MultiDiscreteActionSpace>(action_space)->getNumSpaces();
    }

    std::shared_ptr<ActionSpace> MMDP::getActionSpace(number t) const
    {
        return MDP::getActionSpace(t);
    }
    std::shared_ptr<ActionSpace> MMDP::getActionSpace(number agent_id, number t) const
    {
        return std::static_pointer_cast<MultiDiscreteActionSpace>(this->getActionSpace(t))->getSpace(agent_id);
    }

    std::string MMDP::toStdFormat()
    {
        if (this->getStateSpace()->isDiscrete() && this->getActionSpace()->isDiscrete())
        {

            auto state_space = std::static_pointer_cast<DiscreteStateSpace>(this->getStateSpace());
            auto action_space = std::static_pointer_cast<MultiDiscreteActionSpace>(this->getActionSpace());

            std::ostringstream res;
            number n_agents = this->getNumAgents();

            res << "agents: " << n_agents << std::endl;
            res << "discount: " << this->getDiscount() / 1.0 << std::endl;
            res << "values: \"" << ((this->criterion_ == Criterion::COST_MIN) ? "cost" : "reward") << "\"" << std::endl;
            res << "states: " << state_space->getNumItems() << std::endl;
            res << "start: \"uniform\"" << std::endl;
            res << "actions: \n";
            for (number ag = 0; ag < n_agents; ag++)
            {
                res << std::static_pointer_cast<DiscreteActionSpace>(action_space->getSpace(ag))->getNumItems() << "\n";
            }

            for (const auto &state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    for (const auto &next_state : *state_space)
                    {

                        res << "T: ";
                        for (number agent = 0; agent < n_agents; ++agent)
                        {
                            auto action_agent_i = std::static_pointer_cast<JointAction>(action)->get(agent);
                            res << std::static_pointer_cast<DiscreteActionSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                        }
                        res << ": " << state_space->getItemIndex(state)
                            << " : " << state_space->getItemIndex(next_state)
                            << " : " << this->getTransitionProbability(state, action, next_state)
                            << std::endl;
                    }
                }
            }

            for (const auto &state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    res << "R: ";
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        auto action_agent_i = std::static_pointer_cast<JointAction>(action)->get(agent);
                        res << std::static_pointer_cast<DiscreteActionSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                    }
                    res << ": " << state_space->getItemIndex(state)
                        << " : " << this->getReward(state, action)
                        << std::endl;
                }
            }
            return res.str();
        }
        else
        {
            return "No known Standard format for Continuous MDPs.";
        }
    }

}