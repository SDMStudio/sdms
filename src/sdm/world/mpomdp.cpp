#include <sdm/world/mpomdp.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{

    MPOMDP::MPOMDP(const std::shared_ptr<Space> &state_space,
                   const std::shared_ptr<Space> &action_space,
                   const std::shared_ptr<Space> &obs_space,
                   const std::shared_ptr<RewardInterface> &reward,
                   const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
                   const std::shared_ptr<ObservationDynamicsInterface> &obs_dynamics,
                   const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
                   number horizon,
                   double discount,
                   Criterion criterion)
        : MDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion),
          POMDP(state_space, action_space, obs_space, reward, state_dynamics, obs_dynamics, start_distrib, horizon, discount, criterion),
          MMDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion)
    {
        this->num_agents_ = std::static_pointer_cast<MultiDiscreteSpace>(action_space)->getNumSpaces();
    }

    MPOMDP::~MPOMDP() {}

    std::shared_ptr<Space> MPOMDP::getObservationSpace(number t) const
    {
        return POMDP::getObservationSpace(t);
    }
    std::shared_ptr<Space> MPOMDP::getObservationSpace(number agent_id, number t) const
    {
        return std::static_pointer_cast<MultiDiscreteSpace>(this->getObservationSpace(t))->getSpace(agent_id);
    }

    std::string MPOMDP::toStdFormat()
    {
        if (this->getStateSpace()->isDiscrete() && this->getActionSpace()->isDiscrete())
        {

            auto state_space = std::static_pointer_cast<DiscreteSpace>(this->getStateSpace());
            auto action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getActionSpace());
            auto obs_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getObservationSpace(0));

            std::ostringstream res;
            number n_agents = this->getNumAgents();

            res << "agents: " << n_agents << std::endl;
            res << "discount: " << this->getDiscount() / 1.0 << std::endl;
            res << "values: \"" << ((this->criterion_ == Criterion::COST_MIN) ? "cost" : "reward") << "\"" << std::endl;
            res << "states: " << state_space->getNumItems() << std::endl;
            res << "start: \"uniform\"" << std::endl;

            // Action space
            res << "actions: \n";
            for (number ag = 0; ag < n_agents; ag++)
            {
                res << std::static_pointer_cast<DiscreteSpace>(action_space->getSpace(ag))->getNumItems() << "\n";
            }

            // Observation space
            res << "observations: \n";
            for (number ag = 0; ag < n_agents; ag++)
            {
                res << std::static_pointer_cast<DiscreteSpace>(obs_space->getSpace(ag))->getNumItems() << "\n";
            }

            std::cout << "1" << std::endl;
            for (const auto &state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    for (const auto &next_state : *state_space)
                    {

                        res << "T: ";
                        for (number agent = 0; agent < n_agents; ++agent)
                        {
                            auto action_agent_i = std::static_pointer_cast<Joint<std::shared_ptr<Item>>>(action)->get(agent);
                            res << std::static_pointer_cast<DiscreteSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                        }
                        res << ": " << state_space->getItemIndex(state)
                            << " : " << state_space->getItemIndex(next_state)
                            << " : " << this->getTransitionProbability(state->toState(),action->toAction(), next_state->toState())
                            << std::endl;
                    }
                }
            }
            std::cout << "2" << std::endl;

            for (const auto &next_state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    for (const auto &observation : *obs_space)
                    {
                        res << "O: ";
                        for (number agent = 0; agent < n_agents; ++agent)
                        {
                            auto action_agent_i = std::static_pointer_cast<Joint<std::shared_ptr<Item>>>(action)->get(agent);
                            res << std::static_pointer_cast<DiscreteSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                        }
                        res << ": " << state_space->getItemIndex(next_state) << " : ";
                        for (number agent = 0; agent < n_agents; ++agent)
                        {
                            auto obs_agent_i = std::static_pointer_cast<Joint<std::shared_ptr<Item>>>(observation)->get(agent);
                            res << std::static_pointer_cast<DiscreteSpace>(obs_space->getSpace(agent))->getItemIndex(obs_agent_i) << " ";
                        }
                        res << ": " << this->getObservationProbability(next_state->toState(),action->toAction(), next_state->toState(), observation->toObservation(), 0) << std::endl;
                    }
                }
            }

            std::cout << "3" << std::endl;
            for (const auto &state : *state_space)
            {
                for (const auto &action : *action_space)
                {
                    res << "R: ";
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        auto action_agent_i = std::static_pointer_cast<Joint<std::shared_ptr<Item>>>(action)->get(agent);
                        res << std::static_pointer_cast<DiscreteSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                    }
                    res << ": " << state_space->getItemIndex(state)
                        << " : " << this->getReward(state->toState(), action->toAction())
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