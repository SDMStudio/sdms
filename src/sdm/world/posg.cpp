#include <sdm/world/posg.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{
    POSG::POSG() {}

    POSG::POSG(const std::shared_ptr<StateSpace> &state_space,
               const std::shared_ptr<ActionSpace> &action_space,
               const std::shared_ptr<ObservationSpace> &obs_space,
               const std::shared_ptr<RewardModel> &reward,
               const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
               const std::shared_ptr<ObservationDynamicsInterface> &obs_dynamics,
               const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
               number horizon,
               double discount,
               Criterion criterion) : MDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion),
                                      POMDP(state_space, action_space, obs_space, reward, state_dynamics, obs_dynamics, start_distrib, horizon, discount, criterion),
                                      MMDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion) {}

    double POSG::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return SG::getReward(state, action, t);
    }

    double POSG::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const
    {
        return SG::getReward(state, action, agent_id, t);
    }

    std::string POSG::toStdFormat()
    {
        if (this->getStateSpace()->isDiscrete() && this->getActionSpace()->isDiscrete())
        {
            auto state_space = std::static_pointer_cast<DiscreteStateSpace>(this->getStateSpace());
            auto action_space = std::static_pointer_cast<MultiDiscreteActionSpace>(this->getActionSpace());
            auto obs_space = std::static_pointer_cast<MultiDiscreteObservationSpace>(this->getObservationSpace(0));

            std::ostringstream res;
            number n_agents = this->getNumAgents();

            res << "agents: " << n_agents << std::endl;
            res << "discount: " << this->getDiscount() / 1.0 << std::endl;
            res << "values: \"" << ((this->criterion_ == Criterion::COST_MIN) ? "cost" : "reward") << "\"" << std::endl;
            res << "states: " << state_space->getNumItems() << std::endl;
            res << "start: " << std::endl;

            auto istate_end_iter = this->getStateSpace(0)->end();
            for (auto istate_iter = this->getStateSpace(0)->begin(); !istate_iter->equal(istate_end_iter); istate_iter = istate_iter->next())
            {
                auto initial_state = istate_iter->getCurrent();
                res << std::static_pointer_cast<DiscreteDistribution<std::shared_ptr<State>>>(this->getStartDistribution())->getProbability(initial_state) << " ";
            }

            // Action space
            res << "actions: \n";
            for (number ag = 0; ag < n_agents; ag++)
            {
                res << std::static_pointer_cast<DiscreteActionSpace>(action_space->getSpace(ag))->getNumItems() << "\n";
            }

            // Observation space
            res << "observations: \n";
            for (number ag = 0; ag < n_agents; ag++)
            {
                res << std::static_pointer_cast<DiscreteObservationSpace>(obs_space->getSpace(ag))->getNumItems() << "\n";
            }

            auto state_end_iter = state_space->end();
            for (auto state_iter = state_space->begin(); !state_iter->equal(state_end_iter); state_iter = state_iter->next())
            {
                auto state = state_iter->getCurrent();
                auto action_end_iter = action_space->end();
                for (auto action_iter = action_space->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
                {
                    auto action = action_iter->getCurrent();
                    auto next_state_end_iter = state_space->end();
                    for (auto next_state_iter = state_space->begin(); !next_state_iter->equal(next_state_end_iter); next_state_iter = next_state_iter->next())
                    {
                        auto next_state = next_state_iter->getCurrent();
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

            for (auto state_iter = state_space->begin(); !state_iter->equal(state_end_iter); state_iter = state_iter->next())
            {
                auto next_state = state_iter->getCurrent();
                auto action_end_iter = action_space->end();
                for (auto action_iter = action_space->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
                {
                    auto action = action_iter->getCurrent();
                    auto observation_end_iter = obs_space->end();
                    for (auto observation_iter = obs_space->begin(); !observation_iter->equal(observation_end_iter); observation_iter = observation_iter->next())
                    {
                        auto observation = observation_iter->getCurrent();
                        res << "O: ";
                        for (number agent = 0; agent < n_agents; ++agent)
                        {
                            auto action_agent_i = std::static_pointer_cast<JointAction>(action)->get(agent);
                            res << std::static_pointer_cast<DiscreteActionSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                        }
                        res << ": " << state_space->getItemIndex(next_state) << " : ";
                        for (number agent = 0; agent < n_agents; ++agent)
                        {
                            auto obs_agent_i = std::static_pointer_cast<JointObservation>(observation)->get(agent);
                            res << std::static_pointer_cast<DiscreteObservationSpace>(obs_space->getSpace(agent))->getItemIndex(obs_agent_i) << " ";
                        }
                        res << ": " << this->getObservationProbability(next_state, action, next_state, observation, 0) << std::endl;
                    }
                }
            }

            for (auto state_iter = state_space->begin(); !state_iter->equal(state_end_iter); state_iter = state_iter->next())
            {
                auto state = state_iter->getCurrent();
                auto action_end_iter = action_space->end();
                for (auto action_iter = action_space->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
                {
                    auto action = action_iter->getCurrent();
                    res << "R: ";
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        auto action_agent_i = std::static_pointer_cast<JointAction>(action)->get(agent);
                        res << std::static_pointer_cast<DiscreteActionSpace>(action_space->getSpace(agent))->getItemIndex(action_agent_i) << " ";
                    }
                    res << ": " << state_space->getItemIndex(state)
                        << " : ";
                    for (number agent = 0; agent < n_agents; ++agent)
                    {
                        res << this->getReward(state, action, agent, 0) << " ";
                    }

                    res << std::endl;
                }
            }
            return res.str();
        }
        else
        {
            return "No known Standard format for Continuous MDPs.";
        }
    }
} // namespace sdm
