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
#include <sdm/exception.hpp>
#include <sdm/world/iserial_mmdp.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>
#include <sdm/world/registry.hpp>

namespace sdm
{
    ISerialMMDP::ISerialMMDP()
    {
    }

    ISerialMMDP::ISerialMMDP(Config config) : ISerialMMDP(std::dynamic_pointer_cast<MMDPInterface>(sdm::world::createFromConfig(config)), config)
    {
    }

    ISerialMMDP::ISerialMMDP(const std::shared_ptr<MMDPInterface> &mmdp, Config config)
    {
        this->mmdp_ = mmdp;
    }

    ISerialMMDP::~ISerialMMDP() {}

    std::shared_ptr<Distribution<std::shared_ptr<State>>> ISerialMMDP::getStartDistribution() const
    {
        return this->distribution_serial;
    }

    void ISerialMMDP::createDistribution()
    {
        // Create the distribution of the serial State

        auto discrete_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        auto mmdp_distribution = this->mmdp_->getStartDistribution();

        // The Serial state at 0 are the same state of the mmdp without the vector null of action
        // Consequently, we just take the distrubution of the mmdp to the new serial state
        for (const auto &state : *this->getStateSpace(0))
        {
            discrete_distribution->setProbability(state->toState(), mmdp_distribution->getProbability(state->toState()->toSerial()->getHiddenState(), nullptr));
        }
        this->distribution_serial = discrete_distribution;
    }

    std::shared_ptr<Space> ISerialMMDP::getStateSpace(number t) const
    {
        // std::vector<std::shared_ptr<FunctionSpace<SerialState>>> fspaces;
        // std::vector<std::shared_ptr<Space>> aspaces;
        // for (number i = 0; i < this->getAgentId(t); i++)
        // {
        //     aspaces.push_back(this->mmdp_->getActionSpace(i, t));
        // }
        // for (const auto &state : this->mmdp_->getStateSpace(t))
        // {
        //     fspaces.push_back(std::make_shared<FunctionSpace<SerialState>>({state->toState()}, aspaces));
        // }
        // return this->serial_state_space_.at(this->getAgentId(t));
    }

    std::set<std::shared_ptr<State>> ISerialMMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state, action, t);
    }

    double ISerialMMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &serial_action, number t) const
    {
        return state->getReward(this->mmdp_, serial_action, t);
        // // If the agent t is not the last agent, the reward is 0 else this value is the same as the value of the mmdp.
        // if (!this->isLastAgent(t))
        // {
        //     return 0;
        // }
        // else
        // {
        //     auto serial_state = state->toSerial();
        //     auto joint_action = serial_state->getAction();
        //     joint_action.push_back(serial_action);

        //     return this->mmdp_->getReward(serial_state->getHiddenState(), this->getPointeurJointAction(joint_action), t);
        // }
    }

    double ISerialMMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        auto serial_state = state->toSerial();
        auto next_serial_state = next_state->toSerial();

        auto all_action = serial_state->getAction();
        all_action.push_back(action);

        if (!this->isLastAgent(t))
        {
            // If the next serial_state and the current serial_state don't have the same hidden or it's not the same player to act, then the dynamics is impossible
            return !(((serial_state->getCurrentAgentId() + 1) != next_serial_state->getCurrentAgentId()) ||
                     (serial_state->getHiddenState() != next_serial_state->getHiddenState()) ||
                     (next_serial_state->getAction() != all_action));
        }
        else
        {
            return this->mmdp_->getTransitionProbability(serial_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serial_state->getHiddenState(), t);
        }
    }

} // namespace sdm