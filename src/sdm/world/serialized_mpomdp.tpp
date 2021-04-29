#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    SerializedMPOMDP::SerializedMPOMDP()
    {
    }

    SerializedMPOMDP::SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_mmdp) : decpomdp_(underlying_mmdp)
    {
        this->mmdp_ = this->decpomdp_->toMMDP();

        this->setPlanningHorizon(decpomdp_->getPlanningHorizon());
        this->setDiscount(decpomdp_->getDiscount());
        this->setActionSpace(this->mmdp_->getActionSpace());

        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();

        this->createInitSerialObservationSpace();
        this->createInitReachableObsStateSpace();
    }

    SerializedMPOMDP::SerializedMPOMDP(std::string underlying_mmdp) : SerializedMPOMDP(std::make_shared<DiscreteDecPOMDP>(underlying_mmdp))
    {
    }

    void SerializedMPOMDP::createInitSerialObservationSpace()
    {
        std::vector<std::shared_ptr<DiscreteSpace<number>>> all_observation_space;

        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            auto obs = this->decpomdp_->getObsSpace()->getSpace(ag_id)->getAll();
            std::vector<number> observation = obs;
            observation.push_back(obs.size());

            auto serial_ind_space = std::make_shared<DiscreteSpace<number>>(observation);

            all_observation_space.push_back(serial_ind_space);
        }
        this->serialized_observation_space_ = std::make_shared<MultiDiscreteSpace<number>>(all_observation_space);
    }

    void SerializedMPOMDP::createInitReachableObsStateSpace()
    {
        // for (const auto action : this->getActionSpace()->getAll())
        // {
        //     std::unordered_map<SerializedState, std::set<Joint<number>>> map_serial_state_obs;

        //     for (const auto serialized_state : this->serialized_state_space_->getAll())
        //     {
        //         std::set<Joint<number>> all_obs;

        //         if (serialized_state.getCurrentAgentId() == 0)
        //         {
        //             Joint<number> joint_action(action);
        //             try
        //             {
        //                 for (const auto obs : this->decpomdp_->getReachableObservations(joint_action, serialized_state.getState()))
        //                 {
        //                     all_obs.insert(obs);
        //                 }
        //             }
        //             catch (const std::exception &e)
        //             {
        //             }
        //         }
        //         else
        //         {
        //             all_obs.insert(this->serialized_observation_space_->single2joint(this->serialized_observation_space_->getNumJointItems() - 1));
        //         }
        //         map_serial_state_obs.emplace(serialized_state.getState(), all_obs);
        //     }
        //     this->reachable_obs_state_space.emplace(action, map_serial_state_obs);
        // }
    }

    const std::set<Joint<number>> &SerializedMPOMDP::getReachableObservations(SerializedState serial_state,number serial_action, SerializedState next_serial_state) const
    {
        //return this->reachable_obs_state_space.at(serial_state).at(serial_action).at(next_serial_state);
    }

    std::shared_ptr<SerializedMMDP> SerializedMPOMDP::toMDP()
    {
        return std::make_shared<SerializedMMDP>(this->decpomdp_->toMMDP());
    }

    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMPOMDP::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP::getObsSpace() const
    {
        return this->serialized_observation_space_;
    }

    std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP::getObsSpaceAt(number ag_id) const
    {
        return this->decpomdp_->getObsSpace()->getSpace(ag_id);
    }

    double SerializedMPOMDP::getObservationProbability(const number &action, const Joint<number> &joint_observation, const SerializedState &serialized_state) const
    {
        // if (serialized_state.getCurrentAgentId() != 0)
        // {
        //     return 0;
        // }
        // else
        // {
        //     return this->decpomdp_->getObsDynamics()->getObservationProbability(action, joint_observation, serialized_state.getState());
        // }
    }

    double SerializedMPOMDP::getDynamics(const SerializedState &serialized_state, const number action, const Joint<number> joint_obs, const SerializedState &serialized_state_next) const
    {
        std::vector<number> all_action = serialized_state.getAction();
        all_action.push_back(action);
        if (serialized_state.getCurrentAgentId() != this->getNumAgents() - 1)
        {
            if ((serialized_state.getState() == serialized_state_next.getState()) && (serialized_state_next.getAction() == all_action))
            {
                return 1;
            }
            return 0;
        }
        return this->decpomdp_->getObsDynamics()->getDynamics(serialized_state.getState(), this->getActionSpace()->joint2single(Joint<number>(all_action)), this->getObsSpace()->joint2single(joint_obs), serialized_state_next.getState());
    }

}