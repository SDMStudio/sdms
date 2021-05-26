#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    SerializedMPOMDP::SerializedMPOMDP()
    {
    }

    SerializedMPOMDP::SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_decpomdp) : decpomdp_(underlying_decpomdp)
    {

        // Set parameter for the Serial problem
        this->setPlanningHorizon(this->decpomdp_->getPlanningHorizon());
        this->setDiscount(this->decpomdp_->getDiscount());
        this->setActionSpace(this->decpomdp_->getActionSpace());

        this->setStartDistrib(this->decpomdp_->getStartDistrib());

        // Creation of Serial State
        this->createInitSerializedStateSpace();

        // Creation of Reachable State
        this->createInitReachableStateSpace();

        this->setupObsSpace();
        this->setDynamics();
        this->setReachableObsSpace();
    }

    SerializedMPOMDP::SerializedMPOMDP(std::string filename) : SerializedMPOMDP(std::make_shared<DiscreteDecPOMDP>(filename))
    {
    }

    double SerializedMPOMDP::getDiscount(number t) const
    {
        return (t % this->getNumAgents() == this->getNumAgents() - 1) ? this->decpomdp_->getDiscount() : 1.0;
    }

    void SerializedMPOMDP::setupObsSpace()
    {

        // Set default joint observation : "No Observation"
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            this->empty_serial_observation.push_back(sdm::DEFAULT_OBSERVATION);
        }

        std::vector<std::shared_ptr<DiscreteSpace<observation_type>>> all_observation_space;
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            if ((ag_id + 1) % this->getNumAgents() == 0)
            {
                all_observation_space.push_back(std::make_shared<DiscreteSpace<observation_type>>(this->decpomdp_->getObsSpace()->getAll()));
            }
            else
            {
                all_observation_space.push_back(std::make_shared<DiscreteSpace<observation_type>>(std::vector<observation_type>{this->empty_serial_observation}));
            }
        }
        this->serialized_observation_space_ = std::make_shared<MultiSpace<DiscreteSpace<observation_type>>>(all_observation_space);
    }

    void SerializedMPOMDP::setDynamics()
    {
        // Creation of dynamics and observation_probability
        // Go over serial states
        for (const auto serialized_state : this->serialized_state_space_->getAll())
        {
            this->dynamics.emplace(serialized_state, std::unordered_map<number, std::unordered_map<Joint<number>, std::unordered_map<SerializedState, double>>>());
            this->observation_probability.emplace(serialized_state, std::unordered_map<number, std::unordered_map<Joint<number>, std::unordered_map<SerializedState, double>>>());

            // Go over serial action
            for (const auto serial_action : this->getActionSpace(serialized_state.getCurrentAgentId())->getAll())
            {
                this->dynamics[serialized_state].emplace(serial_action, std::unordered_map<Joint<number>, std::unordered_map<SerializedState, double>>());
                this->observation_probability[serialized_state].emplace(serial_action, std::unordered_map<Joint<number>, std::unordered_map<SerializedState, double>>());

                // Go over joint_obs
                for (const auto joint_obs : this->getObsSpace(serialized_state.getCurrentAgentId())->getAll())
                {
                    this->dynamics[serialized_state][serial_action].emplace(joint_obs, std::unordered_map<SerializedState, double>());
                    this->observation_probability[serialized_state][serial_action].emplace(joint_obs, std::unordered_map<SerializedState, double>());

                    // Go over serial states
                    for (const auto next_serialized_state : this->serialized_state_space_->getAll())
                    {
                        //Update action
                        std::vector<number> all_action(serialized_state.getAction());
                        all_action.push_back(serial_action);

                        double proba_dynamics = 0, proba_observation_probability = 0;

                        // Determine dynamics and observation_probability

                        // If not the new agent
                        if (next_serialized_state.getCurrentAgentId() != 0)
                        {
                            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
                            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
                            proba_dynamics = (this->empty_serial_observation == joint_obs && serialized_state.getState() == next_serialized_state.getState() && next_serialized_state.getAction() == all_action) ? 1 : 0;
                            proba_observation_probability = proba_dynamics;
                        }
                        // If it is new agent, the obs is possible (not empty serial observation) and action corresponding to the number of agent
                        else if (this->decpomdp_->getObsSpace()->contains(joint_obs) && (all_action.size() == this->getNumAgents()))
                        {
                            // The probability is the same of the decpomdp, if the condition are verified
                            proba_dynamics = this->decpomdp_->getObsDynamics()->getDynamics(serialized_state.getState(), this->decpomdp_->getActionSpace()->joint2single(all_action), this->decpomdp_->getObsSpace()->joint2single(joint_obs), next_serialized_state.getState());
                            proba_observation_probability = this->decpomdp_->getObsDynamics()->getObservationProbability(serialized_state.getState(), this->decpomdp_->getActionSpace()->joint2single(all_action), this->decpomdp_->getObsSpace()->joint2single(joint_obs), next_serialized_state.getState());
                        }
                        this->dynamics[serialized_state][serial_action][joint_obs].emplace(next_serialized_state, proba_dynamics);
                        this->observation_probability[serialized_state][serial_action][joint_obs].emplace(next_serialized_state, proba_observation_probability);
                    }
                }
            }
        }
        // auto size = this->serialized_state_space_->getAll().size();

        // // Go over serial action
        // for (number serial_action =0 ; serial_action<this->associate_ag_id_action.size(); serial_action++)
        // {
        //     this->matrix_dynamics.push_back(std::vector<Matrix>());
        //     // Go over joint_obs
        //     for(const auto joint_obs: this->serialized_observation_space_->getAll())
        //     {
        //         this->matrix_dynamics[serial_action].push_back(Matrix(size,size));
        //         // Creation of dynamics and observation_probability
        //         // Go over serial states
        //         for (const auto serialized_state : this->serialized_state_space_->getAll())
        //         {
        //             // Go over serial states
        //             for (const auto next_serialized_state : this->serialized_state_space_->getAll())
        //             {
        //                 //Update action
        //                 std::vector<number> all_action(serialized_state.getAction());
        //                 all_action.push_back(this->associate_ag_id_action[serial_action].second);

        //                 double proba_dynamics =0, proba_observation_probability = 0;

        //                 // Determine dynamics and observation_probability

        //                 // If not the new agent
        //                 if (next_serialized_state.getCurrentAgentId() != 0)
        //                 {
        //                     // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
        //                     // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
        //                     proba_dynamics = (this->empty_serial_observation == joint_obs && serialized_state.getState() == next_serialized_state.getState() && next_serialized_state.getAction() == all_action) ? 1 : 0;
        //                     proba_observation_probability = proba_dynamics;
        //                 }
        //                 // If it is new agent, the obs is possible (not empty serial observation) and action corresponding to the number of agent
        //                 else if( this->decpomdp_->getObsSpace()->contains(joint_obs) && all_action.size() == this->getNumAgents())
        //                 {
        //                     // The probability is the same of the decpomdp, if the condition are verified
        //                     proba_dynamics = this->decpomdp_->getObsDynamics()->getDynamics(serialized_state.getState(), this->decpomdp_->getActionSpace()->joint2single(Joint<number>(all_action)), this->decpomdp_->getObsSpace()->joint2single(joint_obs), next_serialized_state.getState());
        //                     proba_observation_probability = this->decpomdp_->getObsDynamics()->getObservationProbability(serialized_state.getState(), this->decpomdp_->getActionSpace()->joint2single(Joint<number>(all_action)), this->decpomdp_->getObsSpace()->joint2single(joint_obs), next_serialized_state.getState());
        //                 }

        //                 // Tempo ************
        //                 auto index_joint_obs = this->decpomdp_->getObsSpace()->getAll().size();

        //                 this->matrix_dynamics[serial_action][index_joint_obs](serialized_state.getState(),next_serialized_state.getState()) = proba_dynamics;

        //             }
        //         }
        //     }
        // }
    }

    void SerializedMPOMDP::createInitSerializedStateSpace()
    {
        // Vector of all serial state
        std::vector<std::shared_ptr<DiscreteSpace<SerializedState>>> all_serialized_state;

        number n_agents = this->getNumAgents();

        std::vector<std::vector<number>> all_past_action;

        // Go over all agent
        for (int i = 0; i < n_agents; i++)
        {
            // Vector all serial state
            std::vector<SerializedState> serialized_state_i;

            // All possible vector of actions
            std::vector<std::vector<number>> all_new_action;

            //Creation of all possible vector of actions
            if (i > 0)
            {
                // Go over all current serial state
                for (const auto &action : all_past_action)
                {
                    // Add new action to current serial state
                    for (const auto &action_agent_i : this->decpomdp_->getActionSpace()->getSpaces()[i - 1]->getAll())
                    {
                        //Current action
                        std::vector<number> temp_action = action;
                        //Add new action
                        temp_action.push_back(action_agent_i);
                        // Add new possibility in the vector
                        all_new_action.push_back(temp_action);
                    }
                }
            }
            else
            {
                all_new_action.push_back({});
            }

            //Go over all state
            for (const auto &state : this->decpomdp_->getStateSpace()->getAll())
            {
                // Go over all possible vector of actions
                for (const auto &action : all_new_action)
                {
                    // Add new serial state with the state of the problem and vector of action
                    serialized_state_i.push_back(SerializedState(state, action));
                }
            }
            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace<SerializedState>>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }
        this->serialized_state_space_ = std::make_shared<MultiSpace<DiscreteSpace<SerializedState>>>(all_serialized_state);
        this->setStateSpace(this->serialized_state_space_);
    }

    void SerializedMPOMDP::createInitReachableStateSpace()
    {
        for (const auto serialized_state : this->serialized_state_space_->getAll())
        {
            auto x = serialized_state.getState();
            auto u = serialized_state.getAction();
            number agent_identifier = serialized_state.getCurrentAgentId();

            this->reachable_state_space.emplace(serialized_state, std::unordered_map<number, std::set<SerializedState>>());

            for (auto action : this->getActionSpace(agent_identifier)->getAll())
            {
                std::vector<number> serial_action(u);
                serial_action.push_back(action);

                std::set<SerializedState> all_next_serial_state;

                if (agent_identifier + 1 == this->getNumAgents())
                {
                    Joint<number> joint_action(serial_action);

                    try
                    {
                        for (const auto next_state : this->decpomdp_->getReachableStates(x, joint_action))
                        {
                            all_next_serial_state.insert(SerializedState(next_state, std::vector<number>()));
                        }
                    }
                    catch (const std::exception &e)
                    {
                    }
                }
                else
                {
                    all_next_serial_state.insert(SerializedState(x, serial_action));
                }
                this->reachable_state_space[serialized_state].emplace(action, all_next_serial_state);
            }
        }
    }

    void SerializedMPOMDP::setReachableObsSpace()
    {
        // Creation of all reachable Observation Space

        // Go over serial states
        for (const auto serialized_state : this->serialized_state_space_->getAll())
        {
            this->reachable_obs_state_space.emplace(serialized_state, std::unordered_map<number, std::unordered_map<SerializedState, std::set<Joint<number>>>>());

            // Go over serial action
            for (const auto serial_action : this->getActionSpace(serialized_state.getCurrentAgentId())->getAll())
            {
                this->reachable_obs_state_space[serialized_state].emplace(serial_action, std::unordered_map<SerializedState, std::set<Joint<number>>>());

                // Go over joint_obs
                for (const auto next_serialized_state : this->serialized_state_space_->getAll())
                {
                    //Update action
                    Joint<number> joint_action(serialized_state.getAction());
                    joint_action.push_back(serial_action);

                    std::set<Joint<number>> all_obs;

                    // Insert the next reachable Observation State
                    if (next_serialized_state.getCurrentAgentId() == 0 && serialized_state.getCurrentAgentId() == this->getNumAgents() - 1)
                    {
                        try
                        {
                            for (const auto obs : this->decpomdp_->getReachableObservations(serialized_state.getState(), joint_action, next_serialized_state.getState()))
                            {
                                all_obs.insert(obs);
                            }
                        }
                        catch (const std::exception &e)
                        {
                            all_obs.insert(this->empty_serial_observation);
                        }
                    }
                    else
                    {
                        all_obs.insert(this->empty_serial_observation);
                    }
                    this->reachable_obs_state_space[serialized_state][serial_action].emplace(next_serialized_state, all_obs);
                }
            }
        }
    }

    double SerializedMPOMDP::getReward(const SerializedState &serialized_state, const number &serial_action) const
    {
        if (serialized_state.getCurrentAgentId() + 1 != this->getNumAgents())
        {
            return 0;
        }
        else
        {
            std::vector<number> all_action = serialized_state.getAction();
            all_action.push_back(serial_action);

            return this->decpomdp_->getReward(serialized_state.getState(), Joint<number>(all_action));
        }
    }

    double SerializedMPOMDP::getReward(const SerializedState &serialized_state, const Joint<number> &joint_action) const
    {
        if (serialized_state.getAction() != joint_action)
        {
            return 0;
        }
        else
        {
            return this->decpomdp_->getReward(serialized_state.getState(), joint_action);
        }
    }

    std::shared_ptr<Reward> SerializedMPOMDP::getReward() const
    {
        return this->decpomdp_->getReward();
    }

    double SerializedMPOMDP::getTransitionProbability(const SerializedState &s, const number &action, const SerializedState &s_) const
    {
        if (s.getCurrentAgentId() + 1 != this->getNumAgents())
        {
            // If the next serialized_state and the current serialized_state don't have the same hidden or it's not the same player to act, then the dynamics is impossible
            if (s.getCurrentAgentId() + 1 != s_.getCurrentAgentId() or s.getState() != s_.getState())
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            std::vector<number> all_action = s.getAction();
            all_action.push_back(action);
            return this->decpomdp_->getStateDynamics()->getTransitionProbability(s.getState(), this->getJointActionSpace()->joint2single(Joint<number>(all_action)), s_.getState());
        }
    }

    void SerializedMPOMDP::setInternalState(SerializedState new_i_state)
    {
        if (new_i_state.getCurrentAgentId() == 0)
        {
            this->decpomdp_->setInternalState(new_i_state.getState());
            this->internal_state_ = new_i_state;
        }
    }

    void SerializedMPOMDP::setPlanningHorizon(number horizon)
    {
        this->decpomdp_->setPlanningHorizon(horizon);
        this->planning_horizon_ = horizon;
    }

    number SerializedMPOMDP::getNumAgents() const
    {
        return this->decpomdp_->getActionSpace()->getNumSpaces();
    }
    std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP::getActionSpace(number t) const
    {
        return this->decpomdp_->getActionSpace()->getSpace(t % this->getNumAgents());
    }

    std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP::getJointActionSpace() const
    {
        return this->decpomdp_->getActionSpace();
    }

    const std::set<Joint<number>> &SerializedMPOMDP::getReachableObservations(const SerializedState serial_state, const number serial_action, const SerializedState next_serial_state) const
    {
        return this->reachable_obs_state_space.at(serial_state).at(serial_action).at(next_serial_state);
    }

    std::shared_ptr<SerializedMMDP> SerializedMPOMDP::toMDP()
    {
        return std::make_shared<SerializedMMDP>(this->decpomdp_->toMMDP());
    }

    std::shared_ptr<SerializedBeliefMDP<SerializedBeliefState, number, Joint<number>>> SerializedMPOMDP::toBeliefMDP()
    {
        return std::make_shared<SerializedBeliefMDP<SerializedBeliefState, number, Joint<number>>>(this->getptr());
    }

    std::shared_ptr<DiscreteSpace<Joint<number>>> SerializedMPOMDP::getObsSpace(number t) const
    {
        return this->serialized_observation_space_->getSpace(t % this->getNumAgents());
    }

    std::vector<Joint<number>> SerializedMPOMDP::getObsSpaceAt(number ag_id) const
    {
        return (ag_id % this->getNumAgents() == 0) ? std::vector<Joint<number>>{this->empty_serial_observation} : this->decpomdp_->getObsSpace()->getAll();
    }

    double SerializedMPOMDP::getObservationProbability(const SerializedState serialized_state, const number action, const Joint<number> joint_obs, const SerializedState serialized_state_next) const
    {
        return this->observation_probability.at(serialized_state).at(action).at(joint_obs).at(serialized_state_next);
    }

    double SerializedMPOMDP::getDynamics(const SerializedState serialized_state, const number action, const Joint<number> joint_obs, const SerializedState serialized_state_next) const
    {
        return this->dynamics.at(serialized_state).at(action).at(joint_obs).at(serialized_state_next);
    }

    std::shared_ptr<SerializedMPOMDP> SerializedMPOMDP::getptr()
    {
        return std::static_pointer_cast<SerializedMPOMDP>(SerializedMMDPStructure::getptr());
    }

    // const Joint<number> SerializedMPOMDP::getObservation(number index)
    // {
    //     return this->serialized_observation_space_->getAll().at(index);
    // }

    // std::vector<std::vector<Matrix>> SerializedMPOMDP::getDynamics()
    // {
    //     return this->matrix_dynamics;
    // }

    number SerializedMPOMDP::joint2single(const Joint<number> joint_obs)
    {
        return this->decpomdp_->getObsSpace()->joint2single(joint_obs);
    }
}