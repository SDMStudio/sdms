#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : serialized_mpomdp_(std::make_shared<SerializedMPOMDP>(underlying_dpomdp))
    {
        typename TState::jhistory_type jhist;
        if (hist_length > 0)
        {
            jhist = std::make_shared<typename TState::jhistory_type::element_type>(this->serialized_mpomdp_->getNumAgents(), hist_length);
        }
        else
        {
            jhist = std::make_shared<typename TState::jhistory_type::element_type>(this->serialized_mpomdp_->getNumAgents());
        }

        this->initial_history_ = jhist;
        this->initial_state_ = std::make_shared<TState>();

        for (const auto s : this->serialized_mpomdp_->getStateSpace(0)->getAll())
        {
            auto x = s.getState();

            if (this->serialized_mpomdp_->getStartDistrib().probabilities()[x] > 0)
            {
                auto p_s_o = std::make_pair(s, jhist);
                this->initial_state_->setProbabilityAt(p_s_o, this->serialized_mpomdp_->getStartDistrib().probabilities()[x]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
        this->current_state_ = this->initial_state_;
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    SerializedMPOMDP *SerializedOccupancyMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->serialized_mpomdp_.get();
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::getInitialState()
    {
        return *this->initial_state_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> SerializedOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        // Get the individual possible histories for the current agent (as vector)
        auto indiv_hist = ostate.getIndividualHistories(ag_id);

        std::vector<typename TState::jhistory_type::element_type::ihistory_type> v_inputs(indiv_hist.begin(), indiv_hist.end());

        // Generate all individual decision rules for agent 'ag_id' (the current agent)
        FunctionSpace<TAction> f_indiv_dr_space(v_inputs, this->serialized_mpomdp_->getActionSpace(ag_id)->getAll());

        // Now we can return a discrete space of all indiv decision rules
        return std::make_shared<DiscreteSpace<TAction>>(f_indiv_dr_space.getAll());
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &indiv_dr, number, std::shared_ptr<HSVI<TState, TAction>>, bool compression) const
    {
        // The new compressed occupancy state
        std::shared_ptr<TState> new_compressed_occupancy_state;

        number ag_id = ostate.getCurrentAgentId();

        // If this is not the last agent. Same fully uncompressed and one step left uncompressed occ state than before
        if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
        {
            new_compressed_occupancy_state = std::make_shared<TState>();
            for (const auto &pair_s_o_proba : ostate)
            {
                auto o = pair_s_o_proba.first.second;
                auto x = pair_s_o_proba.first.first.first;
                // Update list of actions
                auto u = pair_s_o_proba.first.first.second;
                u.push_back(indiv_dr.act(o->getIndividualHistory(ag_id)));
                // Build next serialized state
                typename TState::state_type new_serialized_state(x, u);
                std::cout << new_serialized_state << std::endl;
                // Set next occupancy measure
                new_compressed_occupancy_state->setProbabilityAt({new_serialized_state, o}, pair_s_o_proba.second);
                new_compressed_occupancy_state->finalize();
                new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_compressed_occupancy_state->getptr());
                new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_compressed_occupancy_state->getptr());
            }
            return *new_compressed_occupancy_state;
        }
        else
        {
            // The new fully uncompressed occupancy state
            std::shared_ptr<TState> new_fully_uncompressed_occupancy_state = std::make_shared<TState>();
            // The new one step left occupancy state
            std::shared_ptr<TState> new_one_step_left_compressed_occupancy_state = std::make_shared<TState>();

            for (auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
            {
                // std::cout << p_x_o << std::endl;
                auto o = ostate.getHistory(p_x_o.first);
                auto serialized_state = ostate.getState(p_x_o.first);
                // std::cout << "serialized_state="<< serialized_state << std::endl;
                auto u = serialized_state.second;
                auto u_agent_i = indiv_dr.act(ostate.getLabel(o->getIndividualHistory(ag_id), ag_id));
                u.push_back(u_agent_i);
                // std::cout << "1" << std::endl;
                // std::cout << "serialized_state="<< serialized_state << std::endl;

                for (const auto &y : this->serialized_mpomdp_->getReachableSerialStates(serialized_state, u_agent_i))
                {
                    // std::cout << "2 : y = " << y << std::endl;
                    for (const auto &z : this->serialized_mpomdp_->getReachableObservations(serialized_state, u_agent_i, y))
                    {
                        // std::cout << "3 : z = " << z << std::endl;
                        // std::cout << "x="<<serialized_state <<  " u="<<u_agent_i <<" z="<<  z<< " y=" <<  y << std::endl;
                        // Get the probability of the next couple (next_serialized_state, next_joint history)
                        double next_occupancy_measure = p_x_o.second * this->serialized_mpomdp_->getDynamics(serialized_state, u_agent_i, z, y);

                        // std::cout << "4 : next_occupancy_measure = " << next_occupancy_measure << std::endl;
                        // If occupancy measure is greater than zero, we build our occupancy states
                        if (next_occupancy_measure > 0)
                        {
                            // Build fully uncompressed occupancy state
                            // std::cout << "4.b" << std::endl;
                            auto joint_history_next = o->expand(z);
                            new_fully_uncompressed_occupancy_state->addProbabilityAt({y, joint_history_next}, next_occupancy_measure);
                            // std::cout << "5" << std::endl;

                            // Build one step left uncompressed occupancy state
                            auto compressed_joint_history = ostate.getCompressedJointHistory(o);
                            auto compressed_joint_history_next = compressed_joint_history->expand(z);
                            new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, compressed_joint_history_next}, next_occupancy_measure);
                            // std::cout << "6" << std::endl;

                            // Update next history labels
                            new_one_step_left_compressed_occupancy_state->updateJointLabels(joint_history_next->getIndividualHistories(), compressed_joint_history_next->getIndividualHistories());
                        }
                    }
                }
                // std::cout << "Pass" << std::endl;
            }
            // std::cout << "8" << std::endl;

            // Finalize the one step left compressed occupancy state
            new_one_step_left_compressed_occupancy_state->finalize();

            if (compression)
            {
                // Compress the occupancy state
                new_compressed_occupancy_state = std::make_shared<TState>(new_one_step_left_compressed_occupancy_state->compress());
                new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
                new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);
                return *new_compressed_occupancy_state;
            }
            new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
            new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);

            return *new_one_step_left_compressed_occupancy_state;
        }
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    double SerializedOccupancyMDP<TState, TAction>::getReward(const TState &ostate, const TAction &indiv_dr) const
    {
        double r = 0;
        number ag_id = ostate.getCurrentAgentId();

        if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
        {
            return 0;
        }

        for (auto &p_s_o : ostate)
        {
            auto pair_s_o = p_s_o.first;
            auto o = pair_s_o.second;

            r += p_s_o.second * this->serialized_mpomdp_->getReward(pair_s_o.first, indiv_dr.act(o->getIndividualHistory(ag_id)));
        }
        return r;
    }

    template <typename TState, typename TAction>
    double SerializedOccupancyMDP<TState, TAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &ostate, const TAction &oaction, number t) const
    {
        TState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDP> SerializedOccupancyMDP<TState, TAction>::toMDP()
    {
        return this->serialized_mpomdp_->toMDP();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedOccupancyMDP<TState, TAction>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction>
    bool SerializedOccupancyMDP<TState, TAction>::isSerialized() const
    {
        return true;
    }
} // namespace sdm
