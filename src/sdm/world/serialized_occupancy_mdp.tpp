#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : BaseSerializedOccupancyMDP<TState,TAction>(underlying_dpomdp,hist_length)
    {
        this->initialize(hist_length);
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    void SerializedOccupancyMDP<TState, TAction>::initialize(number history_length)
    {
        this->initial_history_ = std::make_shared<typename TState::jhistory_type::element_type>(this->serialized_mpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);;
        this->initial_state_ = std::make_shared<TState>(this->serialized_mpomdp_->getNumAgents());

        for (const auto s : this->serialized_mpomdp_->getStateSpace(0)->getAll())
        {
            auto x = s.getState();

            if (this->serialized_mpomdp_->getStartDistrib().probabilities()[x] > 0)
            {
                auto p_s_o = std::make_pair(s, this->initial_history_);
                this->initial_state_->setProbabilityAt(p_s_o, this->serialized_mpomdp_->getStartDistrib().probabilities()[x]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
        this->current_state_ = this->initial_state_;

    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::nextStateSerialStep(const TState &ostate, const TAction &indiv_dr) const
    {
        // Get agent identifier 
        number ag_id = ostate.getCurrentAgentId();

        // Init new compressed and fully uncompressed occupancy states, we omit one-step uncompressed occupancy state
        auto new_compressed_occupancy_state = std::make_shared<TState>(this->serialized_mpomdp_->getNumAgents());
        auto new_fully_uncompressed_occupancy_state = std::make_shared<TState>(this->serialized_mpomdp_->getNumAgents());

        // Go over serial states and joint histories of fully uncompressed occupancy state
        for (const auto &pair_s_o_proba : *ostate.getFullyUncompressedOccupancy())
        {
            // Get entries probability, joint history, and hidden state 
            auto prob = pair_s_o_proba.second;
            auto jhistory = ostate.getHistory(pair_s_o_proba.first);
            auto hidden_state = ostate.getHiddenState(pair_s_o_proba.first);

            // Update list of actions
            auto actions_list = ostate.getAction(pair_s_o_proba.first);

            // Get joint label of current joint history 
            auto compressed_jhistory = ostate.getCompressedJointHistory(jhistory);

            actions_list.push_back(indiv_dr.act(compressed_jhistory->getIndividualHistory(ag_id)));

            // Build next serialized state
            typename TState::state_type new_serialized_state(hidden_state, actions_list);

            // Update next serial occupancy state value at pair {new_serialized_state, jhistory}
            new_fully_uncompressed_occupancy_state->setProbabilityAt({new_serialized_state, jhistory}, prob);

            // Add probability mass to pair {new_serialized_state, compressed_joint_history}
            new_compressed_occupancy_state->addProbabilityAt({new_serialized_state, compressed_jhistory}, prob);
            
            // Update next history labels
            new_compressed_occupancy_state->updateJointLabels(jhistory->getIndividualHistories(), compressed_jhistory->getIndividualHistories());
        }

        // Set underlying strautures for fast getters 
        new_compressed_occupancy_state->finalize();

        // Set agent identifier 
        new_compressed_occupancy_state->setAgent(ag_id+1);
        new_fully_uncompressed_occupancy_state->setAgent(ag_id+1);

        // Set fully uncompressed occupancy state 
        new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
        // Set one-step uncompressed occupancy state 
        new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_compressed_occupancy_state->getptr());

        return *new_compressed_occupancy_state;
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &indiv_dr, number, std::shared_ptr<HSVI<TState, TAction>>, bool compression) const
    {
        number ag_id = ostate.getCurrentAgentId();

        // If this is not the last agent. Same fully uncompressed and one step left uncompressed occ state than before
        if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
        {
            return this->nextStateSerialStep(ostate, indiv_dr);
        }
        else
        {
            // The new compressed occupancy state
            std::shared_ptr<TState> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<TState> new_fully_uncompressed_occupancy_state = std::make_shared<TState>(this->serialized_mpomdp_->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<TState> new_one_step_left_compressed_occupancy_state = std::make_shared<TState>(this->serialized_mpomdp_->getNumAgents());

            for (auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
            {
                auto o = ostate.getHistory(p_x_o.first);
                auto serialized_state = ostate.getState(p_x_o.first);
                auto u = ostate.getAction(p_x_o.first);
                auto u_agent_i = indiv_dr.act(ostate.getLabel(o->getIndividualHistory(ag_id), ag_id));
                u.push_back(u_agent_i);

                for ( const auto &y : this->serialized_mpomdp_->getReachableSerialStates(serialized_state, u_agent_i) ) //this->serialized_mpomdp_->getReachableSerialStates(serialized_state, u_agent_i))
                {
                    for ( const auto &z : this->serialized_mpomdp_->getReachableObservations(serialized_state, u_agent_i, y) ) //this->serialized_mpomdp_->getReachableObservations(serialized_state, u_agent_i, y))
                    {
                        // Get the probability of the next couple (next_serialized_state, next_joint history)
                        double prob = p_x_o.second * this->serialized_mpomdp_->getDynamics(serialized_state, u_agent_i, z, y);

                        // If occupancy measure is greater than zero, we build our occupancy states
                        if (prob > 0)
                        {
                            auto joint_history_next = o->expand(z);

                            new_fully_uncompressed_occupancy_state->addProbabilityAt({y, joint_history_next}, prob);

                            auto compressed_joint_history = ostate.getCompressedJointHistory(o);
                            auto compressed_joint_history_next = compressed_joint_history->expand(z);
                            new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, compressed_joint_history_next}, prob);

                            // Update next history labels
                            new_one_step_left_compressed_occupancy_state->updateJointLabels(joint_history_next->getIndividualHistories(), compressed_joint_history_next->getIndividualHistories());
                        }
                    }
                }
            }

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

//     // ##############################################################################################################################
//     // ##############################################################################################################################
//     // ############################ SPECIALISATION FOR BeliefSerialOccupancyState Structure #########################################
//     // ##############################################################################################################################
//     // ##############################################################################################################################

// #define DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE SerializedOccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>
// #define DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP SerializedOccupancyMDP<DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE, TActionPrescriptor>

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::SerializedOccupancyMDP()
//     {
//     }

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length)
//         : BaseSerializedOccupancyMDP<DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE, TActionPrescriptor>(underlying_dpomdp, hist_length)
//     {
//         this->initialize(hist_length);
//     }

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length)
//         : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length) 
//     {}

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     void DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::initialize(number history_length)
//     {
//         // Initialize history
//         this->initial_history_ = std::make_shared<typename state_type::jhistory_type::element_type>(this->serialized_mpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);

//         // Initialize empty occupancy state
//         this->initial_state_ = std::make_shared<state_type>(this->serialized_mpomdp_->getNumAgents());
//         this->current_state_ = std::make_shared<state_type>(this->serialized_mpomdp_->getNumAgents());

//         BeliefStateVector belief = this->serialized_mpomdp_->getStartDistrib().probabilities();
//         auto belief_graph = std::make_shared<typename state_type::state_type::element_type>(belief, this->serialized_mpomdp_->getDynamics());

//         // Fill the initial state state
//         this->initial_state_->setProbabilityAt({belief_graph, this->initial_history_}, 1);

//         this->initial_state_->finalize();
//         this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
//         this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
//     }

//     // template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     // std::shared_ptr<DiscreteSpace<TActionPrescriptor>> DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::getActionSpaceAt(const DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE &ostate)
//     // {

//     //     // Get id of the current agent
//     //     number ag_id = ostate.getCurrentAgentId();

//     //     // Get the individual possible histories for the current agent (as vector)
//     //     auto indiv_hist = ostate.getIndividualHistories(ag_id);

//     //     std::vector<typename state_type::jhistory_type::element_type::ihistory_type> v_inputs(indiv_hist.begin(), indiv_hist.end());

//     //     // Generate all individual decision rules for agent 'ag_id' (the current agent)
//     //     FunctionSpace<TActionPrescriptor> f_indiv_dr_space(v_inputs, this->serialized_mpomdp_->getActionSpace(ag_id)->getAll());

//     //     // Now we can return a discrete space of all indiv decision rules
//     //     return std::make_shared<DiscreteSpace<TActionPrescriptor>>(f_indiv_dr_space.getAll());
//     // }

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::nextState(const DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE &ostate, const TActionPrescriptor &indiv_dr, number, std::shared_ptr<HSVI<DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE, TActionPrescriptor>>, bool compression) const
//     {
//         number ag_id = ostate.getCurrentAgentId();

//         //***********
//         try
//         {
//             // The new compressed occupancy state
//             std::shared_ptr<state_type> new_compressed_occupancy_state;
//             // The new fully uncompressed occupancy state
//             std::shared_ptr<state_type> new_fully_uncompressed_occupancy_state = std::make_shared<state_type>(this->serialized_mpomdp_->getNumAgents());
//             // The new one step left occupancy state
//             std::shared_ptr<state_type> new_one_step_left_compressed_occupancy_state = std::make_shared<state_type>(this->serialized_mpomdp_->getNumAgents());

//                     // Set agent identifier 

//             auto new_ag_id = (ag_id +1) % this->serialized_mpomdp_->getNumAgents();
//             new_compressed_occupancy_state->setAgent( new_ag_id);
//             new_fully_uncompressed_occupancy_state->setAgent(new_ag_id);
//             new_one_step_left_compressed_occupancy_state->setAgent(new_ag_id);

//             for (auto &pair_belief_history_proba : *ostate.getFullyUncompressedOccupancy())
//             {

//                 auto joint_history = ostate.getHistory(pair_belief_history_proba.first);
//                 auto belief_graph = ostate.getState(pair_belief_history_proba.first);
//                 auto actions = ostate.getAction(pair_belief_history_proba.first);

//                 auto u_agent_i = indiv_dr.act(joint_history->getIndividualHistory(ag_id));
//                 actions.push_back(u_agent_i);

//                 auto index_action = this->serialized_mpomdp_->getIndexAction(ag_id,u_agent_i);

//                 for (const auto &joint_observation : this->serialized_mpomdp_->getObsSpace()->getAll())
//                 {
//                     number index_joint_observation = this->serialized_mpomdp_->joint2single(joint_observation);
//                     // b' = M_z_u * b
//                     auto next_belief_graph = belief_graph->expand(index_action, index_joint_observation);

//                     // p(o') = p(o) * p(z | b, a)
//                     double proba_next_history = pair_belief_history_proba.second * belief_graph->getProbability(index_action, index_joint_observation);

//                     if (proba_next_history > 0)
//                     {
//                         auto next_joint_history = joint_history->expand(joint_observation);

//                         // Build fully uncompressed occupancy state
//                         new_fully_uncompressed_occupancy_state->addProbabilityAt({next_belief_graph, next_joint_history}, proba_next_history);

//                         // Build one step left uncompressed occupancy state
//                         auto compressed_joint_history = ostate.getCompressedJointHistory(joint_history);
//                         auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation);
//                         new_one_step_left_compressed_occupancy_state->addProbabilityAt({next_belief_graph, next_compressed_joint_history}, proba_next_history);

//                         // Update next history labels
//                         new_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->getIndividualHistories(), next_compressed_joint_history->getIndividualHistories());
//                     }
//                 }
//             }

//             // Finalize the one step left compressed occupancy state
//             new_one_step_left_compressed_occupancy_state->finalize();

//             if (compression)
//             {
//                 // Compress the occupancy state
//                 new_compressed_occupancy_state = std::make_shared<state_type>(new_one_step_left_compressed_occupancy_state->compress());
//                 new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
//                 new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);
//                 return *new_compressed_occupancy_state;
//             }
//             new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
//             new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);

//             return *new_one_step_left_compressed_occupancy_state;
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::nextState(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::nextState(const DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr, number h, std::shared_ptr<HSVI<DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE, TActionPrescriptor>> hsvi) const
//     {
//         return this->nextState(ostate, joint_idr, h, hsvi, true);
//     }

//     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
//     double DECLARATION_BELIEF_SERIAL_OCCUPANCY_MDP::getReward(const DECLARATION_BELIEF_SERIAL_OCCUPANCY_STATE &ostate, const TActionPrescriptor &indiv_dr) const
//     {
//         double r = 0;
//         number ag_id = ostate.getCurrentAgentId();

//         if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
//         {
//             return 0;
//         }

//         for (auto &pair_belief_history_proba : ostate)
//         {
//             auto const &belief_graph = ostate.getState(pair_belief_history_proba.first);
//             auto const &joint_history = ostate.getHistory(pair_belief_history_proba.first);

//             auto actions = ostate.getAction(pair_belief_history_proba.first);
//             auto const &u_agent_i = indiv_dr.act(joint_history->getIndividualHistory(ag_id));
//             actions.push_back(u_agent_i);

//             // r += pair_belief_history_proba.second * (belief_graph->getData() ^ this->serialized_mpomdp_->getReward()->getReward(this->serialized_mpomdp_->getActionSpace()->joint2single(joint_action)));
//         }
//         return r;
//     }


} // namespace sdm
