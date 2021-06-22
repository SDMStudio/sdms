// #include <regex>
// #include <sdm/parser/parser.hpp>
// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/utils/struct/pair.hpp>
// #include <sdm/core/space/function_space.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    OccupancyMDP::OccupancyMDP() {}

    OccupancyMDP::OccupancyMDP(std::shared_ptr<MPOMDPInterface> underlying_dpomdp, number hist_length)
        : BeliefMDP(underlying_dpomdp)
    {
        this->initialize(hist_length);
    }

    OccupancyMDP::OccupancyMDP(std::string underlying_dpomdp, number hist_length)
    {
        *this = OccupancyMDP(parser::parse_file(underlying_dpomdp), hist_length);
    }

    void OccupancyMDP::initialize(number history_length)
    {
        // Initialize history
        this->initial_history_ = std::make_shared<typename std::shared_ptr<State>::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);

        // Initialize empty state
        this->initial_state_ = std::make_shared<std::shared_ptr<State>>(this->dpomdp_->getNumAgents());
        this->current_state_ = std::make_shared<std::shared_ptr<State>>(this->dpomdp_->getNumAgents());

        // Fill the initial state state
        for (const typename std::shared_ptr<State>::state_type &s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                const auto &p_x_h = std::make_pair(s, this->initial_history_);
                this->initial_state_->setProbabilityAt(p_x_h, this->dpomdp_->getStartDistrib().probabilities()[s]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> OccupancyMDP::step(std::shared_ptr<Action> joint_idr)
    {
        // Select joint action
        const auto &jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories()));

        // Do a step on the DecPOMDP and get next observation and rewards
        const auto &[next_obs, rewards, done] = this->dpomdp_->step(jaction);

        // Expand the current history
        this->current_history_ = this->current_history_->expand(next_obs);

        // Compute the next compressed occupancy state
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr);

        // return the new occupancy state and the perceived rewards
        return std::make_tuple(*this->current_state_, rewards, done);
    }

    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<State> &ostate)
    {
        std::vector<std::shared_ptr<Space>> vector_indiv_space;
        for (int agent_id = 0; agent_id < num_agents; agent_id++)
        {
            // Get history space of agent i
            auto history_space_i = std::make_shared<DiscreteSpace>(ostate.getIndividualHistories(agent_id));
            // Get action space of agent i
            auto action_space_i = std::static_pointer_cast<Joint<std::shared_ptr<Space>>>(this->getUnderlyingProblem()->getActionSpace())->get(agent_id);
            // Add individual decision rule space of agent i
            vector_indiv_space.push_back(std::make_shared<FunctionSpace<DeterministicDecisionRule>>(history_space_i, action_space_i, false));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<MultiDiscreteSpace>(vector_indiv_space, false);
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyMDP::nextState(const std::shared_ptr<State> &state_tmp, const std::shared_ptr<Action> &action_tmp, number t, std::shared_ptr<HSVI>, bool compression) const
    {
        try
        {
            auto ostate = state_tmp->toOccupancyState();
            auto joint_idr = action_tmp->toJointDeterministicDecisionRule();
            // The new compressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_fully_uncompressed_occupancy_state; // = std::make_shared<std::shared_ptr<OccupancyStateInterface>>(this->getUnderlyingMPOMDP()->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<OccupancyStateInterface> new_one_step_left_compressed_occupancy_state; // = std::make_shared<std::shared_ptr<OccupancyStateInterface>>(this->getUnderlyingMPOMDP()->getNumAgents());

            for (const auto &joint_history : ostate->getFullyUncompressedOccupancy()->getJointHistories())
            {
                for (const auto &belief : ostate->getStatesAt(joint_history))
                {
                    auto joint_action = joint_idr->act(ostate->getJointLabels(joint_history->getIndividualHistories()));

                    for (auto &joint_observation : this->getUnderlyingMPOMDP()->getReachableObservations(belief, joint_action, belief, t))
                    {
                        auto next_belief = BeliefMDP::nextState(belief, joint_action, joint_observation, t);

                        // p(o') = p(o) * p(z | b, a)
                        double proba_next_history = ostate->getProbability({belief, joint_history}) * BeliefMDP::getObservationProbability(belief, joint_action, joint_observation, next_belief, t);

                        if (proba_next_history > 0)
                        {
                            auto next_joint_history = joint_history->expand(joint_observation, joint_action);

                            // Build fully uncompressed occupancy state
                            new_fully_uncompressed_occupancy_state->addProbabilityAt({next_belief_graph, next_joint_history}, proba_next_history);

                            // Build one step left uncompressed occupancy state
                            auto compressed_joint_history = ostate.getCompressedJointHistory(joint_history);
                            
                            auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation);
                            new_one_step_left_compressed_occupancy_state->addProbabilityAt({next_belief_graph, next_compressed_joint_history}, proba_next_history);

                            // Update next history labels
                            new_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->getIndividualHistories(), next_compressed_joint_history->getIndividualHistories());
                        }

                        for (auto &next_state : this->getUnderlyingMPOMDP()->getReachableStates(state, jaction, t))
                        {
                            // Get the probability of the next couple (next_state, next_joint history)
                            double next_occupancy_measure = ostate->getProbability(std::make_pair(state, joint_history)) * this->getUnderlyingMPOMDP()->getDynamics(state, jaction, next_state, joint_obs, t);

                            // If occupancy measure is greater than zero, we build our occupancy states
                            if (next_occupancy_measure > 0)
                            {
                                // Build fully uncompressed occupancy state
                                auto joint_history_next = joint_history->expand(joint_obs);
                                new_fully_uncompressed_occupancy_state->addProbability(std::make_pair(next_state, joint_history_next), next_occupancy_measure);

                                // Build one step left uncompressed occupancy state
                                auto compressed_joint_history = ostate->getCompressedJointHistory(joint_history);
                                auto compressed_joint_history_next = compressed_joint_history->expand(joint_obs);
                                new_one_step_left_compressed_occupancy_state->addProbability(std::make_pair(next_state, compressed_joint_history_next), next_occupancy_measure);

                                // Update next history labels
                                new_one_step_left_compressed_occupancy_state->updateJointLabels(joint_history_next->getIndividualHistories(), compressed_joint_history_next->getIndividualHistories());
                            }
                        }
                    }
                }
            }

            // Finalize the one step left compressed occupancy state
            new_one_step_left_compressed_occupancy_state->finalize();

            if (compression)
            {
                // Compress the occupancy state
                new_compressed_occupancy_state = new_one_step_left_compressed_occupancy_state->compress();
                new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
                new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);

                return std::make_shared<OccupancyStateInterface>(new_compressed_occupancy_state);
            }

            new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
            new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);
            return std::make_shared<OccupancyStateInterface>(new_one_step_left_compressed_occupancy_state);
        }
        catch (const std::exception &exc)
        {
            std::cout << state_tmp->str() << std::endl;
            std::cout << action_tmp->str() << std::endl;
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "OccupancyMDP::nextState(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    std::shared_ptr<State> OccupancyMDP::nextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &joint_idr, number h, std::shared_ptr<HSVI> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    double OccupancyMDP::getReward(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &joint_idr) const
    {
        double r = 0;
        for (const auto &joint_history : ostate->toOccupancyState()->getJointHistories())
        {
            auto jaction = joint_idr->act(ostate->getJointLabels(joint_history->getIndividualHistories()));
            for (const auto &state : ostate->getStatesAt(joint_history))
            {
                r += ostate->toOccupancyState()->getProbability({state, joint_history}) * this->getUnderlyingMPOMDP()->getReward(state, jaction);
            }
        }
        return r;
    }

    std::shared_ptr<MPOMDPInterface> OccupancyMDP::getUnderlyingMPOMDP() const
    {
        return std::dynamic_pointer_cast<MPOMDPInterface>(this->getUnderlyingMDP());
    }

    // ##############################################################################################################################
    // ##############################################################################################################################
    // ############################ SPECIALISATION FOR BeliefOccupancyState Structure ###############################################
    // ##############################################################################################################################
    // ##############################################################################################################################

    // #define DECLARATION_BELIEF_OCCUPANCY_STATE OccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>
    // #define DECLARATION_BELIEF_OCCUPANCY_MDP OccupancyMDP<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     DECLARATION_BELIEF_OCCUPANCY_MDP::OccupancyMDP()
    //     {
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     DECLARATION_BELIEF_OCCUPANCY_MDP::OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length)
    //         : BaseOccupancyMDP<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>(underlying_dpomdp, hist_length)
    //     {
    //         this->initialize(hist_length);
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     DECLARATION_BELIEF_OCCUPANCY_MDP::OccupancyMDP(std::string underlying_dpomdp, number hist_length)
    //         : OccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length) {}

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     void DECLARATION_BELIEF_OCCUPANCY_MDP::initialize(number history_length)
    //     {
    //         // Initialize history
    //         this->initial_history_ = std::make_shared<typename state_type::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);

    //         // Initialize empty occupancy state
    //         this->initial_state_ = std::make_shared<state_type>(this->dpomdp_->getNumAgents());
    //         this->current_state_ = std::make_shared<state_type>(this->dpomdp_->getNumAgents());

    //         BeliefStateVector belief = this->dpomdp_->getStartDistrib().probabilities();
    //         auto belief_graph = std::make_shared<typename state_type::state_type::element_type>(belief, this->dpomdp_->getObsDynamics()->getDynamics());

    //         // Fill the initial state state
    //         this->initial_state_->setProbabilityAt({belief_graph, this->initial_history_}, 1);

    //         this->initial_state_->finalize();
    //         this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
    //         this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     std::tuple<DECLARATION_BELIEF_OCCUPANCY_STATE, std::vector<double>, bool> DECLARATION_BELIEF_OCCUPANCY_MDP::step(TActionPrescriptor joint_idr)
    //     {
    //         // Select joint action
    //         const auto &jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories()));

    //         // Do a step on the DecPOMDP and get next observation and rewards
    //         const auto &[next_obs, rewards, done] = this->dpomdp_->step(jaction);

    //         // Expand the current history
    //         this->current_history_ = this->current_history_->expand(next_obs);

    //         // Compute the next compressed occupancy state
    //         *this->current_state_ = this->nextState(*this->current_state_, joint_idr);

    //         // return the new occupancy state and the perceived rewards
    //         return std::make_tuple(*this->current_state_, rewards, done);
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     std::shared_ptr<DiscreteSpace<TActionPrescriptor>> DECLARATION_BELIEF_OCCUPANCY_MDP::getActionSpaceAt(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate)
    //     {
    //         using decision_rule_t = typename TActionPrescriptor::value_type;

    //         // Get possible histories for all agents
    //         const auto &vect_i_hist = ostate.getAllIndividualHistories();

    //         // Get individual decision rules for each agent
    //         std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
    //         for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
    //         {
    //             // Generate all individual decision rules for agent 'ag_id'
    //             const auto &vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
    //             FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
    //             vect_i_dr.push_back(f_indiv_dr_space.getAll());
    //         }

    //         // Get joint decision rules for each agent
    //         std::vector<TActionPrescriptor> vect_j_dr = {};
    //         for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
    //         {
    //             vect_j_dr.push_back(TActionPrescriptor(joint_idr));
    //         }

    //         // Now we can return a discrete space of all joint decision rules
    //         return std::make_shared<DiscreteSpace<TActionPrescriptor>>(vect_j_dr);
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     DECLARATION_BELIEF_OCCUPANCY_STATE DECLARATION_BELIEF_OCCUPANCY_MDP::nextState(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr, number, std::shared_ptr<HSVI<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>>, bool compression) const
    //     {
    try
    {
        // The new compressed occupancy state
        std::shared_ptr<state_type> new_compressed_occupancy_state;
        // The new fully uncompressed occupancy state
        std::shared_ptr<state_type> new_fully_uncompressed_occupancy_state = std::make_shared<state_type>(this->dpomdp_->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<state_type> new_one_step_left_compressed_occupancy_state = std::make_shared<state_type>(this->dpomdp_->getNumAgents());

        // for all element in the support of the fully uncompressed occupancy state
        for (auto const &pair_belief_history_proba : *ostate.getFullyUncompressedOccupancy())
        {
            auto belief_graph = pair_belief_history_proba.first.first;
            auto joint_history = pair_belief_history_proba.first.second;

            // Get joint action based on a joint decision rule and a joint labels
            auto joint_action = joint_idr.act(ostate.getJointLabels(joint_history->getIndividualHistories()));
            number index_joint_action = this->dpomdp_->getActionSpace()->joint2single(joint_action);

            for (const auto &joint_observation : this->dpomdp_->getObsSpace()->getAll())
            {
                number index_joint_observation = this->dpomdp_->getObsSpace()->joint2single(joint_observation);
                // b' = M_z_u * b
                auto next_belief_graph = belief_graph->expand(index_joint_action, index_joint_observation);

                // p(o') = p(o) * p(z | b, a)
                double proba_next_history = pair_belief_history_proba.second * belief_graph->getProbability(index_joint_action, index_joint_observation);

                if (proba_next_history > 0)
                {
                    auto next_joint_history = joint_history->expand(joint_observation);

                    // Build fully uncompressed occupancy state
                    new_fully_uncompressed_occupancy_state->addProbabilityAt({next_belief_graph, next_joint_history}, proba_next_history);

                    // Build one step left uncompressed occupancy state
                    auto compressed_joint_history = ostate.getCompressedJointHistory(joint_history);
                    auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation);
                    new_one_step_left_compressed_occupancy_state->addProbabilityAt({next_belief_graph, next_compressed_joint_history}, proba_next_history);

                    // Update next history labels
                    new_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->getIndividualHistories(), next_compressed_joint_history->getIndividualHistories());
                }
            }
        }

        // Finalize the one step left compressed occupancy state
        new_one_step_left_compressed_occupancy_state->finalize();

        if (compression)
        {
            // Compress the occupancy state
            new_compressed_occupancy_state = std::make_shared<state_type>(new_one_step_left_compressed_occupancy_state->compress());
            new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
            new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
            return *new_compressed_occupancy_state;
        }

        new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
        new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
        return *new_one_step_left_compressed_occupancy_state;
    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << "DECLARATION_BELIEF_OCCUPANCY_MDP::nextState(..) exception caught: " << exc.what() << std::endl;
        exit(-1);
    }

    //         // try
    //         // {
    //         //     // The new compressed occupancy state
    //         //     std::shared_ptr<state_type> new_compressed_occupancy_state;
    //         //     // The new fully uncompressed occupancy state
    //         //     std::shared_ptr<state_type> new_fully_uncompressed_occupancy_state = std::make_shared<state_type>(this->dpomdp_->getNumAgents());
    //         //     // The new one step left occupancy state
    //         //     std::shared_ptr<state_type> new_one_step_left_compressed_occupancy_state = std::make_shared<state_type>(this->dpomdp_->getNumAgents());

    //         //     // for all element in the support of the fully uncompressed occupancy state
    //         //     for (const auto &pair_belief_history_proba : *ostate.getFullyUncompressedOccupancy())
    //         //     {
    //         //         const auto &belief_graph = pair_belief_history_proba.first.first;
    //         //         const auto &joint_history = pair_belief_history_proba.first.second;

    //         //         // Get joint action based on a joint decision rule and a joint labels
    //         //         const auto &joint_action = joint_idr.act(ostate.getJointLabels(joint_history->getIndividualHistories()));
    //         //         number index_joint_action = this->dpomdp_->getActionSpace()->joint2single(joint_action);

    //         //         for (const auto &joint_observation : this->dpomdp_->getObsSpace()->getAll())
    //         //         {
    //         //             number index_joint_observation = this->dpomdp_->getObsSpace()->joint2single(joint_observation);
    //         //             // b' = M_z_u * b
    //         //             const auto &next_belief_graph = belief_graph->expand(index_joint_action, index_joint_observation);

    //         //             // p(o') = p(o) * p(z | b, a)
    //         //             double next_proba = pair_belief_history_proba.second * belief_graph->getProbability(index_joint_action, index_joint_observation);

    //         //             if (next_proba > 0)
    //         //             {

    //         //                 // Get next joint history from uncompressed joint history
    //         //                 const auto &next_uncompressed_joint_history = joint_history->expand(joint_observation);
    //         //                 if (this->compress)
    //         //                 {
    //         //                     // Get next joint history from joint history labels
    //         //                     const auto &next_compressed_joint_history = ostate.getCompressedJointHistory(joint_history)->expand(joint_observation);

    //         //                     if (this->keep_fully_uncompressed)
    //         //                     {
    //         //                         // Update next history labels
    //         //                         new_one_step_left_compressed_occupancy_state->updateJointLabels(next_uncompressed_joint_history->getIndividualHistories(), next_compressed_joint_history->getIndividualHistories());

    //         //                         // Update fully uncompressed occupancy state
    //         //                         new_fully_uncompressed_occupancy_state->addProbabilityAt({next_belief_graph, next_uncompressed_joint_history}, next_proba);
    //         //                     }

    //         //                     // Update one step left uncompressed occupancy state
    //         //                     new_one_step_left_compressed_occupancy_state->addProbabilityAt({next_belief_graph, next_compressed_joint_history}, next_proba);
    //         //                 }
    //         //                 else
    //         //                 {
    //         //                     // Update one step left uncompressed occupancy state
    //         //                     new_one_step_left_compressed_occupancy_state->addProbabilityAt({next_belief_graph, next_uncompressed_joint_history}, next_proba);
    //         //                 }
    //         //             }
    //         //         }
    //         //     }

    //         //     // Finalize the one step left compressed occupancy state
    //         //     new_one_step_left_compressed_occupancy_state->finalize();

    //         //     if (this->compress)
    //         //     {
    //         //         // Compress the occupancy state
    //         //         new_compressed_occupancy_state = std::make_shared<state_type>(new_one_step_left_compressed_occupancy_state->compress());

    //         //         // Store one step uncompressed
    //         //         new_compressed_occupancy_state->setOneStepUncompressedOccupancy((this->keep_one_step_uncompressed) ? new_one_step_left_compressed_occupancy_state->getptr() : new_compressed_occupancy_state->getptr());

    //         //         // Store fully uncompressed
    //         //         new_compressed_occupancy_state->setFullyUncompressedOccupancy((this->keep_fully_uncompressed) ? new_fully_uncompressed_occupancy_state->getptr() : new_compressed_occupancy_state->getOneStepUncompressedOccupancy());

    //         //         return *new_compressed_occupancy_state;
    //         //     }

    //         //     new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
    //         //     new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy((this->keep_fully_uncompressed) ? new_fully_uncompressed_occupancy_state->getptr() : new_one_step_left_compressed_occupancy_state->getptr());
    //         //     return *new_one_step_left_compressed_occupancy_state;
    //         // }
    //         // catch (const std::exception &exc)
    //         // {
    //         //     // catch anything thrown within try block that derives from std::exception
    //         //     std::cerr << "DECLARATION_BELIEF_OCCUPANCY_MDP::nextState(..) exception caught: " << exc.what() << std::endl;
    //         //     exit(-1);
    //         // }
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     DECLARATION_BELIEF_OCCUPANCY_STATE DECLARATION_BELIEF_OCCUPANCY_MDP::nextState(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr, number h, std::shared_ptr<HSVI<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>> hsvi) const
    //     {
    //         return this->nextState(ostate, joint_idr, h, hsvi, true);
    //     }

    //     template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    //     double DECLARATION_BELIEF_OCCUPANCY_MDP::getReward(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr) const
    //     {
    //         double r = 0;
    //         for (const auto &pair_belief_history_proba : ostate)
    //         {
    //             const auto &belief_graph = pair_belief_history_proba.first.first;
    //             const auto &joint_history = pair_belief_history_proba.first.second;
    //             const auto &joint_action = joint_idr.act(joint_history->getIndividualHistories());

    //             // Compute weighted reward
    //             r += pair_belief_history_proba.second * (belief_graph->getData() ^ this->dpomdp_->getReward()->getReward(this->dpomdp_->getActionSpace()->joint2single(joint_action)));
    //         }
    //         return r;
    //     }

} // namespace sdm
