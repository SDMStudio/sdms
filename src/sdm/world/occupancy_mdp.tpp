#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    OccupancyMDP<TState, TAction>::OccupancyMDP() {}

    template <typename TState, typename TAction>
    OccupancyMDP<TState, TAction>::OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length)
        : BaseOccupancyMDP<TState, TAction>(underlying_dpomdp, hist_length)
    {
        this->initialize(hist_length);
    }

    template <typename TState, typename TAction>
    OccupancyMDP<TState, TAction>::OccupancyMDP(std::string underlying_dpomdp, number hist_length)
        : OccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length) {}

    template <typename TState, typename TAction>
    void OccupancyMDP<TState, TAction>::initialize(number history_length)
    {
        // Initialize history
        this->initial_history_ = std::make_shared<typename TState::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);

        // Initialize empty state
        this->initial_state_ = std::make_shared<TState>();
        this->current_state_ = std::make_shared<TState>();

        // Fill the initial state state
        for (typename TState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                auto p_x_h = std::make_pair(s, this->initial_history_);
                this->initial_state_->setProbabilityAt(p_x_h, this->dpomdp_->getStartDistrib().probabilities()[s]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    }

    template <typename TState, typename TAction>
    std::tuple<TState, std::vector<double>, bool> OccupancyMDP<TState, TAction>::step(TAction joint_idr)
    {
        // Select joint action
        auto jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories()));

        // Do a step on the DecPOMDP and get next observation and rewards
        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);

        // Expand the current history
        this->current_history_ = this->current_history_->expand(next_obs);

        // Compute the next compressed occupancy state
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr);

        // return the new occupancy state and the perceived rewards
        return std::make_tuple(*this->current_state_, rewards, done);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> OccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        using decision_rule_t = typename TAction::value_type;

        // Get possible histories for all agents
        auto vect_i_hist = ostate.getAllIndividualHistories();

        // Get individual decision rules for each agent
        std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
            FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Get joint decision rules for each agent
        std::vector<TAction> vect_j_dr = {};
        for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        {
            vect_j_dr.push_back(TAction(joint_idr));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<TAction>>(vect_j_dr);
    }

    template <typename TState, typename TAction>
    TState OccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number, std::shared_ptr<HSVI<TState, TAction>>, bool compression) const
    {
        // The new compressed occupancy state
        std::shared_ptr<TState> new_compressed_occupancy_state;
        // The new fully uncompressed occupancy state
        std::shared_ptr<TState> new_fully_uncompressed_occupancy_state = std::make_shared<TState>();
        // The new one step left occupancy state
        std::shared_ptr<TState> new_one_step_left_compressed_occupancy_state = std::make_shared<TState>();

        // for all element in the support of the fully uncompressed occupancy state
        for (auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;

            // Get joint action based on a joint decision rule and a joint labels
            auto jaction = joint_idr.act(ostate.getJointLabels(o->getIndividualHistories()));

            for (auto &y : this->dpomdp_->getReachableStates(x, jaction))
            {
                for (auto &z : this->dpomdp_->getReachableObservations(x, jaction, y))
                {
                    // Get the probability of the next couple (next_state, next_joint history)
                    double next_occupancy_measure = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(jaction), this->dpomdp_->getObsSpace()->joint2single(z), y);

                    // If occupancy measure is greater than zero, we build our occupancy states
                    if (next_occupancy_measure > 0)
                    {
                        // Build fully uncompressed occupancy state
                        auto joint_history_next = o->expand(z);
                        new_fully_uncompressed_occupancy_state->addProbabilityAt({y, joint_history_next}, next_occupancy_measure);

                        // Build one step left uncompressed occupancy state
                        auto compressed_joint_history = ostate.getCompressedJointHistory(o);
                        auto compressed_joint_history_next = compressed_joint_history->expand(z);
                        new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, compressed_joint_history_next}, next_occupancy_measure);

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
            new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
            new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
            return *new_compressed_occupancy_state;
        }

        new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
        new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
        return *new_one_step_left_compressed_occupancy_state;
    }

    template <typename TState, typename TAction>
    TState OccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    double OccupancyMDP<TState, TAction>::getReward(const TState &ostate, const TAction &joint_idr) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            auto jaction = joint_idr.act(o->getIndividualHistories());
            r += p_x_o.second * this->dpomdp_->getReward()->getReward(x, this->dpomdp_->getActionSpace()->joint2single(jaction));

            // REPLACE BY IN LATER VERSION
            // r += p_x_o.second * this->dpomdp_->getReward(x, jaction);
        }

        // for (const auto &occupancy_point : occupancy_state)
        // {
        //     auto joint_history = occupancy_state.getHistory(occupancy_point);
        //     auto belief_state = occupancy_state.getBelief(occupancy_point);
        //     auto joint_action = decision_rule.act(joint_history->getIndividualHistories());
        //     total_reward += (belief_state ^ this->dpomdp_->getReward(joint_action));
        // }
        return r;
    }

    // ##############################################################################################################################
    // ##############################################################################################################################
    // ############################ SPECIALISATION FOR BeliefOccupancyState Structure ###############################################
    // ##############################################################################################################################
    // ##############################################################################################################################

#define DECLARATION_BELIEF_OCCUPANCY_STATE OccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>
#define DECLARATION_BELIEF_OCCUPANCY_MDP OccupancyMDP<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    DECLARATION_BELIEF_OCCUPANCY_MDP::OccupancyMDP()
    {
    }

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    DECLARATION_BELIEF_OCCUPANCY_MDP::OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length)
        : BaseOccupancyMDP<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>(underlying_dpomdp, hist_length)
    {
        this->initialize(hist_length);
    }

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    DECLARATION_BELIEF_OCCUPANCY_MDP::OccupancyMDP(std::string underlying_dpomdp, number hist_length)
        : OccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length) {}

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    void DECLARATION_BELIEF_OCCUPANCY_MDP::initialize(number history_length)
    {
        // Initialize history
        this->initial_history_ = std::make_shared<typename state_type::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);

        // Initialize empty occupancy state
        this->initial_state_ = std::make_shared<state_type>();
        this->current_state_ = std::make_shared<state_type>();

        BeliefStateVector belief = this->dpomdp_->getStartDistrib().probabilities();
        auto belief_graph = std::make_shared<typename state_type::state_type::element_type>(belief, this->dpomdp_->getObsDynamics()->getDynamics());

        // Fill the initial state state
        this->initial_state_->setProbabilityAt({belief_graph, this->initial_history_}, 1);

        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    }

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    std::tuple<DECLARATION_BELIEF_OCCUPANCY_STATE, std::vector<double>, bool> DECLARATION_BELIEF_OCCUPANCY_MDP::step(TActionPrescriptor joint_idr)
    {
        // Select joint action
        auto jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories()));

        // Do a step on the DecPOMDP and get next observation and rewards
        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);

        // Expand the current history
        this->current_history_ = this->current_history_->expand(next_obs);

        // Compute the next compressed occupancy state
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr);

        // return the new occupancy state and the perceived rewards
        return std::make_tuple(*this->current_state_, rewards, done);
    }

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    std::shared_ptr<DiscreteSpace<TActionPrescriptor>> DECLARATION_BELIEF_OCCUPANCY_MDP::getActionSpaceAt(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate)
    {
        using decision_rule_t = typename TActionPrescriptor::value_type;

        // Get possible histories for all agents
        auto vect_i_hist = ostate.getAllIndividualHistories();

        // Get individual decision rules for each agent
        std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
            FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Get joint decision rules for each agent
        std::vector<TActionPrescriptor> vect_j_dr = {};
        for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        {
            vect_j_dr.push_back(TActionPrescriptor(joint_idr));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<TActionPrescriptor>>(vect_j_dr);
    }

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    DECLARATION_BELIEF_OCCUPANCY_STATE DECLARATION_BELIEF_OCCUPANCY_MDP::nextState(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr, number, std::shared_ptr<HSVI<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>>, bool compression) const
    {
        // The new compressed occupancy state
        std::shared_ptr<state_type> new_compressed_occupancy_state;
        // The new fully uncompressed occupancy state
        std::shared_ptr<state_type> new_fully_uncompressed_occupancy_state = std::make_shared<state_type>();
        // The new one step left occupancy state
        std::shared_ptr<state_type> new_one_step_left_compressed_occupancy_state = std::make_shared<state_type>();

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

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    DECLARATION_BELIEF_OCCUPANCY_STATE DECLARATION_BELIEF_OCCUPANCY_MDP::nextState(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr, number h, std::shared_ptr<HSVI<DECLARATION_BELIEF_OCCUPANCY_STATE, TActionPrescriptor>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    double DECLARATION_BELIEF_OCCUPANCY_MDP::getReward(const DECLARATION_BELIEF_OCCUPANCY_STATE &ostate, const TActionPrescriptor &joint_idr) const
    {
        double r = 0;
        for (auto const &pair_belief_history_proba : ostate)
        {
            auto const &belief_graph = pair_belief_history_proba.first.first;
            auto const &joint_history = pair_belief_history_proba.first.second;
            auto const &joint_action = joint_idr.act(joint_history->getIndividualHistories());

            // Compute weighted reward
            r += pair_belief_history_proba.second * (belief_graph->getData() ^ this->dpomdp_->getReward()->getReward(this->dpomdp_->getActionSpace()->joint2single(joint_action)));
        }
        return r;
    }

} // namespace sdm
