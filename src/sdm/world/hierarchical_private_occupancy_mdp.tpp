#include <regex>
#include <sdm/parser/parser.hpp>
#include <sdm/world/hierarchical_private_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    HierarchicalPrivateOccupancyMDP<TState, TAction>::HierarchicalPrivateOccupancyMDP() {}

    template <typename TState, typename TAction>
    HierarchicalPrivateOccupancyMDP<TState, TAction>::HierarchicalPrivateOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length)
        : dpomdp_(underlying_dpomdp)
    {
        this->initialize(hist_length);
    }

    template <typename TState, typename TAction>
    HierarchicalPrivateOccupancyMDP<TState, TAction>::HierarchicalPrivateOccupancyMDP(std::string underlying_dpomdp, number hist_length)
    {
        *this = HierarchicalPrivateOccupancyMDP(parser::parse_file(underlying_dpomdp), hist_length);
    }

    template <typename TState, typename TAction>
    void HierarchicalPrivateOccupancyMDP<TState, TAction>::initialize(number history_length)
    {
        // Initialize history
        this->initial_history_ = std::make_shared<typename TState::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (history_length > 0) ? history_length : -1);

        // Initialize empty state
        this->initial_state_ = std::make_shared<TState>(this->dpomdp_->getNumAgents());
        this->current_state_ = std::make_shared<TState>(this->dpomdp_->getNumAgents());

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
    std::tuple<TState, std::vector<double>, bool> HierarchicalPrivateOccupancyMDP<TState, TAction>::step(TAction joint_idr)
    {
        // Select joint action
        auto jaction = joint_idr.act(this->getJointHierarchicalLabels(*this->current_state_, this->current_history_->getIndividualHistories()));
        // auto jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories()));

        // Do a step on the DecPOMDP and get next observation and rewards
        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);

        this->previous_history_ = this->current_history_;
        // Expand the current history
        this->current_history_ = this->current_history_->expand(next_obs);

        observation z_n = next_obs[next_obs.size() - 1];

        // Compute the next compressed occupancy state
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr, z_n);

        // return the new occupancy state and the perceived rewards
        return std::make_tuple(*this->current_state_, rewards, done);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> HierarchicalPrivateOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        using individual_decision_rule_t = typename TAction::first_type::value_type;

        // Get all possible joint histories. This actually corresponds to all possible individual hierarchical histories of agent 1, but we also need it for other
        // other agents, except agent N.
        std::set<typename TState::jhistory_type> all_joint_histories = ostate.getJointHistories();

        std::vector<std::set<Joint<typename TState::ihistory_type>>> all_individual_hierarchical_histories;
        
        // For all agents from N-1 to 1:
        for(number agent = 0; agent < this->dpomdp_->getNumAgents() - 1; agent++){
            std::set<Joint<typename TState::ihistory_type>> individual_hierarchical_histories;
            for(typename TState::jhistory_type joint_history: all_joint_histories){
                Joint<typename TState::ihistory_type> hierarchical_history;
                for(number agent_ = agent; agent_ < this->dpomdp_->getNumAgents(); agent_++){
                    hierarchical_history.push_back(joint_history->getIndividualHistory(agent_));
                }
                individual_hierarchical_histories.emplace(hierarchical_history);
            }
            all_individual_hierarchical_histories.push_back(individual_hierarchical_histories);
        }

        // Get individual decision rules for each agent except agent n
        std::vector<std::vector<individual_decision_rule_t>> vector_of_individual_decision_rules = {};
        for (int agent_id = 0; agent_id < this->dpomdp_->getNumAgents() - 1; agent_id++)
        {
            auto vector_of_individual_histories = sdm::tools::set2vector(all_individual_hierarchical_histories[agent_id]);
            // Build space of possible individual decision rules for agent 'agent_id'
            FunctionSpace<individual_decision_rule_t> individual_decision_rule_space(
                vector_of_individual_histories, 
                this->dpomdp_->getActionSpace()->getSpace(agent_id)->getAll()
            );
            // Generate all individual decision rules for agent 'agent_id'
            vector_of_individual_decision_rules.push_back(individual_decision_rule_space.getAll());
        }

        std::vector<TAction> vector_occupancy_actions;
        // For all joint decision rules of other agents (except N)
        for (const auto &joint_decision_rule : MultiDiscreteSpace<typename TAction::first_type::value_type>(vector_of_individual_decision_rules).getAll())
        {
            // For all actions of agent n
            for (const auto &action_n : this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll())
            {
                vector_occupancy_actions.push_back(TAction(joint_decision_rule, action_n));
            }
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<TAction>>(vector_occupancy_actions);
    }

    template <typename TState, typename TAction>
    TState HierarchicalPrivateOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number z_n, std::shared_ptr<HSVI<TState, TAction>>, bool compression) const
    {
        try
        {
            // The new compressed occupancy state
            std::shared_ptr<TState> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<TState> new_fully_uncompressed_occupancy_state = std::make_shared<TState>(this->dpomdp_->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<TState> new_one_step_left_compressed_occupancy_state = std::make_shared<TState>(this->dpomdp_->getNumAgents());

            // for all element in the support of the fully uncompressed occupancy state
            for (auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
            {

                auto x = p_x_o.first.first;
                auto o = p_x_o.first.second;

                // Get joint action based on a joint decision rule and a joint labels
                auto jaction = joint_idr.act(this->getJointHierarchicalLabels(ostate, o->getIndividualHistories()));

                for (auto &y : this->dpomdp_->getReachableStates(x, jaction))
                {
                    for (auto &z : this->dpomdp_->getReachableObservations(x, jaction, y))
                    {
                        if (z[z.size() - 1] == z_n)
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
        catch (const std::exception &exc)
        {
            std::cout << ostate << std::endl;
            std::cout << joint_idr << std::endl;
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "HierarchicalPrivateOccupancyMDP<TState, TAction>::nextState(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction>
    TState HierarchicalPrivateOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    double HierarchicalPrivateOccupancyMDP<TState, TAction>::getReward(const TState &ostate, const TAction &joint_idr) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            auto jaction = joint_idr.act(this->getJointHierarchicalLabels(ostate, o->getIndividualHistories()));
            r += p_x_o.second * this->dpomdp_->getReward()->getReward(x, this->dpomdp_->getActionSpace()->joint2single(jaction));
        }
        return r;
    }

    template <typename TState, typename TAction>
    TState HierarchicalPrivateOccupancyMDP<TState, TAction>::getInitialState()
    {
        return *this->initial_state_;
    }

    template <typename TState, typename TAction>
    TState HierarchicalPrivateOccupancyMDP<TState, TAction>::reset()
    {
        // Reset the joint history to initial value
        this->current_history_ = this->initial_history_;

        // Reset the occupancy state
        *this->current_state_ = *this->initial_state_;

        // Reset the underlying DecPOMDP
        this->dpomdp_->reset();

        // Return the occupancy (which is the observation in BaseOccupancyMDP formalism)
        return *this->current_state_;
    }

    template <typename TState, typename TAction>
    DiscreteDecPOMDP *HierarchicalPrivateOccupancyMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename TState, typename TAction>
    Joint<Joint<typename TState::ihistory_type>> HierarchicalPrivateOccupancyMDP<TState, TAction>::getJointHierarchicalLabels(const TState &ostate, Joint<typename TState::ihistory_type> individual_histories) const 
    {
        // Get Joint Label, that is Labels for each private history of each agent.
        auto joint_label = ostate.getJointLabels(individual_histories);
        // This is the reversed version of what we want, that is Joint Hierarchical Label, that is Hierarchical Labels for each agent.
        // Each Hierarchical Label contains Labels for agents between agent I and agent N.
        Joint<Joint<typename TState::ihistory_type>> joint_hierarchical_label_reversed;
        // This is what we will use to record Labels of each agent, starting with agent N until agent 1. This is why it's reversed.
        Joint<typename TState::ihistory_type> hierarchical_label_reversed;
        // For agent from N-1 till 0 (N till 1):
        for(int agent = this->dpomdp_->getNumAgents() - 1; agent >= 0; agent--){
            // Push agent I's Label.
            hierarchical_label_reversed.push_back(joint_label.at(agent));
            // This will be in the correct order, that is Labels for agent I till N.
            Joint<typename TState::ihistory_type> hierarchical_label;
            //
            for(int i = hierarchical_label_reversed.size() - 1; i >= 0; i--){
                // 
                hierarchical_label.push_back(hierarchical_label_reversed[i]);
            }
            // Push Hierarchical Label for agent I.
            joint_hierarchical_label_reversed.push_back(hierarchical_label);
        }
        // This will be in the correct order, that is Hierarchical Labels from agent 1 to N.
        Joint<Joint<typename TState::ihistory_type>> joint_hierarchical_labels;
        // 
        for (int i = joint_hierarchical_label_reversed.size() - 1; i > 0; i--){
            //
            joint_hierarchical_labels.push_back(joint_hierarchical_label_reversed[i]);
        }
        return joint_hierarchical_labels;
    }

    template <typename TState, typename TAction>
    Joint<number> HierarchicalPrivateOccupancyMDP<TState, TAction>::getJaction(TAction oaction)
    {   
        return oaction.act(this->getJointHierarchicalLabels(*this->current_state_, this->current_history_->getIndividualHistories()));
    }

    template <typename TState, typename TAction>
    std::vector<Joint<typename TAction::output>> HierarchicalPrivateOccupancyMDP<TState, TAction>::get_vector_lower_ranked_agents_jactions_reversed(number agent)
    {   
        number N = this->dpomdp_->getNumAgents();
        // std::cout << "get_vector_lower_ranked_agents_jactions_reversed()" << std::endl;
        // std::cout << "agent " << agent << std::endl;
        // We shall record number of permutations of actions as we go.
        int n_actions = 1;
        // All posible jactions will be added from action of agent N until N-I where I is the index of the vector.
        std::vector<std::vector<Joint<typename TAction::output>>> vector_vector_lower_ranked_agents_jactions_reversed(N - 1 - agent);
        // std::cout << "vector_vector_lower_ranked_agents_jactions_reversed.size() " << vector_vector_lower_ranked_agents_jactions_reversed.size() << std::endl;
        for (int agent_ = N - 1; agent_ > agent; agent_--)
        {
            number num_actions_agent_ = 0;
            for (const auto & action_agent_: this->dpomdp_->getActionSpace()->getSpace(agent_)->getAll())
            {   
                num_actions_agent_++;
            }
            n_actions = n_actions * num_actions_agent_;
        }
        // std::cout << "n_actions " << n_actions << std::endl;
        // std::cout << "N " << N << std::endl;
        // First we need to do it for agent_ N.
        for (const auto & uN: this->dpomdp_->getActionSpace()->getSpace(N - 1)->getAll())
        {
            // std::cout << "uN " << uN << std::endl;
            Joint<typename TAction::output> lower_ranked_jaction;
            lower_ranked_jaction.push_back(uN);
            vector_vector_lower_ranked_agents_jactions_reversed[0].push_back(lower_ranked_jaction);
        }
        // This will be the index+1 for vector_vector_lower_ranked_agents_jactions_reversed below.
        int i = 0;
        // Then other agent_s, starting with agent_ N-1 until the last agent_ before agent.
        for (int agent_ = N - 2; agent_ > agent; agent_--)
        {
            // std::cout << "agent_ " << agent_ << std::endl;
            i++;
            for(const auto & lower_ranked_agents_jaction_reversed: vector_vector_lower_ranked_agents_jactions_reversed[i - 1])
            {
                for(const auto & uAgent_: this->dpomdp_->getActionSpace()->getSpace(agent_)->getAll())
                {
                    Joint<typename TAction::output> agents_jaction_reversed = lower_ranked_agents_jaction_reversed;
                    agents_jaction_reversed.push_back(uAgent_);
                    vector_vector_lower_ranked_agents_jactions_reversed[i].push_back(agents_jaction_reversed);
                }
            }
        }
        // std::cout << "vector_vector_lower_ranked_agents_jactions_reversed.size() " << vector_vector_lower_ranked_agents_jactions_reversed.size() << std::endl;
        return vector_vector_lower_ranked_agents_jactions_reversed[vector_vector_lower_ranked_agents_jactions_reversed.size() - 1];

    }



} // namespace sdm
