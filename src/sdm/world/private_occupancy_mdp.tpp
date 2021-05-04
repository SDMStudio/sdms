#include <sdm/world/private_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    PrivateOccupancyMDP<TState, TAction>::PrivateOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    PrivateOccupancyMDP<TState, TAction>::PrivateOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length)
        : OccupancyMDP<TState, TAction>()
    {
        this->dpomdp_ = underlying_dpomdp;

        typename TState::jhistory_type jhist;
        jhist = std::make_shared<typename TState::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (hist_length > 0) ? hist_length : -1);
        std::cout << jhist << std::endl;

        this->initial_history_ = jhist;
        this->initial_state_ = std::make_shared<TState>(this->dpomdp_->getNumAgents() - 1, 0);
        this->current_state_ = std::make_shared<TState>(this->dpomdp_->getNumAgents() - 1, 0);

        for (typename TState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                auto p_x_h = std::make_pair(s, jhist);
                this->initial_state_->setProbabilityAt(p_x_h, this->dpomdp_->getStartDistrib().probabilities()[s]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    }

    template <typename TState, typename TAction>
    PrivateOccupancyMDP<TState, TAction>::PrivateOccupancyMDP(std::string underlying_dpomdp, number hist_length)
        : PrivateOccupancyMDP<TState, TAction>(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> PrivateOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        using individual_decision_rule_t = typename TAction::value_type;

        // Get possible histories for all agents
        auto vector_of_set_all_individual_histories = ostate.getAllIndividualHistories();

        // 
        // std::vector<std::vector<individual_decision_rule_t>> vector_of_individual_decision_rules = {};
        // for (int agent_id = 0; agent_id < this->dpomdp_->getNumAgents() - 1; agent_id++)
        // {
        //     // Get all inputs of the futur IndividualDetDecRule<Joint<ihistory>, action> ( = combination of partial joint histories)


        //     // Build space of possible individual decision rules for agent 'agent_id'
        //     FunctionSpace<individual_decision_rule_t> individual_decision_rule_space(vector_of_individual_histories, this->dpomdp_->getActionSpace()->getSpace(agent_id)->getAll());
        //     // Generate all individual decision rules for agent 'agent_id'
        //     vector_of_individual_decision_rules.push_back(individual_decision_rule_space.getAll());
        // }

        // Get individual decision rules for each agent except agent n
        std::vector<std::vector<individual_decision_rule_t>> vector_of_individual_decision_rules = {};
        for (int agent_id = 0; agent_id < this->dpomdp_->getNumAgents() - 1; agent_id++)
        {
            auto vector_of_individual_histories = sdm::tools::set2vector(vector_of_set_all_individual_histories[agent_id]);
            // Build space of possible individual decision rules for agent 'agent_id'
            FunctionSpace<individual_decision_rule_t> individual_decision_rule_space(vector_of_individual_histories, this->dpomdp_->getActionSpace()->getSpace(agent_id)->getAll());
            // Generate all individual decision rules for agent 'agent_id'
            vector_of_individual_decision_rules.push_back(individual_decision_rule_space.getAll());
        }

        std::vector<TAction> vector_occupancy_actions;
        // For all joint decision rules of other agents (except N)
        for (const auto &joint_decision_rule : MultiDiscreteSpace<typename TAction::value_type>(vector_of_individual_decision_rules).getAll())
        {
            // For all actions of agent n
            for (const auto &action_n : this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll())
            {
                vector_occupancy_actions.push_back(TAction(joint_decision_rule, action_n, this->dpomdp_->getNumAgents() - 1));
            }
        }
        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<TAction>>(vector_occupancy_actions);
    }
}