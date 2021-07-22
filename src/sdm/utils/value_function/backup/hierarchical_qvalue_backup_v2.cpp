#include <sdm/utils/value_function/backup/hierarchical_qvalue_backup_v2.hpp>

#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>



namespace sdm
{
    HierarchicalQValueBackupV2::HierarchicalQValueBackupV2()
    {

    }

    HierarchicalQValueBackupV2::HierarchicalQValueBackupV2(
        std::shared_ptr<ExperienceMemory> experience_memory, 
        std::shared_ptr<QValueFunction> q_value_table, 
        std::shared_ptr<QValueFunction> target_q_value_table, 
        double discount,
        std::shared_ptr<Space> action_space
    ) : experience_memory_(experience_memory), 
        q_value_table_(std::dynamic_pointer_cast<HierarchicalQValueFunctionV2>(q_value_table)),
        target_q_value_table_(std::dynamic_pointer_cast<HierarchicalQValueFunctionV2>(target_q_value_table)), 
        discount_(discount), 
        action_space_(std::static_pointer_cast<MultiDiscreteSpace>(action_space))
    {
        this->prepareSubordinateJointActions();
    }

    HierarchicalQValueBackupV2::~HierarchicalQValueBackupV2()
    {

    }

    double HierarchicalQValueBackupV2::backup(number t)
    {   
        auto [observation, decision_rule, reward, next_observation] = this->experience_memory_->sample(t)[0];
        auto s_o = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(observation)->first;
        auto s = s_o->first;
        auto o = s_o->second;
        auto ns_no = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(next_observation)->first;
        auto ns = ns_no->first;
        auto no = ns_no->second;
        auto u = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(next_observation)->second;

        double q_value = this->q_value_table_->getQValueAt(s, o, u, t);
        double next_value = this->getValueAt(next_observation->toState(), t + 1);
        double target_q_value = reward + this->discount_ * next_value;
        double delta = target_q_value - q_value;
        this->q_value_table_->updateQValueAt(s, o, u, t, delta);

        return delta;
    }

    std::shared_ptr<Action> HierarchicalQValueBackupV2::getGreedyAction(const std::shared_ptr<State> &state, number t)
    {
        // std::cout << "-------- HierarchicalQValueBackupV2::getGreedyAction() ---------" << std::endl;
        auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(state)->first->first;
        
        s->prepareIndividualHierarchicalHistoryVectors(t);

        std::unordered_map<std::shared_ptr<JointHistoryInterface>, std::unordered_map<std::shared_ptr<Joint<std::shared_ptr<Action>>>, std::shared_ptr<JointHistoryJointActionPair>>> individual_hierarchical_history_subordinate_jaction_finder;

        ///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 1 /////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 1 /////////////////////////////////" << std::endl;

        // Accessible states for the tmp a of agent 1.
        std::vector<std::shared_ptr<Item>> acc_states_tmp_a1;
        // Actions for each of these acc_states_tmp_a1.
        std::vector<std::shared_ptr<Item>> n_actions_tmp_a1;

        for (const auto &individual_hierarchical_history : s->getIndividualHierarchicalHistoriesOf(t, 0))
        {
            individual_hierarchical_history_subordinate_jaction_finder.emplace(individual_hierarchical_history, std::unordered_map<std::shared_ptr<Joint<std::shared_ptr<Action>>>, std::shared_ptr<JointHistoryJointActionPair>>());
            // Here subordinate agents don't include the agent herself.
            for (const auto &subordinate_jaction :  this->all_subordinate_jactions->at(0))
            {
                auto individual_hierarchical_history_subordinate_jaction = std::make_shared<JointHistoryJointActionPair>(individual_hierarchical_history, subordinate_jaction);
                individual_hierarchical_history_subordinate_jaction_finder.at(individual_hierarchical_history).emplace(subordinate_jaction, individual_hierarchical_history_subordinate_jaction);
                acc_states_tmp_a1.push_back(individual_hierarchical_history_subordinate_jaction->toItem());
                auto q_values_agent_u1 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
                for (const auto &u1 : *this->action_space_->get(0))
                {
                    q_values_agent_u1->setValueAt(u1->toAction(), 0);
                    auto u2 = subordinate_jaction->get(0);
                    auto tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                    tmp_u->push_back(u1->toAction());
                    tmp_u->push_back(u2);
                    auto u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();
                    auto o = s->getJointHistory(individual_hierarchical_history);
                    q_values_agent_u1->addValueAt(u1->toAction(), this->q_value_table_->getQValueAt(s, o, u, t));
                }
                auto greedy_u1 = q_values_agent_u1->argmax();
                n_actions_tmp_a1.push_back(greedy_u1->toItem());
            }
        }

        std::shared_ptr<DecisionRule> tmp_a1 = std::make_shared<DeterministicDecisionRule>(acc_states_tmp_a1, n_actions_tmp_a1);

        // std::cout << *tmp_a1 << std::endl;

        ///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 2 /////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE TEMPORARY DECISION RULE OF AGENT 2 /////////////////////////////////" << std::endl;

        // We can skip tmp_a2 since it's the same as a2. In multiagent code it won't be skipped.

        ///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 2 //////////////////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 2 /////////////////////////////////" << std::endl;


        // Accessible states for the a of agent 2.
        std::vector<std::shared_ptr<Item>> acc_states_a2;
        // Actions for each of these acc_states_a2.
        std::vector<std::shared_ptr<Item>> n_actions_a2;

        for (const auto &individual_hierarchical_history : s->getIndividualHierarchicalHistoriesOf(t, 1))
        {
            acc_states_a2.push_back(individual_hierarchical_history->toItem());
            auto q_values_agent_u2 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
            for (const auto &u2 : *this->action_space_->get(1))
            {
                q_values_agent_u2->setValueAt(u2->toAction(), 0);
                for (const auto &joint_history : s->getJointHistories())
                {
                    auto individual_hierarchical_history_agent_1 = s->getIndividualHierarchicalHistory(t, 0, joint_history);
                    std::shared_ptr<Joint<std::shared_ptr<Action>>> subordinate_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                    subordinate_u->push_back(u2->toAction());
                    for (const auto& real_subordinate_u: this->all_subordinate_jactions->at(0))
                    {
                        if (*real_subordinate_u == *subordinate_u)
                        {
                            subordinate_u =  real_subordinate_u;
                            break;
                        }
                    }
                    std::shared_ptr<Joint<std::shared_ptr<Action>>> tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                    auto individual_hierarchical_history_subordinate_jaction_for_agent_1 = individual_hierarchical_history_subordinate_jaction_finder.at(individual_hierarchical_history_agent_1).at(subordinate_u);
                    tmp_u->push_back(tmp_a1->act(individual_hierarchical_history_subordinate_jaction_for_agent_1));
                    tmp_u->push_back(u2->toAction());
                    auto u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();
                    auto o = s->getJointHistory(individual_hierarchical_history_agent_1);
                    q_values_agent_u2->addValueAt(u2->toAction(), this->q_value_table_->getQValueAt(s, o, u, t) * s->getProbability(joint_history));
                }
            }
            auto greedy_u2 = q_values_agent_u2->argmax();
            n_actions_a2.push_back(greedy_u2->toItem());
        }

        std::shared_ptr<DeterministicDecisionRule> a2 = std::make_shared<DeterministicDecisionRule>(acc_states_a2, n_actions_a2);

        // std::cout << *a2 << std::endl;

        ////////////////////////////////////////////// CREATE DECISION RULE OF AGENT 1 //////////////////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE DECISION RULE OF AGENT 1 /////////////////////////////////" << std::endl;

        // Accessible states for the a of agent 1.
        std::vector<std::shared_ptr<Item>> acc_states_a1;
        // Actions for each of these acc_states_a1.
        std::vector<std::shared_ptr<Item>> n_actions_a1;

        for (const auto &individual_hierarchical_history : s->getIndividualHierarchicalHistoriesOf(t, 0))
        {
            // std::cout << "*individual_hierarchical_history " << *individual_hierarchical_history << std::endl;
            auto individual_history_agent_2 = individual_hierarchical_history->toJointHistory()->getIndividualHistory(1);
            std::shared_ptr<JointHistoryInterface> tmp_individual_hierarchical_history_agent_2 = std::make_shared<JointHistoryTree>();
            tmp_individual_hierarchical_history_agent_2->addIndividualHistory(individual_history_agent_2);
            auto individual_hierarchical_history_agent_2 = s->getIndividualHierarchicalHistory(t, 1, tmp_individual_hierarchical_history_agent_2);
            auto u2 = a2->act(individual_hierarchical_history_agent_2);
            acc_states_a1.push_back(individual_hierarchical_history->toItem());
            auto q_values_agent_u1 = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
            for (const auto &u1 : *this->action_space_->get(0))
            {
                q_values_agent_u1->setValueAt(u1->toAction(), 0);
                std::shared_ptr<Joint<std::shared_ptr<Action>>> tmp_u = std::make_shared<Joint<std::shared_ptr<Action>>>();
                tmp_u->push_back(u1->toAction());
                tmp_u->push_back(u2);
                auto u = this->action_space_->getItemAddress(*tmp_u->toJoint<Item>())->toAction();
                auto o = s->getJointHistory(individual_hierarchical_history);
                q_values_agent_u1->addValueAt(u1->toAction(), this->q_value_table_->getQValueAt(s, o, u, t));
            }
            auto greedy_u1 = q_values_agent_u1->argmax();
            n_actions_a1.push_back(greedy_u1->toItem());
        }

        std::shared_ptr<DeterministicDecisionRule> a1 = std::make_shared<DeterministicDecisionRule>(acc_states_a1, n_actions_a1);

        // std::cout << *a1 << std::endl;

        ////////////////////////////////////////////// CREATE JOINT DECISION RULE //////////////////////////////////////////////
        // std::cout << "///////////////////////////////////////////// CREATE JOINT DECISION RULE /////////////////////////////////" << std::endl;


        std::vector<std::shared_ptr<DeterministicDecisionRule>> a;
        a.push_back(a1);
        a.push_back(a2);

        return std::make_shared<JointDeterministicDecisionRule>(a);
    }

    double HierarchicalQValueBackupV2::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<OccupancyStateInterface> s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(state)->first->first;

        std::shared_ptr<DecisionRule> a =  this->getGreedyAction(state, t)->toDecisionRule();

        double value = 0;
        for (const auto& o : s->getJointHistories())
        {
            auto ho = this->getJointHierarchicalHistory(o, s, t);
            for (const auto& u: *this->action_space_)
            {
                // v += s(o) * a(u|o) * Q_s(o, u, t)
                value += s->getProbability(o) * a->getProbability(ho, u->toAction()) * this->target_q_value_table_->getQValueAt(s, o, u->toAction(), t);
            }
        }
        return value;
    }

    void HierarchicalQValueBackupV2::prepareSubordinateJointActions()
    {
        // std::cout << "-------- HierarchicalQValueBackupV2::prepareSubordinateJointActions() ---------" << std::endl;
        // Initialize it.
        this->all_subordinate_jactions = std::make_shared<std::unordered_map<int, std::vector<std::shared_ptr<Joint<std::shared_ptr<Action>>>>>>();
        // Fill it up with empty vectors for each agent.
        for (int agent = 0; agent < this->num_agents_; agent++)
        {
            this->all_subordinate_jactions->emplace(agent, std::vector<std::shared_ptr<Joint<std::shared_ptr<Action>>>>{});
        }
        // We do it for agent N-1. (it has jactions of agent N, not for herself)
        for (const auto & u2: *this->action_space_->get(this->num_agents_ - 1))
        {
            std::shared_ptr<Joint<std::shared_ptr<Action>>> jaction = std::make_shared<Joint<std::shared_ptr<Action>>>();
            jaction->push_back(u2->toAction());
            this->all_subordinate_jactions->at(this->num_agents_ - 2).push_back(jaction);
        }
        // // We do it for the rest of the agents.
        // for (int agent = this->num_agents_ - 3; agent >= 0; agent--)
        // {
        //     for (const auto & uI: *this->action_space_->get(agent))
        //     {
        //         for (const auto & subordinate_jaction: this->all_subordinate_jactions->at(agent + 1))
        //         {
        //             std::shared_ptr<Joint<std::shared_ptr<Action>>> jaction = std::make_shared<Joint<std::shared_ptr<Action>>>();
        //             jaction->push_back(uI->toAction());
        //             for (int i = 0; i < subordinate_jaction->getNumAgents(); i++)
        //             {
        //                 jaction->push_back(subordinate_jaction->at(i));
        //             }                    
        //             this->all_subordinate_jactions->at(agent).push_back(jaction);
        //         }
        //     }
        // }
    }

    std::shared_ptr<State> HierarchicalQValueBackupV2::getJointHierarchicalHistory(const std::shared_ptr<JointHistoryInterface> &joint_labels, const std::shared_ptr<State> &ostate, number t) const
    {
        // This is the reversed version of what we want, that is Joint Hierarchical Labels, that is Hierarchical Labels for each agent.
        // Each Hierarchical Label contains Labels for agents between agent I and agent N.
        std::shared_ptr<Joint<std::shared_ptr<JointHistoryInterface>>> joint_hierarchical_labels_reversed = std::make_shared<Joint<std::shared_ptr<JointHistoryInterface>>>();
        // This is what we will use to record Labels of each agent, starting with agent N until agent 1. This is why it's reversed.
        std::shared_ptr<JointHistoryInterface> individual_hierarchical_label_reversed = std::make_shared<JointHistoryTree>();
        // For agent from N-1 till 0 (N till 1):
        for(int agent = this->num_agents_ - 1; agent >= 0; agent--)
        {
            // Push agent I's Label.
            auto individual_label = joint_labels->getIndividualHistory(agent);
            // std::cout << "individual_label " << individual_label << std::endl;
            individual_hierarchical_label_reversed->addIndividualHistory(individual_label->toHistory());
            // This will be in the correct order, that is Labels for agent I till N.
            std::shared_ptr<JointHistoryInterface> individual_hierarchical_label = std::make_shared<JointHistoryTree>();
            //
            for(int i = std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label_reversed)->getNumAgents() - 1; i >= 0; i--)
            {
                individual_hierarchical_label->addIndividualHistory(individual_hierarchical_label_reversed->getIndividualHistory(i));
            }
            //
            for (const std::shared_ptr<JointHistoryInterface>& individual_hierarchical_history: ostate->toOccupancyState()->getIndividualHierarchicalHistoriesOf(t, agent))
            {
                if (*std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_history) == *std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label))
                {
                    individual_hierarchical_label = individual_hierarchical_history;
                    break;
                }
            }
            // Push Hierarchical Label for agent I.
            joint_hierarchical_labels_reversed->push_back(individual_hierarchical_label);
        }
        // This will be in the correct order, that is Hierarchical Labels from agent 1 to N.
        std::shared_ptr<Joint<std::shared_ptr<JointHistoryInterface>>> joint_hierarchical_labels = std::make_shared<Joint<std::shared_ptr<JointHistoryInterface>>>();
        // 
        for (int i = joint_hierarchical_labels_reversed->getNumAgents() - 1; i >= 0; i--)
        {
            joint_hierarchical_labels->push_back(joint_hierarchical_labels_reversed->at(i));
        }
        return joint_hierarchical_labels->toJoint<State>();
    }

}